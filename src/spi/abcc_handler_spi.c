/*******************************************************************************
** Copyright 2013-present HMS Industrial Networks AB.
** Licensed under the MIT License.
********************************************************************************
** File Description:
** This file implements the ABCC_DunDriver() and ABCC_ISR() routine for SPI
** operating mode.
********************************************************************************
*/

#include "abcc_config.h"

#if ABCC_CFG_DRV_SPI_ENABLED

#include "abcc_types.h"
#include "../abcc_driver_interface.h"
#include "abp.h"
#include "abcc.h"
#include "../abcc_link.h"
#include "abcc_hardware_abstraction.h"
#include "abcc_log.h"
#include "../abcc_handler.h"
#include "../abcc_timer.h"
#include "../abcc_command_sequencer.h"

#include <inttypes.h>
#include <time.h>

/*------------------------------------------------------------------------------
** pnABCC_DrvRun()
**------------------------------------------------------------------------------
*/
#define SPI_RUN_TIMING_ENABLED        0
#define SPI_RUN_TIMING_REPORT_EVERY   10000U

#if SPI_RUN_TIMING_ENABLED
/*
** Accumulator for one measurement point.
** Kept file-local to this function via block-static arrays below.
*/
typedef struct
{
   UINT64 llSumNs;
   UINT64 llMinNs;
   UINT64 llMaxNs;
} spi_run_TimingBucket;

static inline UINT64 spi_run_TimespecToNs( const struct timespec* ps )
{
   return ( (UINT64)ps->tv_sec * 1000000000ULL ) + (UINT64)ps->tv_nsec;
}
#endif

/*------------------------------------------------------------------------------
** pnABCC_DrvRun()
**------------------------------------------------------------------------------
*/
void ABCC_SpiRunDriver( void )
{
   ABCC_MainStateType eMainState;

#if SPI_RUN_TIMING_ENABLED
   /*
   ** One bucket per phase. Order matches the PHASE_* enum below.
   ** Buckets are reset together after each report.
   */
   enum {
      PHASE_CHECK_WRPD = 0,
      PHASE_LINK_CHECK_SEND,
      PHASE_DRV_TX,                 /* includes the blocking SPI ioctl */
      PHASE_LINK_RX,
      PHASE_TRIGGER_RDPD,
      PHASE_TRIGGER_ANB,
      PHASE_TRIGGER_RX_MSG,
      PHASE_CMD_SEQ,
      PHASE_TOTAL,
      PHASE_COUNT
   };

   static const char* const pacPhaseNames[ PHASE_COUNT ] = {
      "CheckWrPd",
      "LinkCheckSendMsg",
      "DrvRunTx(SPI ioctl)",
      "LinkRunDriverRx",
      "TriggerRdPdUpdate",
      "TriggerAnbStatusUpdate",
      "TriggerReceiveMessage",
      "CmdSequencerExec",
      "TOTAL"
   };

   static spi_run_TimingBucket sBuckets[ PHASE_COUNT ] = { { 0, (UINT64)-1, 0 } };
   static BOOL                 fBucketsInited         = FALSE;
   static UINT32               lSampleCnt             = 0;

   struct timespec sPhaseStart, sPhaseEnd, sCallStart;
   UINT64          llPhaseNs;

   if( !fBucketsInited )
   {
      int i;
      for( i = 0; i < PHASE_COUNT; i++ )
      {
         sBuckets[ i ].llSumNs = 0;
         sBuckets[ i ].llMinNs = (UINT64)-1;
         sBuckets[ i ].llMaxNs = 0;
      }
      fBucketsInited = TRUE;
   }

   #define SPI_RUN_PHASE_BEGIN()                                        \
      do { (void)clock_gettime( CLOCK_MONOTONIC_RAW, &sPhaseStart ); }  \
      while( 0 )

   #define SPI_RUN_PHASE_END( phaseId )                                           \
      do {                                                                        \
         if( clock_gettime( CLOCK_MONOTONIC_RAW, &sPhaseEnd ) == 0 )               \
         {                                                                        \
            llPhaseNs = spi_run_TimespecToNs( &sPhaseEnd )                         \
                      - spi_run_TimespecToNs( &sPhaseStart );                      \
            sBuckets[ (phaseId) ].llSumNs += llPhaseNs;                            \
            if( llPhaseNs < sBuckets[ (phaseId) ].llMinNs )                        \
            {                                                                     \
               sBuckets[ (phaseId) ].llMinNs = llPhaseNs;                          \
            }                                                                     \
            if( llPhaseNs > sBuckets[ (phaseId) ].llMaxNs )                        \
            {                                                                     \
               sBuckets[ (phaseId) ].llMaxNs = llPhaseNs;                          \
            }                                                                     \
         }                                                                        \
      } while( 0 )

   (void)clock_gettime( CLOCK_MONOTONIC_RAW, &sCallStart );
#else
   #define SPI_RUN_PHASE_BEGIN()      ((void)0)
   #define SPI_RUN_PHASE_END( phase ) ((void)0)
#endif

   eMainState = ABCC_GetMainState();

   if( eMainState < ABCC_DRV_SETUP )
   {
      if( eMainState != ABCC_DRV_ERROR )
      {
         ABCC_LOG_ERROR( ABCC_EC_INCORRECT_STATE,
            (UINT32)eMainState,
            "ABCC_RunDriver() called in incorrect state (%d)\n",
            eMainState );
      }
      return;
   }

   SPI_RUN_PHASE_BEGIN();
   ABCC_CheckWrPdUpdate();
   SPI_RUN_PHASE_END( PHASE_CHECK_WRPD );

   SPI_RUN_PHASE_BEGIN();
   ABCC_LinkCheckSendMessage();
   SPI_RUN_PHASE_END( PHASE_LINK_CHECK_SEND );

   /*
   ** Send MOSI frame
   */
   SPI_RUN_PHASE_BEGIN();
   pnABCC_DrvRunDriverTx();
   SPI_RUN_PHASE_END( PHASE_DRV_TX );

#if ABCC_CFG_SYNC_MEASUREMENT_IP_ENABLED
   /*
   ** We have now finished sending data to the Anybus and thus we end the
   ** sync measurement.
   */
   if( fAbccUserSyncMeasurementIp )
   {
      ABCC_HAL_GpioReset();
      fAbccUserSyncMeasurementIp = FALSE;
   }
#endif

   /*
   ** Handle received MISO frame
   */
   SPI_RUN_PHASE_BEGIN();
   ABCC_LinkRunDriverRx();
   SPI_RUN_PHASE_END( PHASE_LINK_RX );

   SPI_RUN_PHASE_BEGIN();
   ABCC_TriggerRdPdUpdate();
   SPI_RUN_PHASE_END( PHASE_TRIGGER_RDPD );

   SPI_RUN_PHASE_BEGIN();
   ABCC_TriggerAnbStatusUpdate();
   SPI_RUN_PHASE_END( PHASE_TRIGGER_ANB );

   SPI_RUN_PHASE_BEGIN();
   ABCC_TriggerReceiveMessage();
   SPI_RUN_PHASE_END( PHASE_TRIGGER_RX_MSG );

#if ABCC_CFG_DRV_CMD_SEQ_ENABLED
   SPI_RUN_PHASE_BEGIN();
   ABCC_CmdSequencerExec();
   SPI_RUN_PHASE_END( PHASE_CMD_SEQ );
#endif

#if SPI_RUN_TIMING_ENABLED
   {
      struct timespec sCallEnd;
      if( clock_gettime( CLOCK_MONOTONIC_RAW, &sCallEnd ) == 0 )
      {
         UINT64 llTotalNs = spi_run_TimespecToNs( &sCallEnd )
                          - spi_run_TimespecToNs( &sCallStart );
         sBuckets[ PHASE_TOTAL ].llSumNs += llTotalNs;
         if( llTotalNs < sBuckets[ PHASE_TOTAL ].llMinNs )
         {
            sBuckets[ PHASE_TOTAL ].llMinNs = llTotalNs;
         }
         if( llTotalNs > sBuckets[ PHASE_TOTAL ].llMaxNs )
         {
            sBuckets[ PHASE_TOTAL ].llMaxNs = llTotalNs;
         }
      }

      lSampleCnt++;

      if( lSampleCnt >= SPI_RUN_TIMING_REPORT_EVERY )
      {
         int i;
         ABCC_LOG_INFO(
            "ABCC_SpiRunDriver timing over %" PRIu32 " calls (ns):\n",
            lSampleCnt );

         for( i = 0; i < PHASE_COUNT; i++ )
         {
            UINT64 llAvgNs;
            UINT64 llMinOut;

            /*
            ** A bucket that was never entered (e.g. CmdSeq disabled) still has
            ** min=UINT64_MAX from init; print 0 instead of a nonsense huge value.
            */
            if( sBuckets[ i ].llMinNs == (UINT64)-1 )
            {
               llMinOut = 0;
               llAvgNs  = 0;
            }
            else
            {
               llMinOut = sBuckets[ i ].llMinNs;
               llAvgNs  = sBuckets[ i ].llSumNs / (UINT64)lSampleCnt;
            }

            ABCC_LOG_INFO(
               "  %-24s min=%8" PRIu64 " avg=%8" PRIu64 " max=%8" PRIu64 "\n",
               pacPhaseNames[ i ],
               llMinOut,
               llAvgNs,
               sBuckets[ i ].llMaxNs );

            sBuckets[ i ].llSumNs = 0;
            sBuckets[ i ].llMinNs = (UINT64)-1;
            sBuckets[ i ].llMaxNs = 0;
         }

         lSampleCnt = 0;
      }
   }
#endif

   #undef SPI_RUN_PHASE_BEGIN
   #undef SPI_RUN_PHASE_END
}

#if ABCC_CFG_INT_ENABLED
void ABCC_SpiISR()
{
   ABCC_MainStateType eMainState;

   eMainState = ABCC_GetMainState();

   if( eMainState < ABCC_DRV_WAIT_COMMUNICATION_RDY )
   {
      return;
   }

   if( eMainState == ABCC_DRV_WAIT_COMMUNICATION_RDY )
   {
      ABCC_SetReadyForCommunication();
      return;
   }

   ABCC_CbfEvent( 0 );
}
#else
void ABCC_SpiISR()
{
   ABCC_LOG_WARNING( ABCC_EC_INTERNAL_ERROR,
      0,
      "ABCC_SpiISR() called when ABCC_CFG_INT_ENABLED is 0\n" );
}
#endif
#endif /* ABCC_CFG_DRV_SPI_ENABLED */
