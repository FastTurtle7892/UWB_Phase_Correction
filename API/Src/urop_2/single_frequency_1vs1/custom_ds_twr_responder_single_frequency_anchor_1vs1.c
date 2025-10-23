/*! ----------------------------------------------------------------------------
 *  @file    ds_twr_responder.c
 *  @brief   Double-sided two-way ranging (DS TWR) responder example code
 *
 *           This is a simple code example which acts as the responder in a DS TWR distance measurement exchange. This application waits for a "poll"
 *           message (recording the RX time-stamp of the poll) expected from the "DS TWR initiator" example code (companion to this application), and
 *           then sends a "response" message recording its TX time-stamp, after which it waits for a "final" message from the initiator to complete
 *           the exchange. The final message contains the remote initiator's time-stamps of poll TX, response RX and final TX. With this data and the
 *           local time-stamps, (of poll RX, response TX and final RX), this example application works out a value for the time-of-flight over-the-air
 *           and, thus, the estimated distance between the two devices, which it writes to the LCD.
 *
 * @attention
 *
 * Copyright 2015-2020 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include <stdlib.h>
#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <example_selection.h>
#include <config_options.h>
#include <shared_function_jang.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
#define UNIT_TEST 0
#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256  


#if defined(CUSTOM_DS_TWR_RESPONDER_SINGLE_FREQUENCY_ANCHOR_1VS1)

extern void test_run_info(unsigned char *data);

/* Example application name */
#define APP_NAME "DS TWR RESP v1.0"

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {

    9,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (128 + 1 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0      /* PDOA mode off */
};

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 0

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */

// channel 9 : 16348
// channel 5 : 16345

#define TX_ANT_DLY 16345
#define RX_ANT_DLY 16345


double CH9_wavelength = 299792458.0 / (2 * 7.9872e9);
double CH5_wavelength = 299792458.0 / (2 * 6.4896e9);

/* Frames used in the ranging process. See NOTE 2 below. */
//static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
//static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
//static uint8_t rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0};
static uint8_t rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t rx_post_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x12};
/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 40
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

// origin 
// POLL_RX_TO_RESP_TX_DLY_UUS 2000 | RESP_TX_TO_FINAL_RX_DLY_UUS 1800 | FINAL_RX_TIMEOUT_UUS 291 | FINAL_RX_POST_FINAL_RX_DLY_UUS 1850 | POST_FINAL_TIMEOUT_UUS 288

// 850K
// POLL_RX_TO_RESP_TX_DLY_UUS 2100 | RESP_TX_TO_FINAL_RX_DLY_UUS 1815 | FINAL_RX_TIMEOUT_UUS 506 | FINAL_RX_POST_FINAL_RX_DLY_UUS 1920 | POST_FINAL_TIMEOUT_UUS 482


/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW IC's delayed TX function. This includes the
 * frame length of approximately 190 us with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 1650
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW IC's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 1500
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 298

#define FINAL_RX_POST_FINAL_RX_DLY_UUS 1500


#define POST_FINAL_TIMEOUT_UUS 279

// state = 0 : now channel 5
// state = 1 : now channel 9
static int state = 0;

/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 5

/* Timestamps of frames transmission/reception. */
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
static uint64_t final_rx_ts;
static uint64_t post_final_rx_ts = 0;

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */


double cir_phase[] = {0, 0, 0, 0};

extern dwt_txconfig_t txconfig_options_ch9;

static dwt_rxdiag_t rx_diag;

#define ACCUM_DATA_LEN (3 + 3) + 1
static uint8_t accum_data[ACCUM_DATA_LEN]; //org array

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn ds_twr_responder()
 *
 * @brief Application entry point.

 *
 * @param  none
 *
 * @return none
 */

char uart_send_data[10];
#define UART_HWFC APP_UART_FLOW_CONTROL_ENABLED

static bool first_cycle_check = false;

int custom_ds_twr_responder_single_frequency_anchor_1vs1(void)
{
    /* Display application name on LCD. */
    test_run_info((unsigned char *)APP_NAME);

    /* Configure SPI rate, DW3000 supports up to 38 MHz */
    port_set_dw_ic_spi_fastrate();

    /* Reset DW IC */
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC

    uint32_t err_code;

    double tof;
    double distance;
    double raw_rec_phase = 0;
    double raw_rec_phase_dist_change = 0;
    double filtered_rec_phase_dist = 0;
    int raw_N = 0;
    int filtered_N = 0;
    double new_distance = 0;

    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          UART_HWFC,
          false,
          NRF_UART_BAUDRATE_1000000
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);


    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
    { };

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED     ");
        while (1)
        { };
    }


    if(dwt_configure(&config)) /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1)
        { };
    }

    dwt_configuretxrf(&txconfig_options_ch9);
    dwt_setrxantennadelay(TX_ANT_DLY);
    dwt_settxantennadelay(RX_ANT_DLY);
    

    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    dwt_configciadiag(1);

    for (int i = 0 ; i < 24; i++ ){rx_buffer[i] = 0;}
    for (int i = 0 ; i < ACCUM_DATA_LEN; i++ ){accum_data[i] = 0;}

    dwt_setpreambledetecttimeout(0);
    dwt_setrxtimeout(0);

    int cnt = 0;
    memset(&rx_diag, 0, sizeof(rx_diag));

    MedianFilter_N filter_n; // N value
    init_filter_N(&filter_n);

    MedianFilter_PHASE filter_p; // phase value
    init_filter_PHASE(&filter_p);


    while (cnt < 1000)
    {

        if (!first_cycle_check)
          dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        {};

        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
        {
            
            uint32_t frame_len;

            /* Clear good RX frame event in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
            if (frame_len <= RX_BUF_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }
 
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                //printf("\n\npoll recv ok");
                uint32_t resp_tx_time;
                int ret;

                poll_rx_ts = get_rx_timestamp_u64();

                resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(resp_tx_time);

                tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
                dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
                ret = dwt_starttx(DWT_START_TX_DELAYED);

                dwt_readdiagnostics(&rx_diag);
                uint16_t fp_int = rx_diag.ipatovFpIndex >> 6;
                dwt_readaccdata(accum_data, ACCUM_DATA_LEN, fp_int);
                cir_phase[0] = calculate_CIR(accum_data, ACCUM_DATA_LEN);

                /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
                if (ret == DWT_SUCCESS)
                {
                    //printf("\nresposne transmit");

                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
                      { };

                      /* Clear TXFRS event. */
                      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

                      /* Increment frame sequence number after transmission of the final message (modulo 256). */
                      frame_seq_nb++;
                    
                      resp_tx_ts = get_tx_timestamp_u64();
                      uint32_t final_rx_time = (resp_tx_ts + (RESP_TX_TO_FINAL_RX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8; 
                      dwt_setdelayedtrxtime(final_rx_time);
                      dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

                      dwt_rxenable(DWT_START_RX_DELAYED); 

               
                      /* Poll for reception of expected "final" frame or error/timeout. See NOTE 8 below. */
                      while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
                      { };

                      /* Increment frame sequence number after transmission of the response message (modulo 256). */
                      frame_seq_nb++;

                      if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
                      {
                    
                          /* Clear good RX frame event and TX frame sent in the DW IC status register. */
                          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

                          /* A frame has been received, read it into the local buffer. */
                          frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
                          if (frame_len <= RX_BUF_LEN)
                          {
                              dwt_readrxdata(rx_buffer, frame_len, 0);
                          }

                          rx_buffer[ALL_MSG_SN_IDX] = 0;

                          if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
                          {
                              //printf("\nfinal recv ok");
                              uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
                              uint8_t temp[8];

                              /* Retrieve response transmission and final reception timestamps. */

                              final_rx_ts = get_rx_timestamp_u64();

                              uint32_t post_final_rx_time;
                              post_final_rx_time = (final_rx_ts + (FINAL_RX_POST_FINAL_RX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8; 
                              dwt_setdelayedtrxtime(post_final_rx_time);
                              dwt_setrxtimeout(POST_FINAL_TIMEOUT_UUS);

                              dwt_rxenable(DWT_START_RX_DELAYED); 

                              /* Get timestamps embedded in the final message. */
                              final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                              final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
                              final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                              snprintf(temp, sizeof(temp), "%s", &rx_buffer[22]);

                              cir_phase[1] = strtod(temp, NULL);

                              dwt_readdiagnostics(&rx_diag);
                              fp_int = rx_diag.ipatovFpIndex >> 6;
                              dwt_readaccdata(accum_data, ACCUM_DATA_LEN, fp_int);
                              cir_phase[2] = calculate_CIR(accum_data, ACCUM_DATA_LEN);


                              while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
                              { };
                        
                              /* Increment frame sequence number after transmission of the response message (modulo 256). */
                              if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
                              {
                            
                                  /* Clear good RX frame event and TX frame sent in the DW IC status register. */
                                  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

                                  /* A frame has been received, read it into the local buffer. */
                                  frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
                                  if (frame_len <= RX_BUF_LEN)
                                  {
                                      dwt_readrxdata(rx_buffer, frame_len, 0);
                                  }
     
                                  rx_buffer[ALL_MSG_SN_IDX] = 0;

                                  if (memcmp(rx_buffer, rx_post_final_msg, ALL_MSG_COMMON_LEN) == 0)
                                  {    
                                    //printf("\npost final recv ok");

                                    first_cycle_check = true;

                                    post_final_rx_ts = get_rx_timestamp_u64();

                                    uint32_t poll_rx_time;
                                    poll_rx_time = (post_final_rx_ts + (1500 * UUS_TO_DWT_TIME)) >> 8;
                                    dwt_setdelayedtrxtime(poll_rx_time);
                                    dwt_setrxtimeout(281);

                                    dwt_rxenable(DWT_START_RX_DELAYED);

                                    
                                    dwt_readdiagnostics(&rx_diag);
                                    fp_int = rx_diag.ipatovFpIndex >> 6;

                                    dwt_readaccdata(accum_data, ACCUM_DATA_LEN, fp_int);
                                    cir_phase[3] = calculate_CIR(accum_data, ACCUM_DATA_LEN);
                              

                                   /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */

                                    uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                                    double Ra, Rb, Da, Db;
                                    int64_t tof_dtu;

                                    poll_rx_ts_32 = (uint32_t)poll_rx_ts;
                                    resp_tx_ts_32 = (uint32_t)resp_tx_ts;
                                    final_rx_ts_32 = (uint32_t)final_rx_ts;
                                    Ra = (double)(resp_rx_ts - poll_tx_ts);
                                    Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                                    Da = (double)(final_tx_ts - resp_rx_ts);
                                    Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                                    tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                                    tof = tof_dtu * DWT_TIME_UNITS;

                                    distance = tof * SPEED_OF_LIGHT;

                                    // phase
                                    raw_rec_phase = normalize_phase(cir_phase[0] + cir_phase[1] - cir_phase[2] + cir_phase[3]);
                                    raw_rec_phase_dist_change = raw_rec_phase * CH9_wavelength / (2*PI);
                                    filtered_rec_phase_dist = process_value_PHASE(&filter_p, raw_rec_phase_dist_change);

                                    // N
                                    raw_N = (int)(distance / CH9_wavelength); 
                                    filtered_N = process_value_N(&filter_n, raw_N);

                                    // recovery dist
                                    new_distance = (double)filtered_N * CH9_wavelength + filtered_rec_phase_dist;

                                    if(cnt > 500)
                                      printf("\n%d|%f|%f", cnt, distance, new_distance);

                                    cnt++;



                                 }
                              }
                          }
                          else
                          {
                              /* Clear RX error/timeout events in the DW IC status register. */
                              dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
                        
                          }
                       }
                  }
             }
             else
             {
                /* Clear RX error/timeout events in the DW IC status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
                
             }
          }

          raw_rec_phase = 0;
          raw_rec_phase_dist_change = 0;
          filtered_rec_phase_dist = 0;
          raw_N = 0;
          filtered_N = 0;
          new_distance = 0;
         
          for(int i=0; i<4; i++)
            cir_phase[i] = 0; 
    }
}
#endif
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW IC.
 * 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 4. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *    and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 *    Initiator: |Poll TX| ..... |Resp RX| ........ |Final TX|
 *    Responder: |Poll RX| ..... |Resp TX| ........ |Final RX|
 *                   ^|P RMARKER|                                    - time of Poll TX/RX
 *                                   ^|R RMARKER|                    - time of Resp TX/RX
 *                                                      ^|R RMARKER| - time of Final TX/RX
 *
 *                       <--TDLY->                                   - POLL_TX_TO_RESP_RX_DLY_UUS (RDLY-RLEN)
 *                               <-RLEN->                            - RESP_RX_TIMEOUT_UUS   (length of poll frame)
 *                    <----RDLY------>                               - POLL_RX_TO_RESP_TX_DLY_UUS (depends on how quickly responder
 *                                                                                                                      can turn around and reply)
 *
 *
 *                                        <--T2DLY->                 - RESP_TX_TO_FINAL_RX_DLY_UUS (R2DLY-FLEN)
 *                                                  <-FLEN--->       - FINAL_RX_TIMEOUT_UUS   (length of response frame)
 *                                    <----RDLY--------->            - RESP_RX_TO_FINAL_TX_DLY_UUS (depends on how quickly initiator
 *                                                                                                                      can turn around and reply)
 *
 * EXAMPLE 1: with SPI rate set to 18 MHz (default on this platform), and frame lengths of ~190 us, the delays can be set to:
 *            POLL_RX_TO_RESP_TX_DLY_UUS of 400uus, and RESP_RX_TO_FINAL_TX_DLY_UUS of 400uus (TXtoRX delays are set to 210uus)
 *            reducing the delays further can be achieved by using interrupt to handle the TX/RX events, or other code optimisations/faster SPI
 *
 * EXAMPLE 2: with SPI rate set to 4.5 MHz, and frame lengths of ~190 us, the delays can be set to:
 *            POLL_RX_TO_RESP_TX_DLY_UUS of 550uus, and RESP_RX_TO_FINAL_TX_DLY_UUS of 600uus (TXtoRX delays are set to 360 and 410 uus respectively)
 *
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete final frame sent by the responder at the
 *    6.81 Mbps data rate used (around 200 us).
 * 6. The preamble timeout allows the receiver to stop listening in situations where preamble is not starting (which might be because the responder is
 *    out of range or did not receive the message to respond to). This saves the power waste of listening for a message that is not coming. We
 *    recommend a minimum preamble timeout of 5 PACs for short range applications and a larger value (e.g. in the range of 50% to 80% of the preamble
 *    length) for more challenging longer range, NLOS or noisy environments.
 * 7. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 8. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW IC User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 9. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to poll RX
 *    timestamp to get response transmission time. The delayed transmission time resolution is 512 device time units which means that the lower 9 bits
 *    of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower 8 bits.
 * 10. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *     automatically appended by the DW IC. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *     work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 11. When running this example on the DWK3000 platform with the POLL_RX_TO_RESP_TX_DLY response delay provided, the dwt_starttx() is always
 *     successful. However, in cases where the delay is too short (or something else interrupts the code flow), then the dwt_starttx() might be issued
 *     too late for the configured start time. The code below provides an example of how to handle this condition: In this case it abandons the
 *     ranging exchange and simply goes back to awaiting another poll message. If this error handling code was not here, a late dwt_starttx() would
 *     result in the code flow getting stuck waiting subsequent RX event that will will never come. The companion "initiator" example (ex_05a) should
 *     timeout from awaiting the "response" and proceed to send another poll in due course to initiate another ranging exchange.
 * 12. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *     subtraction.
 * 13. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW IC API Guide for more details on the DW IC driver functions.
 * 14. In this example, the DW IC is put into IDLE state after calling dwt_initialise(). This means that a fast SPI rate of up to 38 MHz can be used
 *     thereafter.
 * 15. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *     configuration.
 ****************************************************************************************************************************************************/