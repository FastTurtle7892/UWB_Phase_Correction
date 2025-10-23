/*! ----------------------------------------------------------------------------
 *  @file    simple_tx.c
 *  @brief   Simple TX example code
 *
 * @attention
 *
 * Copyright 2015 - 2020 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <example_selection.h>
//-----------------------------UART or SPI_arduino : hearder---------------------------------
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"


#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "$abcdefgh\n"
static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
//static uint8_t       m_rx_buf[30];
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */
static int count = 1;
static uint32_t pin_num = 1;
static uint32_t state;
/**
 * @brief SPI user event handler.
 * @param events
 */

//void nordic_state_gpio_pin_setting(void){

//  NRF_GPIO_Type * reg = ((NRF_GPIO_Type*)NRF_P1_BASE);
//  reg->PIN_CNF[pin_num] = ((uint32_t)GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
//                            |((uint32_t)GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
//                            |((uint32_t)GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)
//                            |((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
//                            |((uint32_t)GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

//}

//uint32_t gpio_pin_state_checking(void){

//   NRF_GPIO_Type * reg = ((NRF_GPIO_Type*)NRF_P1_BASE);
//   return(((reg->IN)>>pin_num) & 1UL);

//}



//void spi_event_handler1(nrf_drv_spi_evt_t const * p_event,
//                       void *                    p_context)
//{
//    spi_xfer_done = true;
//   // NRF_LOG_INFO("Transfer completed(for master)!!!!!.\n");
//    if (m_rx_buf[0] != 0)
//    {
//        //NRF_LOG_INFO(" Received:");
//        //NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
//    }
//    printf("spi_event_handler!");
//}

//void spi_arduino(void)
//{     
//      //printf("arduino_spi start  ");

//      // Reset rx buffer and transfer done flag
//      memset(m_rx_buf, 0, m_length);
//      spi_xfer_done = false;

//      //printf("arduino_spi start2");
//      APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));

//      while (!spi_xfer_done)
//      {
//          __WFE();
//      }
//     // printf("arduino_spi start3");
//      count++;
//     // printf(" %s ",m_rx_buf);
//      printf(" '%d' \n",count);
//     // printf("arduino_spi start4");

//      //NRF_LOG_FLUSH();    // using -> error

//      bsp_board_led_invert(BSP_BOARD_LED_0);
//      nrf_delay_ms(10);
//     // printf("arduino_spi end");

//}
//void spi_arduino_setup(void)
//{
//   // printf("arduino_spi setup start ");
//    bsp_board_init(BSP_INIT_LEDS);
//    //APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
//    //NRF_LOG_DEFAULT_BACKENDS_INIT();
//    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
//    spi_config.ss_pin   = 31; //31
//    spi_config.miso_pin = 30; //30
//    spi_config.mosi_pin = 29; //29
//    spi_config.sck_pin  = 26; //26
//    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler1, NULL));
//    //printf("arduino_spi setup end ");
//}

//-----------------------------UART or SPI_arduino : hearder---------------------------------

#if defined(SIMPLE_TX_CP)

extern void test_run_info(unsigned char *data);

/* Example application name */
#define APP_NAME "SIMPLE TX v1.0"

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
    (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,  /* STS length, see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0      /* PDOA mode off */
};

/* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID, see NOTE 1 below.
 */
static uint8_t tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E'};
/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

#define FRAME_LENGTH    (sizeof(tx_msg)+FCS_LEN) //The real length that is going to be transmitted

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 100

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 2 below. */
extern dwt_txconfig_t txconfig_options;

/**
 * Application entry point.
 */
int simple_tx_cp(void)
{  


    /* Display application name on LCD. */
    test_run_info((unsigned char *)APP_NAME);

    /* Configure SPI rate, DW3000 supports up to 38 MHz */
    port_set_dw_ic_spi_fastrate();
    spi_arduino_setup();// setting spi to arduino


    /* Reset DW IC */
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
    { };

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED     ");
        while (1)
        { };
    }

    /* Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards. */
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK) ;

    /* Configure DW IC. See NOTE 5 below. */
    if(dwt_configure(&config)) /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1)
        { };
    }

    /* Configure the TX spectrum parameters (power PG delay and PG Count) */
    dwt_configuretxrf(&txconfig_options);
    nordic_state_gpio_pin_setting();     // gpio_pin_state_setting

    /* Loop forever sending frames periodically. */
    while(1)
    {

        /* Write frame data to DW IC and prepare transmission. See NOTE 3 below.*/
        dwt_writetxdata(FRAME_LENGTH-FCS_LEN, tx_msg, 0); /* Zero offset in TX buffer. */

        /* In this example since the length of the transmitted frame does not change,
         * nor the other parameters of the dwt_writetxfctrl function, the
         * dwt_writetxfctrl call could be outside the main while(1) loop.
         */
        dwt_writetxfctrl(FRAME_LENGTH, 0, 0); /* Zero offset in TX buffer, no ranging. */

        /* Start transmission. */
        dwt_starttx(DWT_START_TX_IMMEDIATE);
        /* Poll DW IC until TX frame sent event set. See NOTE 4 below.
         * STATUS register is 4 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API

         * function to access it.*/
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
        { };

        /* Clear TX frame sent event. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

        test_run_info((unsigned char *)"TX Frame Sent\n");
        state = gpio_pin_state_checking();
        //printf("\n %d",state);
        if(state == 1)
        {
           uint8_t distance[] = "32m";
           memcpy(m_tx_buf,distance,sizeof(distance));
           spi_arduino(); // transmit spi to arduino
        }

        /* Execute a delay between transmissions. */
        Sleep(TX_DELAY_MS);

        /* Increment the blink frame sequence number (modulo 256). */
        tx_msg[BLINK_FRAME_SN_IDX]++;
    }
}

#endif
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The device ID is a hard coded constant in the blink to keep the example simple but for a real product every device should have a unique ID.
 *    For development purposes it is possible to generate a DW IC unique ID by combining the Lot ID & Part Number values programmed into the
 *    DW IC during its manufacture. However there is no guarantee this will not conflict with someone else�s implementation. We recommended that
 *    customers buy a block of addresses from the IEEE Registration Authority for their production items. See "EUI" in the DW IC User Manual.
 * 2. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 3. dwt_writetxdata() takes the full size of tx_msg as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW IC. This means that our tx_msg could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 4. We use polled mode of operation here to keep the example as simple as possible, but the TXFRS status event can be used to generate an interrupt.
 *    Please refer to DW IC User Manual for more details on "interrupts".
 * 5. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *    configuration.
 ****************************************************************************************************************************************************/