/*! ----------------------------------------------------------------------------
 * @file    example_info.h
 * @brief
 *
 * @attention
 *
 * Copyright 2013-2018(c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#include <assert.h>
#include <example_selection.h>
#include "examples_defines.h"

example_ptr example_pointer;

void build_examples(void)
{
    unsigned char test_cnt=0;

#ifdef TEST_READING_DEV_ID
    extern int read_dev_id(void);

    example_pointer=read_dev_id;
    test_cnt++;
#endif

#ifdef TEST_SIMPLE_TX
    extern int simple_tx(void);

    example_pointer=simple_tx;
    test_cnt++;
#endif

#ifdef TEST_SIMPLE_TX_PDOA
    extern int simple_tx_pdoa(void);

    example_pointer=simple_tx_pdoa;
    test_cnt++;
#endif

#ifdef TEST_SIMPLE_RX
    extern int simple_rx(void);

    example_pointer=simple_rx;
    test_cnt++;
#endif

#ifdef TEST_RX_SNIFF
    extern int rx_sniff(void);

    example_pointer=rx_sniff;
    test_cnt++;
#endif

#ifdef TEST_RX_TRIM
    extern int rx_with_xtal_trim(void);

    example_pointer=rx_with_xtal_trim;
    test_cnt++;
#endif

#ifdef TEST_RX_DIAG
    extern int rx_diagnostics(void);

    example_pointer=rx_diagnostics;
    test_cnt++;
#endif

#ifdef TEST_TX_SLEEP
    extern int tx_sleep(void);

    example_pointer=tx_sleep;
    test_cnt++;
#endif

#ifdef TEST_TX_SLEEP_IDLE_RC
    extern int tx_sleep_idleRC(void);

    example_pointer=tx_sleep_idleRC;
    test_cnt++;
#endif

#ifdef TEST_TX_SLEEP_TIMED
    extern int tx_timed_sleep(void);

    example_pointer=tx_timed_sleep;
    test_cnt++;
#endif

#ifdef TEST_TX_SLEEP_AUTO
    extern int tx_sleep_auto(void);

    example_pointer=tx_sleep_auto;
    test_cnt++;
#endif

#ifdef TEST_TX_WITH_CCA
    extern int tx_with_cca(void);

    example_pointer=tx_with_cca;
    test_cnt++;
#endif

#ifdef TEST_SIMPLE_TX_AES
    extern int simple_tx_aes(void);

    example_pointer=simple_tx_aes;
    test_cnt++;
#endif

#ifdef TEST_SIMPLE_RX_AES
    extern int simple_rx_aes(void);

    example_pointer=simple_rx_aes;
    test_cnt++;
#endif

#ifdef TEST_TX_WAIT_RESP
    extern int tx_wait_resp(void);

    example_pointer=tx_wait_resp;
    test_cnt++;
#endif

#ifdef TEST_TX_WAIT_RESP_INT
    extern int tx_wait_resp_int(void);

    example_pointer=tx_wait_resp_int;
    test_cnt++;
#endif

#ifdef TEST_RX_SEND_RESP
    extern int rx_send_resp(void);

    example_pointer=rx_send_resp;
    test_cnt++;
#endif

#ifdef TEST_SS_TWR_RESPONDER
    extern int ss_twr_responder(void);

    example_pointer=ss_twr_responder;
    test_cnt++;
#endif

#ifdef TEST_SS_TWR_INITIATOR
    extern int ss_twr_initiator(void);

    example_pointer=ss_twr_initiator;
    test_cnt++;
#endif

#ifdef TEST_SS_TWR_INITIATOR_STS
    extern int ss_twr_initiator_sts(void);

    example_pointer=ss_twr_initiator_sts;
    test_cnt++;
#endif

#ifdef TEST_SS_TWR_RESPONDER_STS
    extern int ss_twr_responder_sts(void);

    example_pointer=ss_twr_responder_sts;
    test_cnt++;
#endif

#ifdef TEST_SS_TWR_INITIATOR_STS_NO_DATA
    extern int ss_twr_initiator_sts_no_data(void);

    example_pointer=ss_twr_initiator_sts_no_data;
    test_cnt++;
#endif

#ifdef TEST_SS_TWR_RESPONDER_STS_NO_DATA
    extern int ss_twr_responder_sts_no_data(void);

    example_pointer=ss_twr_responder_sts_no_data;
    test_cnt++;
#endif

#ifdef TX_RX_AES_VERIFICATION
    extern int tx_rx_aes_verification(void);

    example_pointer=tx_rx_aes_verification;
    test_cnt++;
#endif

#ifdef TEST_AES_SS_TWR_INITIATOR
    extern int ss_aes_twr_initiator(void);

    example_pointer=ss_aes_twr_initiator;
    test_cnt++;
#endif

#ifdef TEST_AES_SS_TWR_RESPONDER
    extern int ss_aes_twr_responder(void);

    example_pointer=ss_aes_twr_responder;
    test_cnt++;
#endif

#ifdef TEST_DS_TWR_INITIATOR
    extern int ds_twr_initiator(void);

    example_pointer=ds_twr_initiator;
    test_cnt++;
#endif

#ifdef TEST_DS_TWR_RESPONDER
    extern int ds_twr_responder(void);

    example_pointer=ds_twr_responder;
    test_cnt++;
#endif

#ifdef TEST_DS_TWR_RESPONDER_CP
    extern int ds_twr_responder_cp(void);

    example_pointer=ds_twr_responder_cp;
    test_cnt++;
#endif

#ifdef TEST_DS_TWR_RESPONDER_STS
    extern int ds_twr_responder_sts(void);

    example_pointer=ds_twr_responder_sts;
    test_cnt++;
#endif

#ifdef TEST_DS_TWR_INITIATOR_STS
    extern int ds_twr_initiator_sts(void);

    example_pointer=ds_twr_initiator_sts;
    test_cnt++;
#endif

#ifdef TEST_DS_TWR_STS_SDC_INITIATOR_AOA
    extern int ds_twr_sts_sdc_initiator(void);

    example_pointer=ds_twr_sts_sdc_initiator;
    test_cnt++;
#endif

#ifdef TEST_DS_TWR_STS_SDC_RESPONDER_AOA
    extern int ds_twr_sts_sdc_responder(void);

    example_pointer=ds_twr_sts_sdc_responder;
    test_cnt++;
#endif

#ifdef TEST_CONTINUOUS_WAVE
    extern int continuous_wave_example(void);

    example_pointer=continuous_wave_example;
    test_cnt++;
#endif

#ifdef TEST_CONTINUOUS_FRAME
    extern int continuous_frame_example(void);

    example_pointer=continuous_frame_example;
    test_cnt++;

#endif

#ifdef TEST_ACK_DATA_RX
    extern int ack_data_rx(void);

    example_pointer=ack_data_rx;
    test_cnt++;

#endif

#ifdef TEST_ACK_DATA_TX
    extern int ack_data_tx(void);

    example_pointer=ack_data_tx;
    test_cnt++;

#endif

#ifdef TEST_GPIO
    extern int gpio_example(void);

    example_pointer=gpio_example;
    test_cnt++;
#endif

#ifdef TEST_SIMPLE_TX_STS_SDC
    extern int simple_tx_sts_sdc(void);

    example_pointer=simple_tx_sts_sdc;
    test_cnt++;
#endif

#ifdef TEST_SIMPLE_RX_STS_SDC
    extern int simple_rx_sts_sdc(void);

    example_pointer=simple_rx_sts_sdc;
    test_cnt++;
#endif

#ifdef TEST_FRAME_FILTERING_TX
    extern int frame_filtering_tx(void);

    example_pointer=frame_filtering_tx;
    test_cnt++;
#endif

#ifdef TEST_FRAME_FILTERING_RX
    extern int frame_filtering_rx(void);

    example_pointer=frame_filtering_rx;
    test_cnt++;
#endif

#ifdef TEST_ACK_DATA_RX_DBL_BUFF
    extern int ack_data_rx_dbl_buff(void);

    example_pointer=ack_data_rx_dbl_buff;
    test_cnt++;
#endif

#ifdef TEST_SPI_CRC
    extern int spi_crc(void);

    example_pointer=spi_crc;
    test_cnt++;
#endif

#ifdef TEST_SIMPLE_RX_PDOA
    extern int simple_rx_pdoa(void);

    example_pointer=simple_rx_pdoa;
    test_cnt++;
#endif

#ifdef TEST_OTP_WRITE
    extern int otp_write(void);

    example_pointer=otp_write;
    test_cnt++;
#endif

#ifdef TEST_LE_PEND_TX
    extern int le_pend_tx(void);

    example_pointer=le_pend_tx;
    test_cnt++;
#endif

#ifdef TEST_LE_PEND_RX
    extern int le_pend_rx(void);

    example_pointer=le_pend_rx;
    test_cnt++;
#endif

#ifdef TEST_DS_TWR_INITIATOR_AOA
    extern int ds_twr_initiator_aoa(void);

    example_pointer=ds_twr_initiator_aoa;
    test_cnt++;
#endif

#ifdef TEST_DS_TWR_RESPONDER_AOA
    extern int ds_twr_responder_aoa(void);

    example_pointer=ds_twr_responder_aoa;
    test_cnt++;
#endif

#ifdef LOCALIZATION_DS_TWR_INIT
    extern int localization_ds_twr_initiator(void);

    example_pointer=localization_ds_twr_initiator;
    test_cnt++;
#endif

#ifdef LOCALIZATION_DS_TWR_RESP
    extern int localization_ds_twr_responder(void);

    example_pointer=localization_ds_twr_responder;
    test_cnt++;
#endif

#ifdef TEST_SIMPLE_TX_SPI
    extern int simple_tx(void);

    example_pointer=simple_tx;
    test_cnt++;
#endif

#ifdef AoA_rtls_tx
    extern int aoa_rtls_initiator(void);

    example_pointer=aoa_rtls_initiator;
    test_cnt++;
#endif

#ifdef AoA_rtls_rx
    extern int aoa_rtls_responder(void);

    example_pointer=aoa_rtls_responder;
    test_cnt++;
#endif

#ifdef AoA_RTLS_TX_AI
    extern int aoa_rtls_initiator_ai(void);

    example_pointer=aoa_rtls_initiator_ai;
    test_cnt++;
#endif

#ifdef AoA_RTLS_RX_AI
    extern int aoa_rtls_responder_ai(void);

    example_pointer=aoa_rtls_responder_ai;
    test_cnt++;
#endif


#ifdef AoA_rtls_rx_spi
    extern int aoa_rtls_responder(void);

    example_pointer=aoa_rtls_responder;
    test_cnt++;
#endif

#ifdef AoD
    extern int aod(void);

    example_pointer=aod;
    test_cnt++;
#endif


#ifdef SIMPLE_TX_CP
    extern int simple_tx_cp(void);

    example_pointer=simple_tx_cp;
    test_cnt++;
#endif

#ifdef SIMPLE_RX_CP
    extern int simple_rx_cp(void);

    example_pointer=simple_rx_cp;
    test_cnt++;
#endif



#ifdef UART_TEST
    extern int uart_TEST(void);

    example_pointer=uart_TEST;
    test_cnt++;
#endif

#ifdef TEST_DS_TWR_INITIATOR_CP
    extern int ds_twr_initiator_cp(void);

    example_pointer=ds_twr_initiator_cp;
    test_cnt++;
#endif

#ifdef TEST_RX_DIAG_CP
    extern int rx_diagnostics_cp(void);

    example_pointer=rx_diagnostics_cp;
    test_cnt++;
#endif

#ifdef AoA_rtls_tx_txt
    extern int aoa_rtls_initiator_txt(void);

    example_pointer=aoa_rtls_initiator_txt;
    test_cnt++;
#endif

#ifdef AoA_rtls_rx_txt
    extern int aoa_rtls_responder_txt(void);

    example_pointer=aoa_rtls_responder_txt;
    test_cnt++;
#endif

#ifdef AoA_rtls_tx_two
    extern int aoa_rtls_initiator_two(void);

    example_pointer=aoa_rtls_initiator_two;
    test_cnt++;
#endif

#ifdef AoA_rtls_rx_two
    extern int aoa_rtls_responder_two(void);

    example_pointer=aoa_rtls_responder_two;
    test_cnt++;
#endif

#ifdef DS_TWR_INITIATOR_FINAL
    extern int ds_twr_initiator_final(void);

    example_pointer=ds_twr_initiator_final;
    test_cnt++;
#endif

#ifdef DS_TWR_RESPONDER_FINAL
    extern int ds_twr_responder_final(void);

    example_pointer=ds_twr_responder_final;
    test_cnt++;
#endif

#ifdef DS_TWR_INITIATOR_PACKET_SIMULATION
    extern int ds_twr_initiator_packet_simulation(void);

    example_pointer=ds_twr_initiator_packet_simulation;
    test_cnt++;
#endif

#ifdef DS_TWR_RESPONDER_PACKET_SIMULATION
    extern int ds_twr_responder_packet_simulation(void);

    example_pointer=ds_twr_responder_packet_simulation;
    test_cnt++;
#endif

#ifdef DS_TWR_INITIATOR_ORIGIN
    extern int ds_twr_initiator_origin(void);

    example_pointer=ds_twr_initiator_origin;
    test_cnt++;
#endif

#ifdef DS_TWR_RESPONDER_ORIGIN
    extern int ds_twr_responder_origin(void);

    example_pointer=ds_twr_responder_origin;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_INITIATOR_WITH_NO_STS
    extern int custom_ds_twr_initiator_with_no_sts(void);

    example_pointer=custom_ds_twr_initiator_with_no_sts;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_WITH_NO_STS
    extern int custom_ds_twr_responder_with_no_sts(void);

    example_pointer=custom_ds_twr_responder_with_no_sts;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_INITIATOR_WITH_NO_STS_2
    extern int custom_ds_twr_initiator_with_no_sts_2(void);

    example_pointer=custom_ds_twr_initiator_with_no_sts_2;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_WITH_NO_STS_2
    extern int custom_ds_twr_responder_with_no_sts_2(void);

    example_pointer=custom_ds_twr_responder_with_no_sts_2;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_INITIATOR_WITH_NO_STS_3
    extern int custom_ds_twr_initiator_with_no_sts_3(void);

    example_pointer=custom_ds_twr_initiator_with_no_sts_3;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_WITH_NO_STS_3
    extern int custom_ds_twr_responder_with_no_sts_3(void);

    example_pointer=custom_ds_twr_responder_with_no_sts_3;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_INITIATOR_WITH_NO_STS_4
    extern int custom_ds_twr_initiator_with_no_sts_4(void);

    example_pointer=custom_ds_twr_initiator_with_no_sts_4;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_WITH_NO_STS_4
    extern int custom_ds_twr_responder_with_no_sts_4(void);

    example_pointer=custom_ds_twr_responder_with_no_sts_4;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_INITIATOR_WITH_NO_STS_5
    extern int custom_ds_twr_initiator_with_no_sts_5(void);

    example_pointer=custom_ds_twr_initiator_with_no_sts_5;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_WITH_NO_STS_5
    extern int custom_ds_twr_responder_with_no_sts_5(void);

    example_pointer=custom_ds_twr_responder_with_no_sts_5;
    test_cnt++;
#endif


#ifdef CUSTOM_DS_TWR_INITIATOR_WITH_NO_STS_ORIGIN
    extern int custom_ds_twr_initiator_with_no_sts_origin(void);

    example_pointer=custom_ds_twr_initiator_with_no_sts_origin;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_WITH_NO_STS_ORIGIN
    extern int custom_ds_twr_responder_with_no_sts_origin(void);

    example_pointer=custom_ds_twr_responder_with_no_sts_origin;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_INITIATOR_TRIM
    extern int custom_ds_twr_initiator_trim(void);

    example_pointer=custom_ds_twr_initiator_trim;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_TRIM
    extern int custom_ds_twr_responder_trim(void);

    example_pointer=custom_ds_twr_responder_trim;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_INITIATOR_CIR
    extern int custom_ds_twr_initiator_cir(void);

    example_pointer=custom_ds_twr_initiator_cir;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_CIR
    extern int custom_ds_twr_responder_cir(void);

    example_pointer=custom_ds_twr_responder_cir;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_INITIATOR_SINGLE_FREQUENCY_NO_DELAY
    extern int custom_ds_twr_initiator_single_frequency_no_delay(void);

    example_pointer=custom_ds_twr_initiator_single_frequency_no_delay;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_SINGLE_FREQUENCY_ANCHOR_1_NO_DELAY
    extern int custom_ds_twr_responder_single_frequency_anchor_1_no_delay(void);

    example_pointer=custom_ds_twr_responder_single_frequency_anchor_1_no_delay;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_SINGLE_FREQUENCY_ANCHOR_2_NO_DELAY
    extern int custom_ds_twr_responder_single_frequency_anchor_2_no_delay(void);

    example_pointer=custom_ds_twr_responder_single_frequency_anchor_2_no_delay;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_SINGLE_FREQUENCY_ANCHOR_3_NO_DELAY
    extern int custom_ds_twr_responder_single_frequency_anchor_3_no_delay(void);

    example_pointer=custom_ds_twr_responder_single_frequency_anchor_3_no_delay;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_SINGLE_FREQUENCY_ANCHOR_4_NO_DELAY
    extern int custom_ds_twr_responder_single_frequency_anchor_4_no_delay(void);

    example_pointer=custom_ds_twr_responder_single_frequency_anchor_4_no_delay;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_INITIATOR_MULTI_FREQUENCY
    extern int custom_ds_twr_initiator_multi_frequency(void);

    example_pointer=custom_ds_twr_initiator_multi_frequency;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_MULTI_FREQUENCY_ANCHOR1
    extern int custom_ds_twr_responder_multi_frequency_anchor1(void);

    example_pointer=custom_ds_twr_responder_multi_frequency_anchor1;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_MULTI_FREQUENCY_ANCHOR2
    extern int custom_ds_twr_responder_multi_frequency_anchor2(void);

    example_pointer=custom_ds_twr_responder_multi_frequency_anchor2;
    test_cnt++;
#endif


#ifdef CUSTOM_DS_TWR_INITIATOR_CHANNEL_SWITCHING
    extern int custom_ds_twr_initiator_channel_switching(void);

    example_pointer=custom_ds_twr_initiator_channel_switching;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_CHANNEL_SWITCHING
    extern int custom_ds_twr_responder_channel_switching(void);

    example_pointer=custom_ds_twr_responder_channel_switching;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_CHANNEL_SWITCHING_V2
    extern int custom_ds_twr_responder_channel_switching_v2(void);

    example_pointer=custom_ds_twr_responder_channel_switching_v2;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_INITIATOR_WITH_STS
    extern int custom_ds_twr_initiator_with_sts(void);

    example_pointer=custom_ds_twr_initiator_with_sts;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_WITH_STS
    extern int custom_ds_twr_responder_with_sts(void);

    example_pointer=custom_ds_twr_responder_with_sts;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_INITIATOR_WITH_STS_2
    extern int custom_ds_twr_initiator_with_sts_2(void);

    example_pointer=custom_ds_twr_initiator_with_sts_2;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_WITH_STS_2
    extern int custom_ds_twr_responder_with_sts_2(void);

    example_pointer=custom_ds_twr_responder_with_sts_2;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_INITIATOR_MULTI_CIR_DATA
    extern int custom_ds_twr_initiator_multi_cir_data(void);

    example_pointer=custom_ds_twr_initiator_multi_cir_data;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_MULTI_CIR_DATA
    extern int custom_ds_twr_responder_multi_cir_data(void);

    example_pointer=custom_ds_twr_responder_multi_cir_data;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_INITIATOR_SINGLE_FREQUENCY_1VS1
    extern int custom_ds_twr_initiator_single_frequency_1vs1(void);

    example_pointer=custom_ds_twr_initiator_single_frequency_1vs1;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_SINGLE_FREQUENCY_ANCHOR_1VS1
    extern int custom_ds_twr_responder_single_frequency_anchor_1vs1(void);

    example_pointer=custom_ds_twr_responder_single_frequency_anchor_1vs1;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_INITIATOR_SINGLE_FREQUENCY_1VS2
    extern int custom_ds_twr_initiator_single_frequency_1vs2(void);

    example_pointer=custom_ds_twr_initiator_single_frequency_1vs2;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_SINGLE_FREQUENCY_ANCHOR_1_1VS2
    extern int custom_ds_twr_responder_single_frequency_anchor_1_1vs2(void);

    example_pointer=custom_ds_twr_responder_single_frequency_anchor_1_1vs2;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_SINGLE_FREQUENCY_ANCHOR_2_1VS2
    extern int custom_ds_twr_responder_single_frequency_anchor_2_1vs2(void);

    example_pointer=custom_ds_twr_responder_single_frequency_anchor_2_1vs2;
    test_cnt++;
#endif


#ifdef CUSTOM_DS_TWR_INITIATOR_SINGLE_FREQUENCY
    extern int custom_ds_twr_initiator_single_frequency(void);

    example_pointer=custom_ds_twr_initiator_single_frequency;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_SINGLE_FREQUENCY_ANCHOR_1
    extern int custom_ds_twr_responder_single_frequency_anchor_1(void);

    example_pointer=custom_ds_twr_responder_single_frequency_anchor_1;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_SINGLE_FREQUENCY_ANCHOR_2
    extern int custom_ds_twr_responder_single_frequency_anchor_2(void);

    example_pointer=custom_ds_twr_responder_single_frequency_anchor_2;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_SINGLE_FREQUENCY_ANCHOR_3
    extern int custom_ds_twr_responder_single_frequency_anchor_3(void);

    example_pointer=custom_ds_twr_responder_single_frequency_anchor_3;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_INITIATOR_MULTI_FREQUENCY_1VS1
    extern int custom_ds_twr_initiator_multi_frequency_1vs1(void);

    example_pointer=custom_ds_twr_initiator_multi_frequency_1vs1;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_MULTI_FREQUENCY_ANCHOR_1VS1
    extern int custom_ds_twr_responder_multi_frequency_anchor_1vs1(void);

    example_pointer=custom_ds_twr_responder_multi_frequency_anchor_1vs1;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_INITIATOR_SINGLE_FREQUENCY_1VS4
    extern int custom_ds_twr_initiator_single_frequency_1vs4(void);

    example_pointer=custom_ds_twr_initiator_single_frequency_1vs4;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_SINGLE_FREQUENCY_ANCHOR_1_1VS4
    extern int custom_ds_twr_responder_single_frequency_anchor_1_1vs4(void);

    example_pointer=custom_ds_twr_responder_single_frequency_anchor_1_1vs4;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_SINGLE_FREQUENCY_ANCHOR_2_1VS4
    extern int custom_ds_twr_responder_single_frequency_anchor_2_1vs4(void);

    example_pointer=custom_ds_twr_responder_single_frequency_anchor_2_1vs4;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_SINGLE_FREQUENCY_ANCHOR_3_1VS4
    extern int custom_ds_twr_responder_single_frequency_anchor_3_1vs4(void);

    example_pointer=custom_ds_twr_responder_single_frequency_anchor_3_1vs4;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_SINGLE_FREQUENCY_ANCHOR_4_1VS4
    extern int custom_ds_twr_responder_single_frequency_anchor_4_1vs4(void);

    example_pointer=custom_ds_twr_responder_single_frequency_anchor_4_1vs4;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_INITIATOR_SINGLE_FREQUENCY_1VS4_COMPARE
    extern int custom_ds_twr_initiator_single_frequency_1vs4_compare(void);

    example_pointer=custom_ds_twr_initiator_single_frequency_1vs4_compare;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_SINGLE_FREQUENCY_ANCHOR_1_1VS4_COMPARE
    extern int custom_ds_twr_responder_single_frequency_anchor_1_1vs4_compare(void);

    example_pointer=custom_ds_twr_responder_single_frequency_anchor_1_1vs4_compare;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_SINGLE_FREQUENCY_ANCHOR_2_1VS4_COMPARE
    extern int custom_ds_twr_responder_single_frequency_anchor_2_1vs4_compare(void);

    example_pointer=custom_ds_twr_responder_single_frequency_anchor_2_1vs4_compare;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_SINGLE_FREQUENCY_ANCHOR_3_1VS4_COMPARE
    extern int custom_ds_twr_responder_single_frequency_anchor_3_1vs4_compare(void);

    example_pointer=custom_ds_twr_responder_single_frequency_anchor_3_1vs4_compare;
    test_cnt++;
#endif

#ifdef CUSTOM_DS_TWR_RESPONDER_SINGLE_FREQUENCY_ANCHOR_4_1VS4_COMPARE
    extern int custom_ds_twr_responder_single_frequency_anchor_4_1vs4_compare(void);

    example_pointer=custom_ds_twr_responder_single_frequency_anchor_4_1vs4_compare;
    test_cnt++;
#endif


    //Check that only 1 test was enabled in test_selection.h file
    assert(test_cnt==1);

}
