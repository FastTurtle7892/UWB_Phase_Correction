#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
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

#define MAX_WINDOW_SIZE 15
static double data_buffer_ch9_p[MAX_WINDOW_SIZE];
static int window_size_ch9_p = 0;
static int buffer_index_ch9_p = 0;
static double sum_ch9_p = 0.0;
static int data_count_ch9_p = 0;

static double data_buffer_ch9_N[MAX_WINDOW_SIZE];
static int window_size_ch9_N = 0;
static int buffer_index_ch9_N = 0;
static double sum_ch9_N = 0.0;
static int data_count_ch9_N = 0;



#define PI 3.14159265358979323846

typedef struct {
    double x;  // Estimated state
    double p;  // Estimation error covariance
    double k;  // Kalman gain
    double q;  // Process noise covariance
    double r;  // Measurement noise covariance
} KalmanFilter;


typedef struct {

  uint32_t res_rx_final_tx;
  uint32_t poll_tx_res_rx;
  uint32_t resp_timeout;

} Init_DelayTimes;


typedef struct {

  uint32_t poll_rx_res_tx;
  uint32_t res_tx_final_rx;
  uint32_t final_timeout;

} Resp_DelayTimes;

int getPreambleLength(int txPreambLength);

int getPacLength(int rxPAC);

void getDelayTime_initiator(dwt_config_t *config, uint32_t* res_rx_final_tx, uint32_t* poll_tx_res_rx, uint32_t* resp_timeout);
void getDelayTime_responder(dwt_config_t *config, uint32_t* poll_rx_res_tx, uint32_t* res_tx_final_Rx, uint32_t* final_timeout);
void saveDistancesToTxt(const double* distances, const char* filename, int size);
Init_DelayTimes ds_twr_getDelayTime_initiator(dwt_config_t *config);
Resp_DelayTimes ds_twr_getDelayTime_responder(dwt_config_t *config);


void timer_init_uss(void);
void timer4_irqhandler(void);
float TIMER4_OFTime(void);
uint32_t TIMER4_Check(uint32_t starttime, float clk_time);
uint64_t TIMER4_Check2(uint32_t starttime, float clk_time);

uint64_t get_10_pkt_time(uint8_t preamble_len);

void collect_CIR(FILE *out, uint8_t accum_data[], uint8_t ACCUM_DATA_LEN);
double calculate_CIR(uint8_t accum_data[], uint8_t ACCUM_DATA_LEN);
void calculate_CIRs(uint8_t accum_data[], uint8_t ACCUM_DATA_LEN, float *phase_angles);
void save_CIR(uint8_t accum_data[], uint8_t ACCUM_DATA_LEN, int32_t cir_data[], int size);

static void show_error(void);
static void uart_error_handle(app_uart_evt_t * p_event);
static void uart0_strsend(uint8_t *str);
double calculate_new_distance_CH9(double distance, double poll_phase, double response_phase, double final_phase, double post_final_phase);

void kalman_init(KalmanFilter *kf, double q, double r, double initial_value);

// Update the Kalman filter (using N as the measurement)
double kalman_update(KalmanFilter *kf, double N);
double normalize_phase(double phase);

void init_moving_average_ch9_p(int size);
double apply_moving_average_ch9_p(double new_data);

void init_moving_average_ch9_N(int size);
int apply_moving_average_ch9_N(int new_data);





typedef enum {
    DB_N30 = (0x00<<2)+3,
    DB_N29 = (0x01<<2)+1,
    DB_N28 = (0x02<<2)+0,
    DB_N27 = (0x01<<2)+3,
    DB_N26 = (0x02<<2)+1,
    DB_N25 = (0x03<<2)+0,
    DB_N24 = (0x02<<2)+3,
    DB_N23 = (0x03<<2)+1,
    DB_N22 = (0x05<<2)+0,
    DB_N21 = (0x03<<2)+3,
    DB_N20 = (0x07<<2)+0,
    DB_N19 = (0x04<<2)+3,
    DB_N18 = (0x06<<2)+1,
    DB_N17 = (0x05<<2)+3,
    DB_N16 = (0x06<<2)+3,
    DB_N15 = (0x07<<2)+3,
    DB_N14 = (0x08<<2)+3,
    DB_N13 = (0x09<<2)+3,
    DB_N12 = (0x0A<<2)+3,
    DB_N11 = (0x0F<<2)+1,
    DB_N10 = (0x0D<<2)+3,
    DB_N9 = (0x0F<<2)+3,
    DB_N8 = (0x11<<2)+3,
    DB_N7 = (0x14<<2)+3,
    DB_N6 = (0x17<<2)+3,
    DB_N5 = (0x1A<<2)+3,
    DB_N4 = (0x1F<<2)+3,
    DB_N3 = (0x24<<2)+3,
    DB_N2 = (0x2A<<2)+3,
    DB_N1 = (0x34<<2)+3,
    Db_N0 = (0x3F<<2)+3
} db_gain_e;


#define WINDOW_SIZE 20

typedef struct {
    int window[WINDOW_SIZE]; 
    int sorted_window[WINDOW_SIZE];
    int index;
} MedianFilter_N;

typedef struct {
    double window[WINDOW_SIZE]; 
    double sorted_window[WINDOW_SIZE];
    int index;
} MedianFilter_PHASE;

void insert_and_sort_N(MedianFilter_N *filter, int new_value);
int get_median_N(MedianFilter_N *filter);
void init_filter_N(MedianFilter_N *filter);
int process_value_N(MedianFilter_N *filter, int new_value);

void insert_and_sort_PHASE(MedianFilter_PHASE *filter, double new_value);
double get_median_PHASE(MedianFilter_PHASE *filter);
void init_filter_PHASE(MedianFilter_PHASE *filter);
double process_value_PHASE(MedianFilter_PHASE *filter, double new_value);

#ifdef __cplusplus
}
#endif