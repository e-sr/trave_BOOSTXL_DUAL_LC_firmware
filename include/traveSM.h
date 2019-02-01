/*
 * LoadCell.h
 *
 *  Created on: May 30, 2018
 *      Author: esr
 */
#include <stdint.h>


#ifndef LOADCELL_H_
#define LOADCELL_H_
//%%
//states
#define N_STATES 8
#define IDLE 0
#define TARE 1
#define CAL   2
#define MES_WAIT 3
#define MES_ACQ 4
#define MES_STOP 5
#define SETTINGS 6
#define SCALE 7

//--------------------------------
#define CTRL_MASK_IDLE 0XFF
#define CTRL_MASK_CAL 0XFF
#define CTRL_MASK_TARE 0XFF
#define CTRL_MASK_ 0XFF
#define CTRL_MASK_MES_WAIT 0XFF
#define CTRL_MASK_MES_ACQ 0XFF
#define CTRL_MASK_MES_STOP 0XFF
#define CTRL_MASK_TRASITIONS 0x3F

#define START_STOP_MODE_MANUAL 0
#define START_STOP_MODE_AUTO 1

#define SETUP_NONE 0
#define SETUP_MAVG 1
#define SETUP_START_STOP 2
#define SETUP_SETTINGS 3
#define SETUP_REC_START_VALUE 4
#define SETUP_REC_STOP_VALUE 5

#define ERR_INVALID_CTRL 1
#define ERR_TRANSITION_NOT_ALLOWED 2

typedef enum _setup_ {
    none=0,
    mavg,
    start_stop,
    settings,
    rec_start_value,
    rec_stop_value,
}setup_e;
//&&

//----------------------------
//CAN MSG
//-----------------------------
// 4 tipi di messaggi, controllo , notifica, 2xdati
//
// riceve solo control msg

// in caso di messaggio controllo si ottiene il raggiungimento del control tramite il me
// ssaggio di notifica, in caso di controllo non possibile si notifica un errore
// autonotifiche per : trasnsizioni setup e settings
// in caso di setup ad adempimento setup con messaggio stato forzato
// in caso di request register send tramite la spedizione di registro con msg data

//%%
//DATA MDG
//id
#define DATA_COUNTER_RESET_SECONDS 8 //128

//CTRL MESSAGE groups
#define TRANSITION_CTRL 0
#define SETUP_CTRL 10
#define SET_REG_CTRL 20
#define GET_REG_CTRL 30

//NOTIFY MESSAGE GROUPS
#define NONE_NOTIFICATION 0
#define TRANSITION_NOTIFICATION 1
#define SET_REG_NOTIFICATION 2
#define GET_REG_NOTIFICATION 3
#define SETUP_NOTIFICATION 10
#define ERROR_NOTIFICATION 20

#define MSG_CTRL_ID     2
#define MSG_CTRL_MBOX   0
#define MSG_CTRL_LEN    6

#define MSG_STATUS_ID   1
#define MSG_STATUS_MBOX 1
#define MSG_STATUS_LEN  8

#define MSG_SR400_ID    3
#define MSG_SR400_MBOX  2
#define MSG_SR400_LEN   6

#define MSG_SR200_ID    4
#define MSG_SR200_MBOX  3
#define MSG_SR200_LEN   6

#define MSG_SR100_ID    5
#define MSG_SR100_MBOX  4
#define MSG_SR100_LEN   6

#define MSG_SR50_ID    6
#define MSG_SR50_MBOX  5
#define MSG_SR50_LEN   6

#define MSG_MAVG_ID    7
#define MSG_MAVG_MBOX  6
#define MSG_MAVG_LEN   6

#define MSG_MSD_ID    8
#define MSG_MSD_MBOX  7
#define MSG_MSD_LEN   6

#define FLOAT2INT_FACTOR 100 //i dati spediti via can bus sono formattati come integer uin16. 1 corrisponde a 10g
//&&
#define MBOX_P(mbox_n) ((&ECanaMboxes.MBOX0) + mbox_n)

//_--------------------------
//Initial Values


#define SETTINGS_MAVG_MASK  0x000000ff

struct SETTINGS_BITS
{   //first byte
    uint32_t m_avg_len :3; //m_avg_len = 1<<(m_avg_len+1)
    uint32_t m_avg_input :2; //0-sr400,...,3-sr300
    uint32_t m_avg_decimation_output_rate:3;
    //second byte
    uint32_t start_stop_mode:2;
    uint32_t start_input:3;
    uint32_t stop_input:3;
    //
    uint32_t rsvd :16;
};

union SETTINGS_REG
{
    uint32_t all;
    struct SETTINGS_BITS bit;
};

//-------------------------------
//LCD

#define LCD_EMPTY_ROW "                    "

#define LCD_ROW_STATUS 1
#define LCD_COLUMN_STATE  12
#define LCD_MENU_ROW1 3

#define LCD_STATE_TEXT {"IDLE    ","TARE    ","CAL     ",\
    "MES WAIT","ACQUIRE ","MES STOP"}

#define LCD_MES_TEXT_START 20
#define LCD_MES_TEXT "Force(kg):            Time(s): "
#define LCD_FORCE_START (20+12)
#define LCD_TIME_START (40+12)
#define LCD_MES_STOP_TEXT "ForceMax(kg):       Time(s): "
#define LCD_TARE_CAL_START 20
#define LCD_TARE_CAL_TEXT "Variance: "
#define LCD_SD_START (20+12)
//------------------------------------------------------------


struct CTRL_BITS
{
    uint16_t T_IDLE :1;
    uint16_t T_TARE :1;
    uint16_t T_CAL :1;
    uint16_t T_MES_WAIT :1;
    uint16_t T_MES_ACQ :1;
    uint16_t T_MES_STOP :1;
    uint16_t T_SETTINGS :1;
    uint16_t T_SCALE :1;
    uint16_t SETUP :8;
};

union CTRL_REG
{
    uint16_t all;
    struct CTRL_BITS bit;
};

/*
//tere are internal and external events
//events live until cleared some events
//actions can set and clear events
//some events are cleared after a cycle with a mask
*/
typedef struct _F_t
{
    float sr400;
    float sr100;
    float sr200;
    float sr50;
    float movingAvg;
    float movingSd;
} F_t;

typedef F_t *F_t_handle;


#define REG_SETTINGS 0
typedef volatile struct _TraveSM_
{
    //Control
    union CTRL_REG CTRL;
    union SETTINGS_REG SETTING;
    // ADC offset and calibration
    uint16_t state;
    float calibration_constant; //10Kg
    float sd_stability_threshold;
    uint32_t tare_cal_wait_mAvg_cycles;
    //
    uint32_t offset[2];
    float calibration[2];
    // start stop threshold
    volatile float *start_value_p;
    volatile float *stop_value_p;
    uint16_t stop_wait_sr400_cycles;
    float start_force_threshold;
    float stop_force_threshold;
    //
    uint32_t acquire_counter;
    // moving avg parameters
    uint32_t m_avg_len;
    volatile int32_t *m_avg_input_p[2];
    uint32_t m_avg_decimation_input_rate;
    uint32_t m_avg_decimation_output_rate;
    //force
    F_t force;

} TraveSM_t;

struct KEY_BIT{
  uint16_t enter:1;
  uint16_t esc:1;
  uint16_t rsvd:14;
};
typedef struct KEYS
{
    struct KEY_BIT pressed;
    uint32_t _s;
    uint32_t _state;
    uint32_t _mask;
    uint32_t _ct0;
    uint32_t _ct1;
    uint32_t _lastPOSCNT;
    volatile uint32_t *_gpio_data_regs;
} keys_t;

struct MSG_DATA_HEADER_BITS
{
    Uint16 isr_state:4;
    Uint16 counter:12;
};


typedef union _msg_data_header_
{
    uint16_t all;
    struct MSG_DATA_HEADER_BITS bit;
}msg_data_header_t;


struct CTRL_HEADER_BITS
{
    uint16_t ctrl :8;
    uint16_t id :8;
};

union CTRL_HEADER
{
    uint16_t all;
    struct CTRL_HEADER_BITS bit;
};

 typedef struct _msg_ctrl_
{
    union CTRL_HEADER ctrl;
    uint32_t data;
}msg_ctrl_t ;




struct NOTIFY_BITS
{
    uint32_t state :8;
    uint32_t reply_id :8;
    uint32_t notification:8;
    uint32_t set_get:8;
};

union NOTIFY_HEADER
{
    uint32_t all;
    struct NOTIFY_BITS bit;
};

typedef struct _msg_notify_
{
   union NOTIFY_HEADER header;
   uint32_t data;
}msg_notify_t ;


void key_press_init(volatile keys_t *keys,  volatile uint32_t *gpio_data_regs,uint32_t mask);
void key_press_debounce(volatile keys_t *keys);
int16_t key_get_rotations_clicks(volatile keys_t *keys);
void run_IDLE_menu(volatile TraveSM_t *tSM,volatile keys_t *key, uint16_t reset);
uint16_t run_SETTINGS_menu(volatile TraveSM_t *tSM, volatile keys_t *key,msg_notify_t* msg_notify,
                       uint16_t reset);
uint16_t add_value_clicks(uint16_t value,uint16_t clicks, uint16_t min, uint16_t max);


#endif /* LOADCELL_H_ */
