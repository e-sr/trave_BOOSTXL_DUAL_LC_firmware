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
#define SETUP_TARE_CAL 3

#define ERR_INVALID_CTRL 1
#define ERR_TRANSITION_NOT_ALLOWED 2
#define ERR_MSG_NOT_HANDLED 3

#define LC1_OFFSET 26726
#define LC1_CAL_AT_1Kg (4.98446534e-05)


#define LC0_OFFSET 43629 //
#define LC0_CAL_AT_1Kg (5.01873656e-05)

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

#define TSM_CTRL_ADDR 0
#define TSM_MAVG_SETTINGS_ADDR 1
#define TSM_START_STOP_SETTINGS_ADDR 2
#define TSM_TARE_CAL_SETTINGS_ADDR 3
#define TSM_CALIBRATION_0_ADDR 4
#define TSM_CALIBRATION_1_ADDR 5

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

#define MSG_NOTIFY_ID   1
#define MSG_NOTIFY_MBOX 1
#define MSG_NOTIFY_LEN  8

#define MSG_SR400_ID    3
#define MSG_SR400_MBOX  2
#define MSG_SR400_LEN   6

#define MSG_FIR_ID    4
#define MSG_FIR_MBOX  3
#define MSG_FIR_LEN   6


#define MSG_MAVG_ID    7
#define MSG_MAVG_MBOX  6
#define MSG_MAVG_LEN   6

#define MSG_MSD_ID    8
#define MSG_MSD_MBOX  7
#define MSG_MSD_LEN   6

#define ECAN_MBOX_DIRECTION (1<<MSG_CTRL_MBOX) // 1:TX , 0:RX
#define ECAN_MBOX_ENABLE ((1<<MSG_CTRL_MBOX) |(1<<MSG_NOTIFY_MBOX)|(1<<MSG_SR400_MBOX)|(1<<MSG_FIR_MBOX)|(1<<MSG_MAVG_MBOX)|(1<<MSG_MSD_MBOX));


#define FLOAT2INT_FACTOR 100 //i dati spediti via can bus sono formattati come integer uin16. 1 corrisponde a 10g
//&&
#define MBOX_P(mbox_n) ((&ECanaMboxes.MBOX0) + mbox_n)

//_--------------------------
//Initial Values


#define SETTINGS_MAVG_MASK  0x000000ff

struct MAVG_SETTINGS_BITS
{   //first byte
    uint32_t m_avg_input :2; //0-sr400,...,3-sr300
    uint32_t m_avg_len :8; //m_avg_len = 1<<(m_avg_len+1)
    uint32_t m_avg_decimation_output_rate:8;
    //
    uint32_t m_avg_tot_decimation :8;
    uint32_t fir_decimation :6;
};

typedef union _MAVG_SETTINGS_REG
{
    uint32_t all;
    struct MAVG_SETTINGS_BITS bit;
} tsm_mavg_settings_t;

struct TARE_CAL_SETTINGS_BITS
{   //first byte
    uint32_t lc_calibration:3;
    uint32_t lc_offset_calc:3;
    uint32_t tare_cal_wait_seconds :2;
    uint32_t calibration_value_kg:8;
    uint32_t tare_cal_sd_stability_threshold:16;
};

typedef union TARE_CAL_SETTINGS_REG
{
    uint32_t all;
    struct TARE_CAL_SETTINGS_BITS bit;
} tsm_tare_cal_settings_t;


struct START_STOP_SETTINGS_BITS
{   //first byte
    uint32_t start_stop_mode:2;
    uint32_t start_input:3;
    uint32_t stop_input:3;
    //
    uint32_t start_value:8;
    uint32_t stop_value:8;
    //
    uint32_t wait_sr400_cycles :8;
};

typedef union START_STOP_SETTINGS_REG
{
    uint32_t all;
    struct START_STOP_SETTINGS_BITS bit;
}tsm_mavg_start_stop_settings_t;;

struct CTRL_BITS
{
    uint16_t T_IDLE :1;
    uint16_t T_TARE :1;
    uint16_t T_CAL :1;
    uint16_t T_MES_WAIT :1;
    uint16_t T_MES_ACQ :1;
    uint16_t T_MES_STOP :1;
    //uint16_t T_SETTINGS :1;
    //uint16_t T_SCALE :1;
    uint16_t rsvd : 2;
    uint16_t SETUP_flag :8;
};

typedef union CTRL_REG
{
    uint16_t all;
    struct CTRL_BITS bit;
}tsm_ctrl_t;;

/*
//tere are internal and external events
//events live until cleared some events
//actions can set and clear events
//some events are cleared after a cycle with a mask
*/
typedef struct _F_t
{
    float sr400;
    float fir_out;
    float movingAvg;
    float movingSd;
} F_t;

typedef F_t *F_t_handle;



typedef volatile struct _TraveSM_
{
    //Register
    ///////////////////////////////////////////////////////
    uint16_t state;                                     //0
    tsm_ctrl_t CTRL;                                //1
    tsm_mavg_settings_t mavg_settings;              //2
    tsm_mavg_start_stop_settings_t start_stop_settings;  //3
    tsm_tare_cal_settings_t tare_cal_settings;      //4
    // ADC offset and calibration
    float lc_calibration[2];                            //6,7
    /////////////////////////////////////////////////////////
    // internal
    uint32_t acquire_counter;
    // start stop threshold
    volatile float *start_value_p;
    volatile float *stop_value_p;
    volatile float start_value;
    volatile float stop_value;
    // moving avg parameters
    volatile int32_t *m_avg_input_p[2];
    uint32_t m_avg_decimation_input_rate;
    uint32_t m_avg_decimation_output_rate;
    //
    volatile float tare_cal_sd_stability_threshold;
    //force
    F_t force;

} TraveSM_t;


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



//void run_IDLE_menu(volatile TraveSM_t *tSM,volatile keys_t *key, uint16_t reset);
//uint16_t run_SETTINGS_menu(volatile TraveSM_t *tSM, volatile keys_t *key,msg_notify_t* msg_notify,
//                       uint16_t reset);

void TSM_setup_start_stop_settings(volatile TraveSM_t *tSM);

void TSM_setup_mavg_settings(volatile TraveSM_t *tSM);
void TSM_setup_tare_cal_settings(volatile TraveSM_t *tSM);

void TSM_write_and_send_stream_mbox(uint16_t mbox_n, uint16_t header, float32 v1,
                                float32 v2);
void TSM_handle_msg_in_IDLE(volatile TraveSM_t* tSM, msg_ctrl_t* msg_ctrl,
                       msg_notify_t* msg_notify);


//
void ECan_init(void);
void ECan_SetupMbox(void);
void ECan_setMBoxData(uint16_t mbox_n, uint32_t tx_data[]);
uint16_t ECan_getCTRLMBoxData(msg_ctrl_t* msg_p);
void ECan_sendNotifyMBox(msg_notify_t* msg_p);
void ECan_sendMBox(uint16_t mbox_n);

#endif /* LOADCELL_H_ */
