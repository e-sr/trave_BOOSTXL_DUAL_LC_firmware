//###########################################################################
//
// FILE:   main.c
//
// TITLE:  Trave application
//
//! Questo programma implementa un sistema di aquisizione dati per un trave. Il sistema acquisisch dati, li visualizza sull'LCD e
//! li mette a disposizione tramite CANBUS o UART.
//!
//! Il sistema é composto dalle seguenti parti:
//!
//! - launchxl f28069
//! - PCB acquisizione contenete due ADC esterni ADS1246 con interfaccia spi e START pin alle quali
//!   possono essere collegate due celle di carico
//! - LCD openlcd seriale  + 2 Bottoni e Encoder
//!
//! descrizione del funzionamento:
//!
//! La SOC dei due ADS é gestita usando lo START pin. Il segnale sul pin é generato dal PWM3B
//! ad una frequenza di PWM_FREQ_Hz (1600Hz). I dati vengono poi letti stramite spib nel pwm3 interrupt
//! generato alla fine del duty cycle. La durata del duty cycle corrisponde al tempo
//! di conversione  dell ADS1246 (impostato a 2kHz).
//!
//! I dati acquisiti sono poi elaborati tramit filtri e vengono visualizzati sull LCD e inviati via CAN BUS
//!
//! l'update del LCD ed altre operazioni ricorrenti (debuncing) vengono gestite dal
//! TIMER1 interrupt a 500Hz
//! -----------------------------------------------------------------------------

#include <stdbool.h>
#include <stdio.h>
#include <math.h>
//
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "C28x_FPU_FastRTS.h"
#include "IQmathLib.h"
#include "filter.h"
#include "include/lpf_filters_coefs.h"
#include "modules/moving_avg.h"
#include "include/traveSM.h"
#include "ADS1220.h"
#include "BOOSTXL_DUAL_LOAD_CELL.h"
//#include "openLCD/openlcd.h"

//SYSTEM definitions
//#define SYS_CLK_FREQ_Hz 80000000 //80MHz guarda Step 1. Initialize System Control
//#define LSPC_CLK_FREQ_Hz (SYS_CLK_FREQ_Hz/4) //20MHz guarda Step 1. Initialize System Control

#define DEBUG_GPIO5 GpioDataRegs.GPADAT.bit.GPIO5
#define LED_D9_RED   GpioDataRegs.GPBDAT.bit.GPIO34
#define LED_D10_BLUE  GpioDataRegs.GPBDAT.bit.GPIO39

//ADS1246 definitions
//#define ADS1246_SETTLING_TIME_us (578+10)//@ 2000Hz guarda datasheet
//#define ADS1246_0_CS GpioDataRegs.GPBDAT.bit.GPIO51
//#define ADS1246_1_CS GpioDataRegs.GPADAT.bit.GPIO3
//#define ADS1246_01_RESET GpioDataRegs.GPADAT.bit.GPIO19
//#define ADS1246_1_START GpioDataRegs.GPADAT.bit.GPIO4
//#define ADS1246_2_START GpioDataRegs.GPADAT.bit.GPIO1
//#define ADS1246_1_DRDY GpioDataRegs.GPBDAT.bit.GPIO55
//#define ADS1246_01_START_HIGH EPwm3Regs.AQCSFRC.bit.CSFA = 2 //PWM Action Qualifier continuus software force HIGH
//#define ADS1246_01_START_LOW EPwm3Regs.AQCSFRC.bit.CSFA = 1 //PWM Action Qualifier continuus software force LOW
//#define ADS1246_01_START_PWM EPwm3Regs.AQCSFRC.bit.CSFA = 0 //PWM Action Qualifier continuus software force disable

#define TIMER1_PRESCALE  0x0050 //80-prescale da system clock
#define TIMER1_FREQ 0x01f4 // 500Hz
#define TIMER_1_ISR_DECIMATION_10_MS 5
#define TIMER_1_ISR_DECIMATION_100_MS 50

//LCD
//#define OPENLCD_BAUDRATE 9600
//#define LCD_CONTRAST 20
//#define OPENLCD_DEFAULT_BG 0x00080808
//#define LCD_LEN (20*2)
//keys
//#define KEY_ENTER_GPIODAT_MASK (1<<0)
//#define KEY_ESC_GPIODAT_MASK (1<<1)

// Function Prototypes

#ifdef FLASH
#pragma CODE_SECTION(epwm3_timer_isr, "ramfuncs");
#pragma CODE_SECTION(cpu_timer1_isr, "ramfuncs");
#pragma CODE_SECTION(FIR32_alt_calc, "ramfuncs");
#pragma CODE_SECTION(write_and_send_stream_mbox, "ramfuncs");
#endif

//__interrupt void epwm3_timer_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void xint1_isr(void);


void GPIO_init(void);
void setup_start_stop(volatile TraveSM_t *tSM);
void write_and_send_stream_mbox(uint16_t mbox_n, uint16_t header, float32 v1,
                                float32 v2);
void setup_MAVG(volatile TraveSM_t *tSM);

//--------
// Globals
//--------
//lcd
//OpenLCD_t openlcd;
//const char test_string[] = OPENLCD_TEST_STRING;
//keys
//trave sm
volatile TraveSM_t traveSM;
F_t f0;
F_t f1;
F_t_handle force_handle[2];
volatile Uint32 Xint1Count;

ADS1220_t ads1220_0;
ADS1220_t ads1220_1;
ADS1220_t* ads1220_handle[2];
const uint32_t ads1220_cs_gpio_n[2]= {BXL_D_LC_CS0,BXL_D_LC_CS1};

volatile uint16_t wait_for_m_avg_cycles;
volatile uint16_t wait_for_sr400_cycles;
volatile uint16_t mes_wait_flag;
volatile uint16_t tare_cal_flag;
volatile _iq30 accumulator;
volatile uint16_t tare_cal_lc;
volatile uint16_t mes_acq_flag;
volatile uint16_t stream_flag;
volatile uint16_t flag_100_ms;
//-------------------------------
//FILTERING
#pragma DATA_SECTION(lpf_0, "firfilt");
#pragma DATA_SECTION(lpf_dbuffer_0,"firldb");
#pragma DATA_SECTION(lpf_1, "firfilt");
#pragma DATA_SECTION(lpf_dbuffer_1,"firldb");
#pragma DATA_SECTION(lpf_coefs, "coefffilt");

//FIR handle
FIR32_Handle lpf_handle[2];
//FIR obj
FIR32 lpf_0 = FIR32_ALT_DEFAULTS;
FIR32 lpf_1 = FIR32_ALT_DEFAULTS;
//FIR data buffer
int32_t lpf_dbuffer_0[LPF__ORDER + 1];
int32_t lpf_dbuffer_1[LPF__ORDER + 1];
//FIR coefficients
const int32_t lpf_coefs[LPF__ORDER + 1] = LPF__COEF;

//FIR decimation factor
#define DECIMATION_FACTOR_1600_TO_400 LPF_1600_TO_400_DECIMATION_FACTOR                                     //4
#define WAIT_FOR_TARE_CAL_s (1)//2 seconds
#define WAIT_FOR_SR400_CYCLES_STOP (uint16_t)(400/20)//s seconds

#define PROCESS_COUNTER_RESET PWM_FREQ_Hz*DATA_COUNTER_RESET_SECONDS //128
//-----------------------------------------
//moving average
M_AVG_Obj m_avg_obj_0;
int32_t m_avg_dbuffer_0[M_AVG_BUFFER_SIZE];

M_AVG_Obj m_avg_obj_1;
int32_t m_avg_dbuffer_1[M_AVG_BUFFER_SIZE];

M_AVG_Obj* m_avg_obj_handler[2];

extern Uint16 CoefffiltLoadStart;
extern Uint16 CoefffiltLoadEnd;
extern Uint16 CoefffiltRunStart;
extern Uint16 CoefffiltLoadSize;

void main(void)

{
    uint16_t i;
    //===================================
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    InitSysCtrl();

#ifdef FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);
    memcpy(&CoefffiltRunStart, &CoefffiltLoadStart, (Uint32)&CoefffiltLoadSize);
    //
    // Call Flash Initialization to setup flash waitstates
    // This function must reside in RAM
    //
    InitFlash();

#endif

    uint16_t msg_in_flag;
    msg_ctrl_t msg_ctrl_in;
    msg_notify_t msg_notify_out;
    //float lcd_acquire_time;
    //uint16_t reset_menu = 0;
    //========================
    // Step 2. Initalize GPIO
    BXL_D_LC_F28069_GPIO_setup();
    GPIO_init();
    //
    DEBUG_GPIO5 = 0;
    //turn led off
    LED_D10_BLUE = 1;
    LED_D9_RED = 1;

    //=====================================
    //Step 3. Initialize PIE and interrupts
    DINT;
    // Initialize PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2806x_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    InitPieVectTable();
    EALLOW;
    PieVectTable.TINT1 = &cpu_timer1_isr;
    PieVectTable.XINT1 = &xint1_isr;

    EDIS;

    //===============================================
    // Step 4. Initialize all the Device Peripherals:
    BXL_D_LC_F28069_PWM_J8_6_setup();

    BXL_D_LC_F28069_Spib_setup();
    BXL_D_LC_F28069_DRDY_EXT_INT_setup();
    //--------------------
    //init spib peripheral
    //initialize cpu timer 0
    CpuTimer1Regs.TCR.bit.TSS = 1;    // stop timer
    CpuTimer1Regs.TPR.all = TIMER1_PRESCALE;    // Pre-scale of SYSCLKOUT
    CpuTimer1Regs.TPRH.all = 0;
    CpuTimer1Regs.PRD.all = (SYS_CLK_FREQ_Hz / TIMER1_PRESCALE) / TIMER1_FREQ; // Initialize timer period
    CpuTimer1Regs.TCR.bit.TRB = 1; // Reload all counter register with period value
    CpuTimer1Regs.TCR.bit.TIE = 1; //enable interrupt
    //-------------------------
    // Initialize eCAN-A module
    ECan_init();
    ECan_SetupMbox();
    //---------------
    //Enable Interrupts
    //-----------------
    IER |= M_INT1; // Enable CPU INT1 which is connected to EPWM1-6 INT
    IER |= M_INT13; // Enable CPU INT3 which is connected to TINT0
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1; // Enable EPWM INTn in the PIE: Group 3 interrupt 1-6
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1; //
    EINT;
    // Enable Global interrupt INTMp
    ERTM;
    // Enable Global realtime interrupt DBGM
    //-------------------------
    // Initialize FIR32 object
    //=======
    //400
    lpf_handle[0] = &lpf_0;
    //
    lpf_handle[0]->order = LPF__ORDER;
    lpf_handle[0]->dbuffer_ptr = (int32_t *) lpf_dbuffer_0;
    lpf_handle[0]->coeff_ptr = (int32_t *) lpf_coefs;
    FIR32_alt_init(lpf_handle[0]);

    lpf_handle[1] = &lpf_1;
    //
    lpf_handle[1]->order = LPF__ORDER;
    lpf_handle[1]->dbuffer_ptr = (int32_t *) lpf_dbuffer_1;
    lpf_handle[1]->coeff_ptr = (int32_t *) lpf_coefs;
    FIR32_alt_init(lpf_handle[1]);

    //moving average object init
    m_avg_obj_handler[0] = &m_avg_obj_0;
    m_avg_obj_handler[1] = &m_avg_obj_1;
    M_AVG_init(m_avg_obj_handler[0], m_avg_dbuffer_0);
    M_AVG_init(m_avg_obj_handler[1], m_avg_dbuffer_1);
    //init force handle
    force_handle[0] = &f0;
    force_handle[1] = &f1;

    //ads1220
    ads1220_handle[0]=&ads1220_0;
    ads1220_handle[1]=&ads1220_1;
    BXL_D_LC_F28069_ADS1220_RESET();

    for (i = 0; i < BXL_D_LC_ADS1220_NUMBER; ++i) {
        ADS1220Init(ads1220_handle[i],
                    ads1220_cs_gpio_n[i],
                    &(BXL_D_LC_F28069_Spib_assert_cs),
                    &(BXL_D_LC_F28069_Spib_tx_byte),
                    &(BXL_D_LC_F28069_Spib_rx_byte));
        BXL_D_LC_ADS1220_setup(ads1220_handle[i]);
    }

    //openlcd
    //OpenLCD_init(&openlcd, &(Scia_can_tx_n_bytes), &(Scia_tx_byte));
    //OpenLCD_set_contrast(&openlcd, LCD_CONTRAST);
    //DELAY_US(1000000);
    //OpenLCD_clear(&openlcd);
    //openlcd.update_flag = 1;
    //-------------------------------------

    traveSM.SETTING.bit.m_avg_decimation_output_rate = 2;
    traveSM.SETTING.bit.m_avg_len = 3;    //16
    traveSM.SETTING.bit.m_avg_input = 1;

    traveSM.SETTING.bit.start_stop_mode = START_STOP_MODE_AUTO;
    traveSM.SETTING.bit.stop_input = 2;
    traveSM.SETTING.bit.start_input = 4;
    traveSM.start_force_threshold = 5.0;
    traveSM.stop_force_threshold = 2.0;

    setup_start_stop(&traveSM);

    traveSM.offset[0] = 20655545;
    traveSM.offset[1] = 20655545;
    traveSM.calibration[0] = 826.314819;
    traveSM.calibration[1] = 826.314819;

    traveSM.calibration_constant = 15.0f;
    traveSM.sd_stability_threshold = 0.001f;

    traveSM.state = IDLE;
    traveSM.CTRL.all = 0;
    traveSM.CTRL.bit.T_IDLE = 1;
    traveSM.CTRL.bit.SETUP = SETUP_MAVG;

    tare_cal_flag = 0;
    mes_acq_flag = 0;
    mes_wait_flag = 0;

    // run CPUTimer0
    CpuTimer1Regs.TCR.bit.TSS = 0;
    BXL_D_LC_F28069_ADS1220_SYNC_START();
    BXL_D_LC_F28069_DRDY_EXT_INT_enable(1);
    for (;;)
    {
        switch (traveSM.state)
        {
        case IDLE:

            //enter actions
            if (traveSM.CTRL.bit.T_IDLE)
            {
                stream_flag = 0;
                traveSM.CTRL.bit.T_IDLE = 0;
                msg_notify_out.header.bit.state = traveSM.state;
                msg_notify_out.header.bit.notification =
                        TRANSITION_NOTIFICATION;
                ECan_sendNotifyMBox(&msg_notify_out);
                msg_notify_out.header.bit.reply_id = 0;
            }
            // setup
            if (traveSM.CTRL.bit.SETUP != SETUP_NONE)
            {
                msg_notify_out.header.bit.state = traveSM.state;
                msg_notify_out.header.bit.notification = SETUP_NOTIFICATION
                        + traveSM.CTRL.bit.SETUP;
                switch (traveSM.CTRL.bit.SETUP)
                {
                case SETUP_MAVG:

                    setup_MAVG(&traveSM);
                    for (i = 0; i < 2; ++i)
                    {
                        M_AVG_setup(m_avg_obj_handler[i], traveSM.m_avg_len,
                                    traveSM.m_avg_input_p[i]);
                    }

                    //TODOdelay...wait filters to seattle
                    wait_for_m_avg_cycles = M_AVG_BUFFER_SIZE * 3;
                    while (wait_for_m_avg_cycles == 0)
                        ;

                    traveSM.CTRL.bit.SETUP = SETUP_NONE;
                    msg_notify_out.data = traveSM.SETTING.all;
                    break;

                case SETUP_START_STOP:
                    setup_start_stop(&traveSM);
                    msg_notify_out.data = traveSM.SETTING.all;
                    break;
                case SETUP_REC_START_VALUE:
                    msg_notify_out.data = traveSM.SETTING.all;
                    break;
                case SETUP_REC_STOP_VALUE:
                    msg_notify_out.data = traveSM.SETTING.all;
                    break;
                default:
                    break;
                }
                //send notify
                ECan_sendNotifyMBox(&msg_notify_out);

                msg_notify_out.header.bit.reply_id = 0;
                //
                traveSM.CTRL.bit.SETUP = SETUP_NONE;
            }

            //
            if (msg_in_flag)
            {
                msg_in_flag = 0;
                msg_notify_out.header.bit.reply_id = msg_ctrl_in.ctrl.bit.id;
                switch (msg_ctrl_in.ctrl.bit.ctrl)
                {
                case TARE:
                    traveSM.CTRL.bit.T_TARE = 1;
                    break;
                case CAL:
                    traveSM.CTRL.bit.T_CAL = 1;
                    break;
                case MES_WAIT:
                    traveSM.CTRL.bit.T_MES_WAIT = 1;
                    break;
                case SETUP_CTRL + SETUP_MAVG:
                    traveSM.CTRL.bit.SETUP = SETUP_MAVG;
                    traveSM.SETTING.all &= ~SETTINGS_MAVG_MASK;
                    traveSM.SETTING.all |= msg_ctrl_in.data
                            & SETTINGS_MAVG_MASK;
                    break;
                case GET_REG_CTRL + 1:
                    msg_notify_out.data = traveSM.SETTING.all;
                    msg_notify_out.header.bit.notification =
                            GET_REG_NOTIFICATION;
                    msg_notify_out.header.bit.set_get = 1;
                    ECan_sendNotifyMBox(&msg_notify_out);
                    break;
                case GET_REG_CTRL + 2:
                    msg_notify_out.data = traveSM.calibration_constant;
                    msg_notify_out.header.bit.notification =
                            GET_REG_NOTIFICATION;
                    msg_notify_out.header.bit.set_get = 2;
                    ECan_sendNotifyMBox(&msg_notify_out);
                    break;
                default:
                    if (msg_ctrl_in.ctrl.bit.ctrl < TRANSITION_CTRL + 9)
                    {
                        msg_notify_out.header.bit.notification =
                                ERROR_NOTIFICATION + ERR_TRANSITION_NOT_ALLOWED;
                    }
                    else
                    {
                        msg_notify_out.header.bit.notification =
                                ERROR_NOTIFICATION + ERR_INVALID_CTRL;
                    }
                    ECan_sendNotifyMBox(&msg_notify_out);
                    msg_notify_out.header.bit.reply_id = 0;
                    break;
                }

            }

            //Transitions
            if (traveSM.CTRL.bit.T_TARE)
            {
                traveSM.state = TARE;
            }
            else if (traveSM.CTRL.bit.T_CAL)
            {
                traveSM.state = CAL;
            }
            else if (traveSM.CTRL.bit.T_MES_WAIT)
            {
                traveSM.state = MES_WAIT;
            }
            break;

        case MES_WAIT:
            //entry action
            if (traveSM.CTRL.bit.T_MES_WAIT)
            {
                traveSM.CTRL.bit.T_MES_WAIT = 0;
                msg_notify_out.header.bit.state = traveSM.state;
                msg_notify_out.header.bit.notification =
                        TRANSITION_NOTIFICATION;
                ECan_sendNotifyMBox(&msg_notify_out);

                msg_notify_out.header.bit.reply_id = 0;
                LED_D10_BLUE = 0;
                traveSM.acquire_counter = 0;
                stream_flag = 1;
                mes_wait_flag = 1;
            }
            //auto
            if (mes_wait_flag == 0)
            {
                traveSM.CTRL.bit.T_MES_ACQ = 1;
            }
            else if (msg_in_flag)
            {
                msg_in_flag = 0;
                msg_notify_out.header.bit.reply_id = msg_ctrl_in.ctrl.bit.id;
                switch (msg_ctrl_in.ctrl.bit.ctrl)
                {
                case IDLE:
                    traveSM.CTRL.bit.T_IDLE = 1;
                    break;
                default:
                    msg_notify_out.header.bit.notification = ERROR_NOTIFICATION;
                    ECan_sendNotifyMBox(&msg_notify_out);

                    msg_notify_out.header.bit.reply_id = 0;
                    break;
                }

            }
            //Transitions
            if (traveSM.CTRL.bit.T_IDLE)
            {    //reset
                traveSM.state = IDLE;
                LED_D10_BLUE = 1;
                mes_wait_flag = 0;
            }
            else if (traveSM.CTRL.bit.T_MES_ACQ)
            {    //reset
                traveSM.state = MES_ACQ;
                mes_wait_flag = 0;
            }
            break;

        case MES_ACQ:
            //perform on entry actions
            if (traveSM.CTRL.bit.T_MES_ACQ)
            {
                traveSM.CTRL.bit.T_MES_ACQ = 0;
                LED_D9_RED = 0;
                msg_notify_out.header.bit.state = traveSM.state;
                msg_notify_out.header.bit.notification =
                        TRANSITION_NOTIFICATION;
                ECan_sendNotifyMBox(&msg_notify_out);

                msg_notify_out.header.bit.reply_id = 0;
                wait_for_m_avg_cycles = WAIT_FOR_SR400_CYCLES_STOP;
                mes_acq_flag = 1;
            }
             //Auto stop  Transitions
            if (mes_acq_flag == 0)
            {
                traveSM.CTRL.bit.T_MES_STOP = 1;

            }
            else if (msg_in_flag)
            {
                msg_in_flag = 0;
                msg_notify_out.header.bit.reply_id = msg_ctrl_in.ctrl.bit.id;
                switch (msg_ctrl_in.ctrl.bit.ctrl)
                {
                case MES_WAIT:
                    traveSM.CTRL.bit.T_MES_WAIT = 1;
                    break;
                default:
                    msg_notify_out.header.bit.notification = ERROR_NOTIFICATION;
                    ECan_sendNotifyMBox(&msg_notify_out);

                    msg_notify_out.header.bit.reply_id = 0;
                    break;
                }

            }
            //Transitions
            if (traveSM.CTRL.bit.T_MES_WAIT)
            {
                LED_D9_RED = 1;
                traveSM.state = MES_WAIT;
                mes_acq_flag = 0;
            }
            else if (traveSM.CTRL.bit.T_MES_STOP)
            {    //reset
                LED_D9_RED = 1;
                traveSM.state = MES_STOP;
                mes_acq_flag = 0;
            }
            break;

        case MES_STOP:
            if (traveSM.CTRL.bit.T_MES_STOP)
            {
                traveSM.CTRL.bit.T_MES_STOP = 0;
                msg_notify_out.header.bit.state = traveSM.state;
                msg_notify_out.header.bit.notification =
                        TRANSITION_NOTIFICATION;
                ECan_sendNotifyMBox(&msg_notify_out);

            }
            if (msg_in_flag)
            {
                msg_in_flag = 0;
                msg_notify_out.header.bit.reply_id = msg_ctrl_in.ctrl.bit.id;
                switch (msg_ctrl_in.ctrl.bit.ctrl)
                {
                case MES_WAIT:
                    traveSM.CTRL.bit.T_MES_WAIT = 1;
                    break;
                default:
                    msg_notify_out.header.bit.notification = ERROR_NOTIFICATION;
                    ECan_sendNotifyMBox(&msg_notify_out);

                    msg_notify_out.header.bit.reply_id = 0;
                    break;
                }

            }
            //Transitions
            if (traveSM.CTRL.bit.T_MES_WAIT)
            {    //reset
                traveSM.state = MES_WAIT;
            }

            break;

        case TARE:

            if (traveSM.CTRL.bit.T_TARE)
            {
                traveSM.CTRL.bit.T_TARE = 0;
                msg_notify_out.header.bit.state = traveSM.state;
                msg_notify_out.header.bit.notification =
                        TRANSITION_NOTIFICATION;
                ECan_sendNotifyMBox(&msg_notify_out);

                msg_notify_out.header.bit.reply_id = 0;

                wait_for_m_avg_cycles = traveSM.tare_cal_wait_mAvg_cycles; //2sec

                stream_flag = 1;
                accumulator=_IQ30(0.0);
                tare_cal_lc = 0;
                tare_cal_flag = 1;

            }
            if (msg_in_flag)
            {
                msg_in_flag = 0;
                msg_notify_out.header.bit.state = traveSM.state;
                msg_notify_out.header.bit.notification = ERROR_NOTIFICATION;
                ECan_sendNotifyMBox(&msg_notify_out);

                msg_notify_out.header.bit.reply_id = 0;

            }

            // exit
            if (tare_cal_flag == 0)
            {
                if (tare_cal_lc == 0)
                {
                    wait_for_m_avg_cycles = traveSM.tare_cal_wait_mAvg_cycles; //2sec
                    accumulator=_IQ30(0.0);
                    tare_cal_lc = 1;
                    tare_cal_flag = 1;
                }
                else if (tare_cal_lc == 1)
                {
                    tare_cal_lc = 2;
                }
            }

            break;

        case CAL:
            //
            if (traveSM.CTRL.bit.T_CAL)
            {
                traveSM.CTRL.bit.T_CAL = 0;
                msg_notify_out.header.bit.state = traveSM.state;
                msg_notify_out.header.bit.notification =
                        TRANSITION_NOTIFICATION;
                ECan_sendNotifyMBox(&msg_notify_out);

                msg_notify_out.header.bit.reply_id = 0;

                wait_for_m_avg_cycles = traveSM.tare_cal_wait_mAvg_cycles; //2sec
                stream_flag = 1;
                accumulator=_IQ30(0.0);
                tare_cal_lc = 0;
                tare_cal_flag = 1;
            }
            if (msg_in_flag)
            {
                msg_in_flag = 0;
                msg_notify_out.header.bit.state = traveSM.state;
                msg_notify_out.header.bit.notification = ERROR_NOTIFICATION;
                ECan_sendNotifyMBox(&msg_notify_out);

                msg_notify_out.header.bit.reply_id = 0;

            }
            if (flag_100_ms)
            {
                flag_100_ms = 0;

            }
            if (tare_cal_flag == 0)
            {
                if (tare_cal_lc == 0)
                {
                    wait_for_m_avg_cycles = traveSM.tare_cal_wait_mAvg_cycles; //2sec
                    accumulator=_IQ30(0.0);
                    tare_cal_lc = 1;
                    tare_cal_flag = 1;
                }
                else if (tare_cal_lc == 1)
                {
                    tare_cal_lc = 2;
                }
            }
            // exit

            if(1)//eys.pressed.esc)
            {
                tare_cal_flag = 0;
                //keys.pressed.esc = 0;
                traveSM.CTRL.bit.T_IDLE = 1;
                traveSM.state = IDLE;
            }
            break;
        }
        //clear message if not cleared and return response
        if (msg_in_flag)
        {
            msg_in_flag = 0;
            msg_notify_out.header.bit.state = traveSM.state;
            msg_notify_out.header.bit.notification = ERROR_NOTIFICATION;
            ECan_sendNotifyMBox(&msg_notify_out);

            msg_notify_out.header.bit.reply_id = 0;
        }
        //read control mailbox
        if ((ECanaRegs.CANRMP.all & (1 << MSG_CTRL_MBOX)))
        {
            msg_in_flag = ECan_getCTRLMBoxData(&msg_ctrl_in);
            msg_in_flag = 1;
        }

    }
}


__interrupt void
xint1_isr(void)
{
    uint16_t i;
    static uint16_t process_count = 0;
    static uint16_t sr_400_count = 0;
    //GpioDataRegs.GPBCLEAR.all = 0x4;   // GPIO34 is low
    DEBUG_GPIO5 = 1;
    Xint1Count++;
    if (++process_count == BXL_D_LC_DATARATE)
    {
        process_count = 0;
        sr_400_count = 0;
    }

    for (i = 0; i < 2; ++i)
    {
        ADS1220ReadData(ads1220_handle[i]);
    }
    //
    // Acknowledge this interrupt to get more from group 1
    //
    DEBUG_GPIO5 = 0;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

/*
__interrupt void epwm3_timer_isr(void)
{
    static uint16_t process_count = 0;
    static uint16_t sr_400_count = 0;
    static int32_t ads1246_conversion[2];
    msg_data_header_t msg_data_header;
    uint16_t i;

    DEBUG_GPIO5 = 1;
    //
    if (++process_count == PROCESS_COUNTER_RESET)
    {
        process_count = 0;
        sr_400_count = 0;
    }
    // run filters

    //---------------------
    //filter and decimation
    //---------------------
    //Decimation by 4 to 400Hz SR
    if (process_count % DECIMATION_FACTOR_1600_TO_400 == 0)
    {   //Start 400Hz Loop

        //msg_data_header
        msg_data_header.bit.isr_state = (mes_acq_flag << 1) + (mes_wait_flag)
                + (tare_cal_flag << (2+tare_cal_flag));
        msg_data_header.bit.counter = (0x0fff & sr_400_count);

        if (mes_acq_flag)
        {
            traveSM.acquire_counter++;
        }
        //
        for (i = 0; i < 2; ++i)
        {
            force_handle[i]->sr400 = _IQ30toF(lpf_1600_to_400_handle[i]->output)
                    * traveSM.calibration[i];
            //multiply by two because of different output format
            lpf_handle[i]->input = _IQmpy2(lpf_1600_to_400_handle[i]->output);
            FIR32_alt_calc(lpf_handle[i]);
        }
        //run cascaded filter

        if (stream_flag)
        {
            write_and_send_stream_mbox(
                    MSG_SR400_MBOX, msg_data_header.all,
                    (force_handle[0]->sr400 * FLOAT2INT_FACTOR),
                    (force_handle[1]->sr400 * FLOAT2INT_FACTOR));
        }

        if (process_count
                % (DECIMATION_FACTOR_1600_TO_400 * LPF__DECIMATION_FACTOR) == 0)
        {
            for (i = 0; i < 2; ++i)
            {
                force_handle[i]->sr100 = _IQ30toF(lpf_handle[i]->output)
                        * traveSM.calibration[i];
            }
            //run cascaded filter

            if (stream_flag)
            {
                write_and_send_stream_mbox(
                        MSG_SR100_MBOX, msg_data_header.all,
                        (force_handle[0]->sr100 * FLOAT2INT_FACTOR),
                        (force_handle[1]->sr100 * FLOAT2INT_FACTOR));
            }
        }
        //Decimation moving average rate
        if (process_count % traveSM.m_avg_decimation_input_rate == 0)
        {
            for (i = 0; i < 2; ++i)
            {
                M_AVG_run(m_avg_obj_handler[i]);
                force_handle[i]->movingAvg = _IQ30toF(m_avg_obj_handler[i]->avg)
                        * traveSM.calibration[i];
                force_handle[i]->movingSd = _IQ30toF(m_avg_obj_handler[i]->var)
                        * traveSM.calibration[i];

            }
            //tare or calibrate load cell sequentially
            if (tare_cal_flag)
            {
                if (force_handle[tare_cal_lc]->movingSd
                        >= traveSM.sd_stability_threshold)
                {
                    wait_for_m_avg_cycles =
                            traveSM.tare_cal_wait_mAvg_cycles;
                    accumulator=_IQ30(0.0);
                }else{
                    accumulator += _IQ30mpy( m_avg_obj_handler[tare_cal_lc]->avg , _IQ30(1.0/traveSM.tare_cal_wait_mAvg_cycles));
                }
                if (wait_for_m_avg_cycles-- == 0)
                { //set offset
                    tare_cal_flag = 0;
                    if (traveSM.state == TARE)
                    {
                        traveSM.offset[tare_cal_lc] += _IQmpy2(accumulator);
                    }
                    else if (traveSM.state == CAL)
                    {
                        traveSM.calibration[tare_cal_lc] =
                                traveSM.calibration_constant
                                        / _IQ30toF(accumulator);
                    }
                }

            }
            //send
            if (stream_flag
                    & (process_count % traveSM.m_avg_decimation_output_rate == 0))
            {
                write_and_send_stream_mbox(
                        MSG_MAVG_MBOX, msg_data_header.all,
                        (force_handle[0]->movingAvg * FLOAT2INT_FACTOR),
                        (force_handle[1]->movingAvg * FLOAT2INT_FACTOR));
                write_and_send_stream_mbox(
                        MSG_MSD_MBOX, msg_data_header.all,
                        (force_handle[0]->movingSd * FLOAT2INT_FACTOR),
                        (force_handle[1]->movingSd * FLOAT2INT_FACTOR));
            }
            //End MAVG
        }
        //mean value of both adc
        traveSM.force.sr400 = (force_handle[0]->sr400 + force_handle[1]->sr400)
                / 2.0;
        traveSM.force.movingAvg = (force_handle[0]->movingAvg
                + force_handle[1]->movingAvg) / 2.0;
        traveSM.force.movingSd = (force_handle[0]->movingSd
                + force_handle[1]->movingSd) / 2.0;
        //start stop conditions
        if (traveSM.SETTING.bit.start_stop_mode == START_STOP_MODE_AUTO)
        {
            if (mes_wait_flag)
            {
                if (*traveSM.start_value_p > traveSM.start_force_threshold)
                {
                    mes_wait_flag = 0;
                }

            }
            else if (mes_acq_flag)
            { //stop condition
                if (*traveSM.stop_value_p > traveSM.stop_force_threshold)
                {
                    wait_for_sr400_cycles = traveSM.stop_wait_sr400_cycles;
                    //TODO:sd stop
                }
                if (wait_for_sr400_cycles-- == 0)
                { //set offset
                    mes_acq_flag = 0;
                }
            }
        }
        sr_400_count++;
        //End 400Hz Loop
    }

}
*/

__interrupt void cpu_timer1_isr(void)
{
    static uint16_t decimation_counter = 0; //2MS
    decimation_counter++;
    //OpenLCD_update(&openlcd);
//
    if (decimation_counter % TIMER_1_ISR_DECIMATION_10_MS == 0)
    {
        //key_press_debounce(&keys);
    }
    if (decimation_counter % TIMER_1_ISR_DECIMATION_100_MS == 0)
    {
        flag_100_ms = 1;
    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}



void GPIO_init(void)
{
    EALLOW;
    //--------------
    //Init ECAN GPIO
    GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;   // Enable pull-up for GPIO30 (CANRXA)
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;   // Enable pull-up for GPIO31 (CANTXA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 3;   // Asynch qual for GPIO30 (CANRXA)
    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 1;

    //----------------
    //Init DEBUG GPIO
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;    //output
    //----------------
    // Initialize GPIOs for the LEDs and turn them off
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;

    EDIS;
    //----------------
}

void setup_MAVG(volatile TraveSM_t *tSM)
{
    uint16_t i;
    //moving_avg_init
    tSM->m_avg_len = 1 << (tSM->SETTING.bit.m_avg_len + 1);
    switch (tSM->SETTING.bit.m_avg_input)
    {
    case 1:
        for (i = 0; i < 2; ++i)
        {
            tSM->m_avg_input_p[i] = &(lpf_handle[i]->output);
        }
        tSM->m_avg_decimation_input_rate =
        DECIMATION_FACTOR_1600_TO_400 * LPF__DECIMATION_FACTOR;
        break;
    }
    tSM->m_avg_decimation_output_rate = 1
            << (tSM->SETTING.bit.m_avg_decimation_output_rate);
    tSM->tare_cal_wait_mAvg_cycles =200; //WAIT_FOR_TARE_CAL_s
            //* (1600 / tSM->m_avg_decimation_input_rate);
}

void setup_start_stop(volatile TraveSM_t *tSM)
{
    //start stop mode setup

    switch (tSM->SETTING.bit.start_input)
    {
    case 0:
        tSM->start_value_p = &tSM->force.sr400;
        break;
    case 1:
        tSM->start_value_p = &tSM->force.sr200;
        break;
    case 2:
        tSM->start_value_p = &tSM->force.sr100;
        break;
    case 3:
        tSM->start_value_p = &tSM->force.sr50;
        break;
    case 4:
        tSM->start_value_p = &tSM->force.movingAvg;
        break;
    }
    //
    switch (tSM->SETTING.bit.stop_input)
    {
    case 0:
        tSM->stop_value_p = &tSM->force.sr400;
        break;
    case 1:
        tSM->stop_value_p = &tSM->force.sr200;
        break;
    case 2:
        tSM->stop_value_p = &tSM->force.sr100;
        break;
    case 3:
        tSM->stop_value_p = &tSM->force.sr50;
        break;
    case 4:
        tSM->stop_value_p = &tSM->force.movingAvg;
        break;
    }
    tSM->stop_wait_sr400_cycles = WAIT_FOR_SR400_CYCLES_STOP;

}

//

void write_and_send_stream_mbox(uint16_t mbox_n, uint16_t header, float32 v1,
                                float32 v2)
{
    volatile struct MBOX *Mailbox_p = &ECanaMboxes.MBOX0 + mbox_n;
    Mailbox_p->MDL.word.HI_WORD = header;
    Mailbox_p->MDL.word.LOW_WORD = (int16_t) v1; //
    Mailbox_p->MDH.word.HI_WORD = (int16_t) v2; //
    ECan_sendMBox(mbox_n);
}


// End of File
//
