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
#include "include/ads1246.h"
#include "include/lpf_filters_coefs.h"
#include "modules/moving_avg.h"
#include "include/traveSM.h"
#include "openLCD/openlcd.h"

//SYSTEM definitions
#define SYS_CLK_FREQ_Hz 80000000 //80MHz guarda Step 1. Initialize System Control
#define LSPC_CLK_FREQ_Hz (SYS_CLK_FREQ_Hz/4) //20MHz guarda Step 1. Initialize System Control
#define PWM_FREQ_Hz 1600
#define DEBUG_GPIO5 GpioDataRegs.GPADAT.bit.GPIO5
#define LED_D9_RED   GpioDataRegs.GPBDAT.bit.GPIO34
#define LED_D10_BLUE  GpioDataRegs.GPBDAT.bit.GPIO39
//keys
#define KEY_ENTER_GPIODAT_MASK (1<<0)
#define KEY_ESC_GPIODAT_MASK (1<<1)

#define LCD_LEN (20*2)

//ADS1246 definitions
#define ADS1246_SPICLK_FREQ_HZ 2000000 //guarda datasheet per max min
#define ADS1246_SETTLING_TIME_us (578+10)//@ 2000Hz guarda datasheet
#define ADS1246_0_CS GpioDataRegs.GPBDAT.bit.GPIO51
#define ADS1246_1_CS GpioDataRegs.GPADAT.bit.GPIO3
#define ADS1246_01_RESET GpioDataRegs.GPADAT.bit.GPIO19
//#define ADS1246_1_START GpioDataRegs.GPADAT.bit.GPIO4
//#define ADS1246_2_START GpioDataRegs.GPADAT.bit.GPIO1
//#define ADS1246_1_DRDY GpioDataRegs.GPBDAT.bit.GPIO55
#define ADS1246_01_START_HIGH EPwm3Regs.AQCSFRC.bit.CSFA = 2 //PWM Action Qualifier continuus software force HIGH
#define ADS1246_01_START_LOW EPwm3Regs.AQCSFRC.bit.CSFA = 1 //PWM Action Qualifier continuus software force LOW
#define ADS1246_01_START_PWM EPwm3Regs.AQCSFRC.bit.CSFA = 0 //PWM Action Qualifier continuus software force disable

#define TIMER1_PRESCALE  0x0050 //80-prescale da system clock
#define TIMER1_FREQ 0x01f4 // 500Hz
#define TIMER_1_ISR_DECIMATION_10_MS 5
#define TIMER_1_ISR_DECIMATION_100_MS 50

//LCD
#define OPENLCD_BAUDRATE 9600
#define LCD_CONTRAST 20
#define OPENLCD_DEFAULT_BG 0x00080808
//communication

// Function Prototypes
void spia_trasmit_to_ads1246_noFIFO(uint16_t buf[], uint16_t bytesNumbers,
                                    uint16_t ads1246);
int32_t spia_read_ads1246_conversion(uint16_t ads1246);
#ifdef FLASH
#pragma CODE_SECTION(epwm3_timer_isr, "ramfuncs");
#pragma CODE_SECTION(cpu_timer1_isr, "ramfuncs");
#pragma CODE_SECTION(FIR32_alt_calc, "ramfuncs");
#pragma CODE_SECTION(write_and_send_stream_mbox, "ramfuncs");
#endif

__interrupt void epwm3_timer_isr(void);
__interrupt void cpu_timer1_isr(void);

uint16_t Scia_can_tx_n_bytes(void);
void Scia_tx_byte(uint16_t uint8_data);
void GPIO_init(void);
void setup_start_stop(volatile TraveSM_t *tSM);
uint16_t setup_ADS1246(uint16_t spia_buf[], uint16_t ads1246);
void reset_ADS1246(void);
void write_and_send_stream_mbox(uint16_t mbox_n, uint16_t header, float32 v1,
                                float32 v2);
void setup_MAVG(volatile TraveSM_t *tSM);

//--------
// Globals
//--------
//lcd
OpenLCD_t openlcd;
const char test_string[] = OPENLCD_TEST_STRING;
//keys
volatile keys_t keys;
//ads and ads spia
uint16_t spia_bytes_buf[13];
uint16_t spia_trasmit_n_bytes;
uint16_t spia_trasmit_flag;
//trave sm
volatile TraveSM_t traveSM;
F_t f0;
F_t f1;
F_t_handle force_handle[2];

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
#pragma DATA_SECTION(lpf_1600_to_400_0, "firfilt");
#pragma DATA_SECTION(lpf_1600_to_400_dbuffer_0,"firldb");
#pragma DATA_SECTION(lpf_1600_to_400_1, "firfilt");
#pragma DATA_SECTION(lpf_1600_to_400_dbuffer_1,"firldb");
#pragma DATA_SECTION(lpf_1600_to_400_coefs, "coefffilt");

#pragma DATA_SECTION(lpf_0, "firfilt");
#pragma DATA_SECTION(lpf_dbuffer_0,"firldb");
#pragma DATA_SECTION(lpf_1, "firfilt");
#pragma DATA_SECTION(lpf_dbuffer_1,"firldb");
#pragma DATA_SECTION(lpf_coefs, "coefffilt");

//FIR Handle
FIR32_Handle lpf_1600_to_400_handle[2];
//FIR obj
FIR32 lpf_1600_to_400_0 = FIR32_ALT_DEFAULTS;
FIR32 lpf_1600_to_400_1 = FIR32_ALT_DEFAULTS;
//FIR data buffer
int32_t lpf_1600_to_400_dbuffer_0[LPF_1600_TO_400_ORDER + 1];
int32_t lpf_1600_to_400_dbuffer_1[LPF_1600_TO_400_ORDER + 1];
//FIR coefficients
const int32_t lpf_1600_to_400_coefs[LPF_1600_TO_400_ORDER + 1] =
        LPF_1600_TO_400_COEF;

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
    float lcd_acquire_time;
    uint16_t reset_menu = 0;
    //========================
    // Step 2. Initalize GPIO:
    GPIO_init();
    //put ADS1246 in reset
    ADS1246_01_RESET = 0;
    ADS1246_0_CS = 1;
    ADS1246_1_CS = 1;
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
    PieVectTable.EPWM3_INT = &epwm3_timer_isr;
    PieVectTable.TINT1 = &cpu_timer1_isr;
    EDIS;

    //===============================================
    // Step 4. Initialize all the Device Peripherals:

    //--------------------
    //init spia peripheral
    //SPICLK requirements Accordung to ADS1246 datasheet for datarate 2000sps: 488ns < SPICLK period < 520ns => 2MHz
    // ADS1246 require CLK POLARITY 0  and PHASE 1: output on the rising edge, input is latched on the falling edge
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;     // Reset SPI
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;
    SpiaRegs.SPICCR.bit.SPICHAR = 7;        //SPIWORD 8 bit
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;
    SpiaRegs.SPICTL.bit.TALK = 1;
    SpiaRegs.SPISTS.all = 0x0000;
    SpiaRegs.SPIBRR = ((LSPC_CLK_FREQ_Hz / ADS1246_SPICLK_FREQ_HZ) - 1); // Baud rate register: SPICLK=LSPCLK/(SPIBRR+1), LSPCLK = SYSCLK/4=20MHZ, 20/(9+1)=2MHz:
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;      // Enable SPI

    //----------------------
    //init epwm3 peripheral
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;      // Stop all the TB clocks
    EDIS;
    //Timer Base submodule
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm3Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
    EPwm3Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm3Regs.TBPRD = (((SYS_CLK_FREQ_Hz / 8) / PWM_FREQ_Hz) + 6); //period register frequenza TBPRD+1
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;     // Count up
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0x01; // divide by 2 => TBCLK = SYSCLKOUT / (HSPCLKDIV × CLKDIV) = 80/8 =10MHZ
    EPwm3Regs.TBCTL.bit.CLKDIV = 0x02; //divide by 4
    //Compare submodule    //
    // Setup shadow register load immediate
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_IMMEDIATE;
    EPwm3Regs.CMPB = (ADS1246_SETTLING_TIME_us
            * (SYS_CLK_FREQ_Hz / (8 * 1000000)));
    //Action Qualifier submodule
    EPwm3Regs.AQCTLA.bit.CBU = AQ_CLEAR;
    EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;
    ADS1246_01_START_LOW;
    //Event Trigger submodule -> interrupt
    EPwm3Regs.ETSEL.bit.INTSEL = ET_CTRU_CMPB; // Enable INT on CMPB event up cunting
    //EPwm3Regs.ETSEL.bit.INTEN = 1;                  // Enable INT
    EPwm3Regs.ETPS.bit.INTPRD = ET_1ST;            // Generate INT on 1st event
    //
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;       // Start all the timers synced
    EDIS;
    //-------------------------
    //initialize SCIA
    SciaRegs.SCIFFTX.bit.SCIFFENA = 1;
    SciaRegs.SCIFFTX.bit.TXFFINT = 0;
    SciaRegs.SCIFFTX.bit.TXFFIENA = 1;
    SciaRegs.SCIFFCT.bit.FFTXDLY = 4;

    SciaRegs.SCICCR.all = 0x0007; //  1STOP,NP,8, async mode, idle-line protocol
    SciaRegs.SCICTL1.all = 0x0002;
    SciaRegs.SCICTL2.all = 0x0003;
    SciaRegs.SCICTL2.bit.TXINTENA = 0;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 0;
    SciaRegs.SCIHBAUD = 0x0001;       //BRR = LPSCLK/(BAUD*8)-1
    SciaRegs.SCILBAUD = 0x0003;
    SciaRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
    //-------------
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
    //Initialize eqep1
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;

    //
    // PCRM=00 mode - QPOSCNT reset on index event
    //
    EQep1Regs.QEPCTL.bit.PCRM = 1;
    EQep1Regs.QPOSMAX = 0xffffffff;
    EQep1Regs.QPOSINIT = 0x7fffffff;     //Initialization Value
    EQep1Regs.QEPCTL.bit.QPEN = 1;      // QEP enable
    EQep1Regs.QEPCTL.bit.SWI = 1;         //Software Initialization
    //---------------
    //Enable Interrupts
    //-----------------
    IER |= M_INT3; // Enable CPU INT3 which is connected to EPWM1-6 INT
    IER |= M_INT13; // Enable CPU INT3 which is connected to TINT0
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1; // Enable EPWM INTn in the PIE: Group 3 interrupt 1-6

    EINT;
    // Enable Global interrupt INTMp
    ERTM;
    // Enable Global realtime interrupt DBGM
    //-------------------------
    // Initialize FIR32 object
    //1600
    lpf_1600_to_400_handle[0] = &lpf_1600_to_400_0;
    //
    lpf_1600_to_400_handle[0]->order = LPF_1600_TO_400_ORDER;
    lpf_1600_to_400_handle[0]->dbuffer_ptr =
            (int32_t *) lpf_1600_to_400_dbuffer_0;
    lpf_1600_to_400_handle[0]->coeff_ptr = (int32_t *) lpf_1600_to_400_coefs;
    FIR32_alt_init(lpf_1600_to_400_handle[0]);

    lpf_1600_to_400_handle[1] = &lpf_1600_to_400_1;
    lpf_1600_to_400_handle[1]->order = LPF_1600_TO_400_ORDER;
    lpf_1600_to_400_handle[1]->dbuffer_ptr =
            (int32_t *) lpf_1600_to_400_dbuffer_1;
    lpf_1600_to_400_handle[1]->coeff_ptr = (int32_t *) lpf_1600_to_400_coefs;
    FIR32_alt_init(lpf_1600_to_400_handle[1]);
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

    //ads1246
    ADS1246_01_RESET = 0;
    ADS1246_0_CS = 1;
    ADS1246_1_CS = 1;
    reset_ADS1246();
    setup_ADS1246(spia_bytes_buf, 0);
    setup_ADS1246(spia_bytes_buf, 1);

    //openlcd
    OpenLCD_init(&openlcd, &(Scia_can_tx_n_bytes), &(Scia_tx_byte));
    OpenLCD_set_contrast(&openlcd, LCD_CONTRAST);
    DELAY_US(1000000);
    OpenLCD_clear(&openlcd);
    openlcd.update_flag = 1;
    //keys
    key_press_init(&keys, &GpioDataRegs.GPADAT.all,
    KEY_ENTER_GPIODAT_MASK | KEY_ESC_GPIODAT_MASK);
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
                //clear id
                OpenLCD_clear(&openlcd);
                OpenLCD_print_string(&openlcd, "    IDLE", LCD_COLUMN_STATE);
                //reset rotation to 0
                key_get_rotations_clicks(&keys);
                reset_menu = 1;
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
                    OpenLCD_print_string(&openlcd, "SETUP FILTERS       ",
                                         OPENLCD_COLS);
                    //stop pwm3 interrupt
                    EPwm3Regs.ETSEL.bit.INTEN = 0;
                    setup_MAVG(&traveSM);
                    for (i = 0; i < 2; ++i)
                    {
                        M_AVG_setup(m_avg_obj_handler[i], traveSM.m_avg_len,
                                    traveSM.m_avg_input_p[i]);
                    }
                    //start pwm and enable interrupt
                    ADS1246_01_START_PWM;
                    EPwm3Regs.ETSEL.bit.INTEN = 1;
                    //TODOdelay...wait filters to seattle
                    wait_for_m_avg_cycles = M_AVG_BUFFER_SIZE * 3;
                    while (wait_for_m_avg_cycles == 0)
                        ;

                    traveSM.CTRL.bit.SETUP = SETUP_NONE;
                    msg_notify_out.data = traveSM.SETTING.all;
                    OpenLCD_clear_row_from(&openlcd, OPENLCD_COLS);
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
            else if (reset_menu)
            {
                reset_menu = 0;
                run_IDLE_menu(&traveSM, &keys, 1);
            }
            else
            {
                run_IDLE_menu(&traveSM, &keys, 0);
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
            else if (traveSM.CTRL.bit.T_SETTINGS)
            {
                traveSM.state = SETTINGS;
            }
            break;
        case SETTINGS:
            //enter actions
            if (traveSM.CTRL.bit.T_SETTINGS)
            {
                stream_flag = 0;
                traveSM.CTRL.bit.T_SETTINGS = 0;
                msg_notify_out.header.bit.state = traveSM.state;
                msg_notify_out.header.bit.notification =
                        TRANSITION_NOTIFICATION;
                ECan_sendNotifyMBox(&msg_notify_out);

                msg_notify_out.header.bit.reply_id = 0;
                OpenLCD_clear(&openlcd);
                OpenLCD_print_string(&openlcd, "SETTINGS", LCD_COLUMN_STATE);
                run_SETTINGS_menu(&traveSM, &keys, &msg_notify_out, 1);
            }

            if (run_SETTINGS_menu(&traveSM, &keys, &msg_notify_out, 0))
            {
                msg_notify_out.header.bit.state = traveSM.state;
                ECan_sendNotifyMBox(&msg_notify_out);

                msg_notify_out.header.bit.reply_id = 0;
            }

            if (msg_in_flag)
            {
                msg_in_flag = 0;
                msg_notify_out.header.bit.state = traveSM.state;
                msg_notify_out.header.bit.notification = ERROR_NOTIFICATION;
                ECan_sendNotifyMBox(&msg_notify_out);

                msg_notify_out.header.bit.reply_id = 0;

            }
            //Transitions
            if (traveSM.CTRL.bit.T_IDLE)
            {    //reset
                traveSM.state = IDLE;
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
                //OpenLCD_clear(&openlcd);
                OpenLCD_clear_row_from(&openlcd, 60);
                OpenLCD_print_string(&openlcd, "MES WAIT", LCD_COLUMN_STATE);
                OpenLCD_print_string(&openlcd, LCD_MES_TEXT,
                                     LCD_MES_TEXT_START);
                LED_D10_BLUE = 0;
                traveSM.acquire_counter = 0;
                //lcd_acquire_time = 0.0;
                //OpenLCD_print_float(&openlcd, lcd_acquire_time, 3, 2,
                //LCD_TIME_START,
                //                    1);
                stream_flag = 1;
                mes_wait_flag = 1;
            }
            if (flag_100_ms)
            {
                flag_100_ms = 0;
                OpenLCD_print_float(&openlcd, traveSM.force.movingAvg, 3, 2,
                                    LCD_FORCE_START, 1);
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
            //Button Transitions
            else if (keys.pressed.esc)
            {
                keys.pressed.esc = 0;
                traveSM.CTRL.bit.T_IDLE = 1;
            }
            else if (keys.pressed.enter)
            {
                keys.pressed.enter = 0;
                if (traveSM.SETTING.bit.start_stop_mode
                        == START_STOP_MODE_MANUAL)
                {
                    traveSM.CTRL.bit.T_MES_ACQ = 1;
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
                OpenLCD_clear(&openlcd);
                OpenLCD_print_string(&openlcd, " MES ACQ", LCD_COLUMN_STATE);
                OpenLCD_print_string(&openlcd, LCD_MES_TEXT,
                                     LCD_MES_TEXT_START);
                wait_for_m_avg_cycles = WAIT_FOR_SR400_CYCLES_STOP;
                mes_acq_flag = 1;
            }
            if (flag_100_ms)
            {
                flag_100_ms = 0;
                lcd_acquire_time = ((float) traveSM.acquire_counter) / 400;
                OpenLCD_print_float(&openlcd, traveSM.force.movingAvg, 3, 2,
                                    LCD_FORCE_START, 1);
                OpenLCD_print_float(&openlcd, lcd_acquire_time, 3, 2,
                                    LCD_TIME_START, 1);
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
            else if (keys.pressed.esc)
            {
                keys.pressed.esc = 0;
                traveSM.CTRL.bit.T_MES_WAIT = 1;
            }
            else if (keys.pressed.enter)
            {   //quando resettare key?
                keys.pressed.enter = 0;
                if (traveSM.SETTING.bit.start_stop_mode
                        == START_STOP_MODE_MANUAL)
                {
                    traveSM.CTRL.bit.T_MES_STOP = 1;
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

                msg_notify_out.header.bit.reply_id = 0;
                OpenLCD_clear(&openlcd);
                OpenLCD_print_string(&openlcd, "MES STOP", LCD_COLUMN_STATE);
                //OpenLCD_print_string(&openlcd, LCD_MES_STOP_TEXT,
                //LCD_MES_TEXT_START);
                //OpenLCD_print_float(&openlcd, traveSM.force.movingAvg, 4, 2,
                //LCD_FORCE_START,
                //                    1);
                lcd_acquire_time = ((float) traveSM.acquire_counter) / 400;
                OpenLCD_print_float(&openlcd, lcd_acquire_time, 4, 2,
                                    LCD_TIME_START, 1);

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
            // wait till button reset do nothing
            else //if (keys.pressed.esc)
            {
                //keys.pressed.esc = 0;
                traveSM.CTRL.bit.T_MES_WAIT = 1;
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
                OpenLCD_clear(&openlcd);
                OpenLCD_print_string(&openlcd, "    TARE", LCD_COLUMN_STATE);
                OpenLCD_print_string(&openlcd, LCD_TARE_CAL_TEXT,
                                     LCD_TARE_CAL_START);

                wait_for_m_avg_cycles = traveSM.tare_cal_wait_mAvg_cycles; //2sec
                OpenLCD_print_string(&openlcd, "TARE LC0", 40);
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
                OpenLCD_print_float(&openlcd, traveSM.force.movingSd, 3, 2,
                                    LCD_SD_START, 1);
            }

            // exit
            if (tare_cal_flag == 0)
            {
                if (tare_cal_lc == 0)
                {
                    OpenLCD_print_string(&openlcd, "OK", 50);
                    OpenLCD_print_string(&openlcd, "TARE LC1", 60);
                    wait_for_m_avg_cycles = traveSM.tare_cal_wait_mAvg_cycles; //2sec
                    accumulator=_IQ30(0.0);
                    tare_cal_lc = 1;
                    tare_cal_flag = 1;
                }
                else if (tare_cal_lc == 1)
                {
                    OpenLCD_print_string(&openlcd, "OK", 70);
                    tare_cal_lc = 2;
                }
            }

            if (keys.pressed.esc)
            {
                tare_cal_flag = 0;
                keys.pressed.esc = 0;
                traveSM.CTRL.bit.T_IDLE = 1;
                traveSM.state = IDLE;
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
                OpenLCD_clear(&openlcd);
                OpenLCD_print_string(&openlcd, "     CAL", LCD_COLUMN_STATE);
                OpenLCD_print_string(&openlcd, LCD_TARE_CAL_TEXT,
                                     LCD_TARE_CAL_START);

                wait_for_m_avg_cycles = traveSM.tare_cal_wait_mAvg_cycles; //2sec
                OpenLCD_print_string(&openlcd, "CAL LC0", 40);
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
                OpenLCD_print_float(&openlcd, traveSM.force.movingSd, 3, 2,
                                    LCD_SD_START, 1);

            }
            if (tare_cal_flag == 0)
            {
                if (tare_cal_lc == 0)
                {
                    OpenLCD_print_string(&openlcd, "OK", 50);
                    OpenLCD_print_string(&openlcd, "CAL LC1", 60);
                    wait_for_m_avg_cycles = traveSM.tare_cal_wait_mAvg_cycles; //2sec
                    accumulator=_IQ30(0.0);
                    tare_cal_lc = 1;
                    tare_cal_flag = 1;
                }
                else if (tare_cal_lc == 1)
                {
                    OpenLCD_print_string(&openlcd, "OK", 70);
                    tare_cal_lc = 2;
                }
            }
            // exit

            if (keys.pressed.esc)
            {
                tare_cal_flag = 0;
                keys.pressed.esc = 0;
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

//
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
    for (i = 0; i < 2; ++i)
    {
        ads1246_conversion[i] = spia_read_ads1246_conversion(i);
        lpf_1600_to_400_handle[i]->input = ads1246_conversion[i]
                - traveSM.offset[i];
        FIR32_alt_calc(lpf_1600_to_400_handle[i]);

    }
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
//toggle led
    DEBUG_GPIO5 = 0;
    EPwm3Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void cpu_timer1_isr(void)
{
    static uint16_t decimation_counter = 0; //2MS
    decimation_counter++;
    OpenLCD_update(&openlcd);
//
    if (decimation_counter % TIMER_1_ISR_DECIMATION_10_MS == 0)
    {
        key_press_debounce(&keys);
    }
    if (decimation_counter % TIMER_1_ISR_DECIMATION_100_MS == 0)
    {
        flag_100_ms = 1;
    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void spia_trasmit_to_ads1246_noFIFO(uint16_t buf[], uint16_t bytesNumbers,
                                    uint16_t ads1246)
{
    uint16_t i;
    if (ads1246 == 0)
    {
        ADS1246_0_CS = 0; //CS1 low
    }
    else if (ads1246 == 1)
    {
        ADS1246_1_CS = 0; //CS2 low
    }
    DELAY_US(1); //at least 10ns
    for (i = 0; i < bytesNumbers; i++)
    {
        SpiaRegs.SPITXBUF = (buf[i] & 0xff) << 8;      //Send data
        while (!SpiaRegs.SPISTS.bit.INT_FLAG)
            ;
        buf[i] = (SpiaRegs.SPIRXBUF & 0xff);
    }
//delay at least 0.5us
    DELAY_US(1);
    if (ads1246 == 0)
    {
        ADS1246_0_CS = 1; //CS1 high
    }
    else if (ads1246 == 1)
    {
        ADS1246_1_CS = 1; //CS2 high
    }

}

int32_t spia_read_ads1246_conversion(uint16_t ads1246)
{
    const uint16_t rdata = (RDATA << 8);
    const uint16_t nop = (NOP << 8);
    uint16_t i;
    int32_t value;

    if (ads1246 == 0)
    {
        ADS1246_0_CS = 0; //CS1 low
    }
    else if (ads1246 == 1)
    {
        ADS1246_1_CS = 0; //CS2 low
    }
    DELAY_US(1); //at least 10ns
//TODO: use FIFO???
//TODO: nicer use of for!
    SpiaRegs.SPITXBUF = rdata;      //Send data
    while (!SpiaRegs.SPISTS.bit.INT_FLAG)
        ;
    value = SpiaRegs.SPIRXBUF;
    value = 0;
    i = 24;
    while (i != 0)
    {
        i -= 8;
        SpiaRegs.SPITXBUF = nop;      //Send data
        while (!SpiaRegs.SPISTS.bit.INT_FLAG)
            ;
        value |= ((uint32_t) (SpiaRegs.SPIRXBUF & 0xff) << i);
    }
//delay at least 0.5us
    DELAY_US(1);
    if (ads1246 == 0)
    {
        ADS1246_0_CS = 1; //CS1 high
    }
    else if (ads1246 == 1)
    {
        ADS1246_1_CS = 1; //CS2 high
    }
    return (value << 8);
}

uint16_t Scia_can_tx_n_bytes(void)
{
    return (4 - SciaRegs.SCIFFTX.bit.TXFFST); //number of free words remaining in TXFIFO
}

void Scia_tx_byte(uint16_t uint8_data)
{
    SciaRegs.SCITXBUF = uint8_data;
}

void key_press_init(volatile keys_t *keys, volatile uint32_t *gpio_data_regs,
                    uint32_t mask)
{
    keys->pressed.enter = 0;
    keys->pressed.esc = 0;
    keys->pressed.rsvd = 0;
    keys->_lastPOSCNT = EQep1Regs.QPOSCNT;
    keys->_s = 0;
    keys->_mask = mask;
    keys->_ct0 = 0xFFFFFFFF;
    keys->_ct1 = 0xFFFFFFFF;
    keys->_state = 0;
    keys->_gpio_data_regs = gpio_data_regs;
}

void key_press_debounce(volatile keys_t *keys)
{
    uint32_t i;
    uint32_t POSCNT;

    POSCNT = EQep1Regs.QPOSCNT;
    i = (*keys->_gpio_data_regs) & keys->_mask;
    i = keys->_s ^ ~i;         // key changed ? funzione per acltive low,
                               // altrimenti solo key_pin
    keys->_ct0 = ~(keys->_ct0 & i);                      // reset or count ct0
    keys->_ct1 = keys->_ct0 ^ (keys->_ct1 & i);           // reset or count ct1
    i &= keys->_ct0 & keys->_ct1;                   // count until roll over ?
    keys->_s ^= i;                             // then toggle debounced state
    keys->_state |= keys->_s & i;                    // 0->1: key press detect

    if (keys->_state & KEY_ENTER_GPIODAT_MASK)
    {
        keys->pressed.enter = 1;
        keys->_state &= (~KEY_ENTER_GPIODAT_MASK);

    }
    if (keys->_state & KEY_ESC_GPIODAT_MASK)
    {
        keys->pressed.esc = 1;
        keys->_state &= (~KEY_ESC_GPIODAT_MASK);
    }

}

int16_t key_get_rotations_clicks(volatile keys_t *keys)
{
    int16_t click;
    uint32_t POSCNT;
    POSCNT = EQep1Regs.QPOSCNT;
    click = (int16_t) (POSCNT - keys->_lastPOSCNT) / 2;
    if (click != 0)
    {
        keys->_lastPOSCNT = POSCNT;           // -(POSCNT- keys->_lastPOSCNT)%2;
    }
    return click;
}
void GPIO_init(void)
{
    EALLOW;

    //-------------------
    //ADS1246 related PIN
    //Init SPIA GPIO
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;  // Enable pull-up on GPIO16 (SPISIMOA)
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;  // Enable pull-up on GPIO17 (SPISOMIA)
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;  // Enable pull-up on GPIO18 (SPICLKA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3; // Asynch input GPIO16 (SPISIMOA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // Asynch input GPIO17 (SPISOMIA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3; // Asynch input GPIO18 (SPICLKA)
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1; // Configure GPIO16 as SPISIMOA
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1; // Configure GPIO17 as SPISOMIA
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1; // Configure GPIO18 as SPICLKA
    // CS1(GPIO51) e CS2(GPIO3)
    GpioCtrlRegs.GPBPUD.bit.GPIO51 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO51 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;
    // RESET(GPIO19)
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;    //output
    // START1(GPIO4 epwm) e START2(GPIO1)
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Disable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   //mux to pwm
    // DRDY1(GPIO55) e DRDY 2(GPIO2)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO55 = 0; //input

    //--------------
    //Init ECAN GPIO
    GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;   // Enable pull-up for GPIO30 (CANRXA)
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;   // Enable pull-up for GPIO31 (CANTXA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 3;   // Asynch qual for GPIO30 (CANRXA)
    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 1;

    //----------------------
    //SCI tx per lcd seriale
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;   // Enable pull-up for GPIO12 (SCITXDA)
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 2;

    //----------------
    //Init DEBUG GPIO
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;    //output
    //------------------
    //Init key GPIO
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;   // Enable pullup on GPIO0
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 0;   //input

    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;   // Enable pullup on GPIO0
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 0; //input
    //----------------
    // Initialize GPIOs for the LEDs and turn them off
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;
    //---------------------
    //Init GPIO for qep
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;   // Enable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;   // Enable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 0;  // Sync to SYSCLKOUT GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 0;  // Sync to SYSCLKOUT GPIO21 (EQEP1B)

    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1;   // Configure GPIO20 as EQEP1A
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1;   // Configure GPIO21 as EQEP1B

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
    case 0:
        for (i = 0; i < 2; ++i)
        {
            tSM->m_avg_input_p[i] = &(lpf_1600_to_400_handle[i]->output);
        }
        tSM->m_avg_decimation_input_rate =
        DECIMATION_FACTOR_1600_TO_400;
        break;
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

void reset_ADS1246(void)
{
    ADS1246_01_START_HIGH;
    //Reset ADS1246
    ADS1246_01_RESET = 0;
    DELAY_US(1000);
    ADS1246_01_RESET = 1;
    DELAY_US(10000);
}

uint16_t setup_ADS1246(uint16_t spia_buf[], uint16_t ads1246)
{
    uint16_t i;
    uint16_t spia_trasmit_n_bytes;
    ADS1246_01_START_HIGH;
    //Setup  ADS1246
    spia_trasmit_n_bytes = 6;
    spia_buf[0] = WREG;
    spia_buf[1] = 3;
    spia_buf[2] = BCS_RESET;
    spia_buf[3] = VBIAS_RESET;
    spia_buf[4] = MUX1_RESET;
    spia_buf[5] = PGA2_128 | DOR3_2000;
    spia_trasmit_to_ads1246_noFIFO(spia_buf, spia_trasmit_n_bytes, ads1246);

    DELAY_US(20000);
    //Read  ADS1246 register as control
    //TODO: raise error if necessary
    spia_trasmit_n_bytes = 13;
    spia_buf[0] = RREG;
    spia_buf[1] = 10;
    for (i = 2; i < 13; i++)
    {
        spia_buf[i] = NOP;
    }
    spia_trasmit_to_ads1246_noFIFO(spia_buf, spia_trasmit_n_bytes, ads1246);
    DELAY_US(10000);
    return 1;
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

void run_IDLE_menu(volatile TraveSM_t *tSM, volatile keys_t *key,
                   uint16_t reset)
{
#define MENU_ITEMS 5
    static int16_t menu_state = 0;
    static int16_t click;
    const char *lcd_menu_text[MENU_ITEMS] = { ">MEASURE    ", ">TARE       ",
                                              ">CALIBRATE  ", ">SETTTINGS  ",
                                              ">SCALE      " };
#define ROW 3
    //assumere 1 tasto alla volta
    //i tasti che non hanno effeto vengono azzerati
    //
    click = key_get_rotations_clicks(key);
    if (reset)
    {
        menu_state = 0;
        OpenLCD_print_string(&openlcd, lcd_menu_text[menu_state],
        ROW * OPENLCD_COLS);
    }
    else if (key->pressed.esc)
    {
        menu_state = 0;
        key->pressed.esc = 0;
    }
    else if (click != 0)
    {
        menu_state = add_value_clicks(menu_state, click, 0, MENU_ITEMS - 1);
        OpenLCD_print_string(&openlcd, lcd_menu_text[menu_state],
        ROW * OPENLCD_COLS);
    }
    else if (key->pressed.enter)
    {
        switch (menu_state)
        {
        case 0:
            tSM->CTRL.bit.T_MES_WAIT = 1;
            menu_state = 0;
            break;
        case 1:
            tSM->CTRL.bit.T_TARE = 1;
            menu_state = 0;
            break;
        case 2:
            tSM->CTRL.bit.T_CAL = 1;
            menu_state = 0;
            break;
        case 3:
            tSM->CTRL.bit.T_SETTINGS = 1;
            menu_state = 0;
            break;
        case 4:
            break;
        }
        key->pressed.enter = 0;
    }
    //azzera tutti i key
}

uint16_t run_SETTINGS_menu(volatile TraveSM_t *tSM, volatile keys_t *key,
                           msg_notify_t *msg_notify, uint16_t reset)
{
#define MENU_SETTINGS_ITEMS 9
    static int16_t menu_state = 0;
    int16_t click;
    static uint16_t reset2 = 0;
    uint16_t send = 0;
    const char *lcd_menu_text[MENU_SETTINGS_ITEMS] = { "Moving avg. length>",
                                                       "Moving avg. input >",
                                                       "Moving avg. dec   >",
                                                       "Start/stop mode   >",
                                                       "Start input       >",
                                                       "Stop input        >",
                                                       "Start value (Kg)  >",
                                                       "Stop value  (Kg)  >",
                                                       "Cal. value  (Kg)  >",

    };
    static uint16_t set_value;

    //assumere 1 tasto alla volta
    //i tasti che non hanno effeto vengono azzerati
    //
    click = key_get_rotations_clicks(key);
    if (reset | reset2)
    {
        menu_state = 0;
        OpenLCD_print_string(&openlcd, lcd_menu_text[menu_state], 20);
        OpenLCD_clear_row_from(&openlcd, 40);
        reset2 = 0;

    }
    else if (key->pressed.esc)
    {
        if (menu_state < MENU_SETTINGS_ITEMS)
        {
            tSM->CTRL.bit.T_IDLE = 1;
        }
        else
        {
            reset2 = 1;
        }
        key->pressed.esc = 0;
    }
    else if (click != 0)
    {
        if (menu_state < MENU_SETTINGS_ITEMS)
        {
            menu_state = add_value_clicks(menu_state, click, 0,
            MENU_SETTINGS_ITEMS - 1);
            OpenLCD_print_string(&openlcd, lcd_menu_text[menu_state], 20);

        }
        //
        else if (menu_state == 10)
        {
            set_value = add_value_clicks(set_value, click, 1, 7);
            OpenLCD_print_uint16(&openlcd, set_value, 40, 1);
        }
        else if (menu_state == 11)
        {
            set_value = add_value_clicks(set_value, click, 1, 3);
            OpenLCD_print_uint16(&openlcd, set_value, 40, 1);
        }
        else if (menu_state == 12)
        {
            set_value = add_value_clicks(set_value, click, 1, 4);
            OpenLCD_print_uint16(&openlcd, set_value, 40, 1);
        }
        else if (menu_state == 18)
        {
            set_value = add_value_clicks(set_value, click, 10, 100);
            OpenLCD_print_uint16(&openlcd, set_value, 40, 1);
        }

    }
    else if (key->pressed.enter)
    {
        switch (menu_state)
        {
        case 0:    //mavg len
            menu_state += 10;
            set_value = (uint16_t) tSM->SETTING.bit.m_avg_len;
            OpenLCD_print_uint16(&openlcd, set_value, 40, 1);
            break;
        case 1:    //mavg len
            menu_state += 10;
            set_value = (uint16_t) tSM->SETTING.bit.m_avg_input;
            OpenLCD_print_uint16(&openlcd, set_value, 40, 1);
            break;
        case 2:    //mavg len
            menu_state += 10;
            set_value =
                    (uint16_t) tSM->SETTING.bit.m_avg_decimation_output_rate;
            OpenLCD_print_uint16(&openlcd, set_value, 40, 1);
            break;
        case 8:
            menu_state += 10;
            set_value = (uint16_t) tSM->calibration_constant;
            OpenLCD_print_uint16(&openlcd, set_value, 40, 1);
            break;
            //===========================
        case 10:
            tSM->SETTING.bit.m_avg_len = set_value;
            tSM->CTRL.bit.SETUP = SETUP_MAVG;
            tSM->CTRL.bit.T_IDLE = 1;
            menu_state = 0;
            break;
        case 11:
            tSM->SETTING.bit.m_avg_input = set_value;
            tSM->CTRL.bit.SETUP = SETUP_MAVG;
            tSM->CTRL.bit.T_IDLE = 1;
            menu_state = 0;
            break;
        case 12:
            tSM->SETTING.bit.m_avg_decimation_output_rate = set_value;
            tSM->CTRL.bit.SETUP = SETUP_MAVG;
            tSM->CTRL.bit.T_IDLE = 1;
            menu_state = 0;
            break;
        case 18:
            tSM->calibration_constant = (float) set_value;
            menu_state = 0;
            reset2 = 1;
            msg_notify->header.bit.notification = SET_REG_NOTIFICATION + 1;
            msg_notify->data = tSM->calibration_constant;
            send = 1;
            break;
        default:
            menu_state = 0;
        }
        key->pressed.enter = 0;
    }
    return send;
    //azzera tutti i key
}

uint16_t add_value_clicks(uint16_t value, uint16_t clicks, uint16_t min,
                          uint16_t max)
{
    value += clicks;
    if (value < min)
    {
        value = min;
    }
    else if (value > max)
    {
        value = max;
    }
    return value;

}
// End of File
//
