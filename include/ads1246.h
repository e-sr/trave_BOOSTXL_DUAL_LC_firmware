#ifndef ads1246_h
#define ads1246_h

/* For information to the register and settings see manual page (p..) */


/* ADS1246 Register (see p42 for Register Map) */

#define		BCS     0x00 //Multiplexer Control Register 0
#define		VBIAS 	  0X01 //Bias Voltage Register
#define		MUX1     0x02 //Multiplexer Control REgister 1
#define		SYS0	  0x03 //System Control Register 0
#define		OFC0	  0X04 //Offset Calibration Coefficient Register 0
#define		OFC1	  0x05 //Offset Calibration Coefficient Register 1
#define		OFC2	  0x06 //Offset Calibration Coefficient Register 2
#define		FSC0 	  0x07 //Full scale Callibration Coefficient Register 0
#define		FSC1	  0x08 //Full scale Callibration Coefficient Register 1
#define		FSC2	  0x09 //Full scale Callibration Coefficient REgister 2
#define		ID	  0x0A //IDAC Control Register 0


/* BCS0 - BCS—Burn-out Current Source Register (offset = 00h) [reset = 01h] */
/* BIT7 - BIT6 -  BIT5   -  BIT4   -  BIT3   -  BIT2   -  BIT1   -  BIT0 */
/* BCS1 - BCS0 - 0 - 0 - 0 - 0 - 0 - 1 */
#define BCS_RESET 0x01      // Reset MUX0 Register 
/* BCS1:0 These bits select the magnitude of the sensor detect current source */
#define		BCS1_1		0b00000000    // 00 Burnout current source off (default)
#define		BCS1_2      0b01000000    // 01 Burnout current source on 0.5 �A
#define		BCS1_3      0b10000000    // 10 Burnout current source on 2 �A
#define		BCS1_4      0b11000000    // 11 Burnout current source on 10 �A


/* VBIAS - Bias Voltage Register (see p43 - bring together with bitwise OR | */
/*  BIT7  -  BIT6  -  BIT5  -  BIT4  -  BIT3  -  BIT2  -  BIT1  -  BIT0  */
/* VBIAS7 - VBIAS6 - VBIAS5 - VBIAS4 - VBIAS3 - VBIAS2 - VBIAS1 - VBIAS0 */

/* VBIAS These bits apply a bias voltage of midsupply (AVDD + AVSS)/2 to the selected analog input */
#define		VBIAS_RESET	0x00		 // Reset VBIAS Register 
#define		VBIAS_1		0b00000010    // AIN1
#define		VBIAS_0		0b00000001    // AIN0


/* MUX1 - Multiplexer Control Register 1 (see p44 - bring together with bitwise OR | */
/*  BIT7   -   BIT6   -   BIT5   -   BIT4   -   BIT3   -  BIT2   -  BIT1   -  BIT0 */
/* CLKSTAT - 0 - 0 - 0 - 0 - MUXCAL2 - MUXCAL1 - MUXCAL0 */
#define			MUX1_RESET		0x00      // Reset MUX1 Register 
/* CLKSTAT This bit is read-only and indicates whether the internal or external oscillator is being used
0 = internal, 1 = external */
/* MUXCAL2:0 These bits are used to select a system monitor. The MUXCAL selection supercedes selections from registers MUX0 and MUX1  */
#define		MUXCAL2_NORMAL		0b00000000    // Normal operation (default)
#define		MUXCAL2_OFFSET		0b00000001    // Offset measurement
#define		MUXCAL2_GAIN		0b00000010    // Gain measurement
#define		MUXCAL2_TEMP		0b00000011    // Temperature diode


/* SYS0 - System Control Register 0 (see p45 - bring together with bitwise OR | */
/* BIT7 - BIT6 - BIT5 - BIT4 - BIT3 - BIT2 - BIT1 - BIT0 */
/*  0   - PGA2 - PGA1 - PGA0 - DOR3 - DOR2 - DOR1 - DOR0 */
/* PGA2:0 These bits determine the gain of the PGA  */
#define		PGA2_0			0b00000000    // 1 (default)
#define		PGA2_2			0b00010000    // 2
#define		PGA2_4			0b00100000    // 4
#define		PGA2_8			0b00110000    // 8
#define		PGA2_16			0b01000000    // 16
#define		PGA2_32			0b01010000    // 32
#define		PGA2_64			0b01100000    // 64
#define		PGA2_128		0b01110000    // 128
/* DOR3:0 These bits select the output data rate of the ADC  */
#define		DOR3_5      0b00000000    // 5SPS (default)
#define		DOR3_10     0b00000001    // 10SPS
#define		DOR3_20     0b00000010    // 20SPS
#define		DOR3_40     0b00000011    // 40SPS
#define		DOR3_80     0b00000100    // 80SPS
#define		DOR3_160    0b00000101    // 160SPS
#define		DOR3_320    0b00000110    // 320SPS
#define		DOR3_640    0b00000111    // 640SPS
#define		DOR3_1000   0b00001000    // 1000SPS
#define		DOR3_2000   0b00001001    // 2000SPS




/* SPI COMMAND DEFINITIONS (p49) */
/*SYSTEM CONTROL */
#define		WAKEUP		0x0000 	//Exit Sleep Mode
#define 	SLEEP 		0x0001	//Enter Sleep Mode
#define 	SYNC 		0x0004	//Synchornize the A/D Conversion
#define 	RESET		0x0006	//Reset To Power UP values
#define 	NOP			0xFFFF	//NO Operation
/*DATA READ*/
#define		RDATA		0x0012	//Read data once
#define 	RDATAC		0x0014	//Read data continously
#define 	SDATAC 		0x0016	//Stop reading data continously
/*READ REGISTER */
#define 	RREG		0x0020	//Read From Register
#define 	WREG 		0x0040	//Write To Register
/*Calibration */
#define 	SYSOCAL		0x0060	//System Offset Calibration
#define 	SYSGCAL		0x0061	//System Gain Calibration
#define 	SELFOCAL	0x0062	//Self Offset Calibration


#define SPI_SPEED 4000000
#define DRDY_PIN 8
#define _PIN 
#define Reset_PIN 




#endif
