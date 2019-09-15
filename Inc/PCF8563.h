/*
 * PCF8563.h
 *
 *	The MIT License.
 *  Created on: 5.09.2019
 *      Author: Mateusz Salamon
 *		Contact: mateusz@msalamon.pl
 *
 *      Website: https://msalamon.pl/dalsze-zmagania-z-rtc-ds1307-i-pcf8563-na-stm32/
 *      GitHub: https://github.com/lamik/PCF8563_RTC_STM32_HAL
 */

//
//	Uncomment when you are using DMA reading
//
#define PCF8563_USE_DMA

#define PCF8563_ADDRESS              	(0x51<<1)
#define PCF8563_I2C_TIMEOUT				100

#define PCF8563_REG_CONTROL_STATUS1     0x00
#define PCF8563_REG_CONTROL_STATUS2     0x01
#define PCF8563_REG_TIME         		0x02
#define PCF8563_REG_SECONDS          	0x02
#define PCF8563_REG_MINUTES         	0x03
#define PCF8563_REG_HOURS          		0x04
#define PCF8563_REG_DATE             	0x05
#define PCF8563_REG_WEEKDAY             0x06
#define PCF8563_REG_MONTH            	0x07
#define PCF8563_REG_YEAR             	0x08
#define PCF8563_REG_ALARM_MINUTE        0x09
#define PCF8563_REG_ALARM_HOUR       	0x0A
#define PCF8563_REG_ALARM_DAY   		0x0B
#define PCF8563_REG_ALARM_WEEKDAY    	0x0C
#define PCF8563_REG_CLKOUT    			0x0D
#define PCF8563_REG_TIMER_CONTROL    	0x0E
#define PCF8563_REG_TIMER     			0x0F

//
//	Controll register 1 0x00
//
#define PCF8563_CONTROL1_TEST1		7
#define PCF8563_CONTROL1_STOP		5
#define PCF8563_CONTROL1_TESTC		3

//
//	Controll register 2 0x01
//
#define PCF8563_CONTROL2_TI_TP		4
#define PCF8563_CONTROL2_AF			3
#define PCF8563_CONTROL2_TF			2
#define PCF8563_CONTROL2_AIE		1
#define PCF8563_CONTROL2_TIE		0

//
//	CLKOUT control register 0x0D
//
#define PCF8563_CLKOUT_CONTROL_FE				7
#define PCF8563_CLKOUT_CONTROL_FD1				1
#define PCF8563_CLKOUT_CONTROL_FD0				0

typedef enum
{
	CLKOUT_FREQ_1HZ 		= 3,
	CLKOUT_FREQ_32HZ 		= 2,
	CLKOUT_FREQ_1024HZ 		= 1,
	CLKOUT_FREQ_32768HZ 	= 0
}CLKOUT_Freq;

typedef struct
{
	uint16_t 	Year;
	uint8_t  	Month;
	uint8_t		Day;
	uint8_t		Hour;
	uint8_t		Minute;
	uint8_t		Second;
	uint8_t		DayOfWeek;
}RTCDateTime;

void PCF8563_TEST1Enable(uint8_t Enable);
void PCF8563_STOPEnable(uint8_t Enable);
void PCF8563_TESTCEnable(uint8_t Enable);
void PCF8563_InterruptEnable(uint8_t Enable);
void PCF8563_AlarmFlagEnable(uint8_t Enable);
void PCF8563_TimerFlagEnable(uint8_t Enable);
void PCF8563_AlarmInterruptEnable(uint8_t Enable);
void PCF8563_TimerInterruptEnable(uint8_t Enable);

void PCF8563_ClkoutFrequency(CLKOUT_Freq Frequency);

#ifdef PCF8563_USE_DMA
void PCF8563_ReceiveDateTimeDMA(void);	// Use in PCF8563 Interrupt handler
void PCF8563_CalculateDateTime(RTCDateTime *DateTime);	// Use in DMA Complete Receive interrupt
#else
void PCF8563_GetDateTime(RTCDateTime *DateTime);	// Use in blocking/interrupt mode in PCF8563_INT EXTI handler
#endif

void PCF8563_Init(I2C_HandleTypeDef *hi2c);
