/*
 * PCF8563.c
 *
 *	The MIT License.
 *  Created on: 5.09.2019
 *      Author: Mateusz Salamon
 *		Contact: mateusz@msalamon.pl
 *
 *      Website: https://msalamon.pl/dalsze-zmagania-z-rtc-ds1307-i-pcf8563-na-stm32/
 *      GitHub: https://github.com/lamik/PCF8563_RTC_STM32_HAL
 */
#include "main.h"
#include "PCF8563.h"

I2C_HandleTypeDef *hi2c_pcf8563;
uint8_t Pcf8563Buffer[7];


void WriteBitToControlRegister(uint8_t ControlRegister, uint8_t BitNumber, uint8_t Value)
{
	uint8_t tmp;

	if(Value>1) Value = 1;
	if(ControlRegister>1) Value = 1;
	if(BitNumber>7) Value = 7;

	HAL_I2C_Mem_Read(hi2c_pcf8563, PCF8563_ADDRESS, ControlRegister?PCF8563_REG_CONTROL_STATUS1:PCF8563_REG_CONTROL_STATUS2, 1, &tmp, 1, PCF8563_I2C_TIMEOUT);
	tmp &= ~(1<<BitNumber);
	tmp |= (Value<<BitNumber);
	ControlRegister?(Value &= 0b10101000):(Value &= 0b00011111); // Put zeros where zero is needed
	HAL_I2C_Mem_Write(hi2c_pcf8563, PCF8563_ADDRESS, ControlRegister?PCF8563_REG_CONTROL_STATUS1:PCF8563_REG_CONTROL_STATUS2, 1, &Value, 1, PCF8563_I2C_TIMEOUT);
}

void PCF8563_TEST1Enable(uint8_t Enable)
{
	WriteBitToControlRegister(PCF8563_REG_CONTROL_STATUS1, PCF8563_CONTROL1_TEST1, Enable);
}

void PCF8563_STOPEnable(uint8_t Enable)
{
	WriteBitToControlRegister(PCF8563_REG_CONTROL_STATUS1, PCF8563_CONTROL1_STOP, Enable);
}

void PCF8563_TESTCEnable(uint8_t Enable)
{
	WriteBitToControlRegister(PCF8563_REG_CONTROL_STATUS1, PCF8563_CONTROL1_TESTC, Enable);
}

void PCF8563_InterruptEnable(uint8_t Enable)
{
	WriteBitToControlRegister(PCF8563_REG_CONTROL_STATUS1, PCF8563_CONTROL2_TI_TP, Enable);
}

void PCF8563_AlarmFlagEnable(uint8_t Enable)
{
	WriteBitToControlRegister(PCF8563_REG_CONTROL_STATUS1, PCF8563_CONTROL2_AF, Enable);
}

void PCF8563_TimerFlagEnable(uint8_t Enable)
{
	WriteBitToControlRegister(PCF8563_REG_CONTROL_STATUS1, PCF8563_CONTROL2_TF, Enable);
}

void PCF8563_AlarmInterruptEnable(uint8_t Enable)
{
	WriteBitToControlRegister(PCF8563_REG_CONTROL_STATUS1, PCF8563_CONTROL2_AIE, Enable);
}

void PCF8563_TimerInterruptEnable(uint8_t Enable)
{
	WriteBitToControlRegister(PCF8563_REG_CONTROL_STATUS1, PCF8563_CONTROL2_TIE, Enable);
}

void PCF8563_ClkoutFrequency(CLKOUT_Freq Frequency)
{
	uint8_t tmp;

	if(Frequency>3) Frequency = 3;

	HAL_I2C_Mem_Read(hi2c_pcf8563, PCF8563_ADDRESS, PCF8563_REG_CLKOUT, 1, &tmp, 1, PCF8563_I2C_TIMEOUT);
	tmp &= ~(3<<PCF8563_CLKOUT_CONTROL_FD0);
	tmp |= (Frequency<<PCF8563_CLKOUT_CONTROL_FD0);
	HAL_I2C_Mem_Write(hi2c_pcf8563, PCF8563_ADDRESS, PCF8563_REG_CLKOUT, 1, &tmp, 1, PCF8563_I2C_TIMEOUT);
}

uint8_t bcd2dec(uint8_t BCD)
{
	return (((BCD & 0xF0)>>4) *10) + (BCD & 0xF);
}

uint8_t dec2bcd(uint8_t DEC)
{
	return ((DEC / 10)<<4) + (DEC % 10);
}

int dayofweek(int Day, int Month, int Year)
{
    int Y, C, M, N, D;
    M = 1 + (9 + Month) % 12;
    Y = Year - (M > 10);
    C = Y / 100;
    D = Y % 100;
    N = ((13 * M - 1) / 5 + D + D / 4 + 6 * C + Day + 5) % 7;
    return (7 + N) % 7;
}

void PCF8563_SetDateTime(RTCDateTime *DateTime)
{
	uint8_t tmp[7];

	if(DateTime->Second > 59) DateTime->Second = 59;
	if(DateTime->Minute > 59) DateTime->Minute = 59;
	if(DateTime->Hour > 23) DateTime->Hour = 23;
	if(DateTime->Day > 31) DateTime->Day = 31;
	if(DateTime->Month > 12) DateTime->Month = 12;
	if(DateTime->Year> 2099) DateTime->Year = 2099;

	tmp[0] = dec2bcd((DateTime->Second) & 0x7F);
	tmp[1] = dec2bcd(DateTime->Minute);
	tmp[2] = dec2bcd(DateTime->Hour);
	tmp[3] = dec2bcd(DateTime->Day);
	tmp[4] = dayofweek(DateTime->Day, DateTime->Month, DateTime->Year);
	tmp[5] = dec2bcd(DateTime->Month);
	tmp[6] = dec2bcd(DateTime->Year - 2000);

	HAL_I2C_Mem_Write(hi2c_pcf8563, PCF8563_ADDRESS, PCF8563_REG_TIME, 1, tmp, 7, PCF8563_I2C_TIMEOUT);
}

void PCF8563_CalculateDateTime(RTCDateTime *DateTime)
{
	DateTime->Second = bcd2dec((Pcf8563Buffer[0]) & 0x7F);
	DateTime->Minute = bcd2dec(Pcf8563Buffer[1]);
	DateTime->Hour = bcd2dec(Pcf8563Buffer[2]);
	DateTime->Day = Pcf8563Buffer[3];
	DateTime->DayOfWeek = bcd2dec(Pcf8563Buffer[4] + 1); // too keep weekdays in 1-7 format
	DateTime->Month = bcd2dec(Pcf8563Buffer[5] & 0x1F);
	DateTime->Year = 2000 + bcd2dec(Pcf8563Buffer[6]);
}

#ifdef PCF8563_USE_DMA
void PCF8563_ReceiveDateTimeDMA(void)
{
	HAL_I2C_Mem_Read_DMA(hi2c_pcf8563, PCF8563_ADDRESS, PCF8563_REG_TIME, 1, Pcf8563Buffer, 7);
}
#else
void PCF8563_GetDateTime(RTCDateTime *DateTime)
{
	HAL_I2C_Mem_Read(hi2c_pcf8563, PCF8563_ADDRESS, PCF8563_REG_TIME, 1, Pcf8563Buffer, 7, PCF8563_I2C_TIMEOUT);

	void PCF8563_CalculateDateTime(DateTime);
}
#endif

void PCF8563_Init(I2C_HandleTypeDef *hi2c)
{
	hi2c_pcf8563 = hi2c;

	PCF8563_ClkoutFrequency(CLKOUT_FREQ_1HZ);
	PCF8563_STOPEnable(0);
}
