/*
 * Connections
 * ~~~~~~~~~~~
 * SCK	->	PB10
 * SDA	->	PB11
 * +5V	->	EXT_5V
 * GND	->	GND
 */



/* Includes */
#include <stddef.h>
#include "stm32l1xx.h"
#include "stm32l1xx_i2c.h"

#include "discover_board.h"
#include "stm32l_discovery_lcd.h"

/* 7bit add. of slave is 1010.xxx ... does STM library left shift? should it be x50?*/
#define I2C2_SLAVE_ADDRESS7 0xA0
#define ClockSpeed 100000 /*tried multiple speeds, no difference */

void RCC_Configuration(void);
void Init_GPIOs(void);

static volatile uint32_t TimingDelay;
RCC_ClocksTypeDef RCC_Clocks;

void Config_Systick()
{
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
}


#define FOSC 8000000
#define BAUD 9600
#define BMP085_R 0xEF
#define BMP085_W 0xEE
#define OSS 0    // Oversampling Setting (note: code is not set up to use other OSS values)

#define sbi(var, mask)   ((var) |= (unsigned short_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (unsigned short_t)~(1 << mask))

///============Function Prototypes=========/////////////////
void BMP085_Calibration(void);

///============I2C Prototypes=============//////////////////
short bmp085ReadShort(unsigned char address);
long bmp085ReadTemp(void);
long bmp085ReadPressure(void);
void bmp085Convert(long * temperature, long * pressure);

///============Initialize Prototypes=====//////////////////
void ioinit(void);
/*void UART_Init(unsigned int ubrr);
static int uart_putchar(char c, FILE *stream);
void put_char(unsigned char byte);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
*/
void Delay(unsigned int nTime);

/////=========Global Variables======////////////////////
short ac1;
short ac2;
short ac3;
unsigned short ac4;
unsigned short ac5;
unsigned short ac6;
short b1;
short b2;
short mb;
short mc;
short md;

int main(void)
{
    long temperature = 0;
    long pressure = 0;
    long altitude = 0;
    long hPa = 0;
    double temp = 0;
    char LCDmessage[10];
    int i;

    /* Init Systick */
    Config_Systick();
    RCC_Configuration();

    /* Init I/O ports */
    Init_GPIOs();

    Delay(100);

    /* Switch on the leds at start */
    GPIO_HIGH(LD_PORT,LD_GREEN);
    GPIO_HIGH(LD_PORT,LD_BLUE);

    /* Initializes the LCD glass */
    LCD_GLASS_Init();
//    LCD_GLASS_ScrollSentence((uint8_t*)"      ** STM32L-DISCOVERY **", 1, 200);
    LCD_GLASS_Clear();
//    Delay(1000);

    LCD_GLASS_DisplayString((uint8_t*)"BMP085 ");
    Delay(1000);

    /* Set-up I/O ports for I2C*/
    i2cInit();
    BMP085_Calibration();

    while(1)
    {
    	for (i = 0; i <= 1; i++)
    	{
			/* Set-up I/O ports for I2C*/
			i2cInit();
			bmp085Convert(&temperature, &pressure);

			printf("Temperature: %ld (in 0.1 deg C)\n", temperature);
			printf("Pressure: %ld Pa\n\n", pressure);

			// Convert to altitude
			hPa =  pressure / 100;
			temp = (double) pressure/101325;
			temp = 1-pow(temp, 0.19029);
			altitude = round(44330*temp*3.28);
			printf("Altitude: %ldm\n\n", altitude);

			/* Set-up I/O ports for LCD*/
			Init_GPIOs();

			if (i)
				sprintf(LCDmessage, "  %dmB", hPa);
			else
				sprintf(LCDmessage, "  %dC", temperature);

			LCD_GLASS_Clear();
			LCD_GLASS_DisplayString((uint8_t*)LCDmessage);

			GPIO_TOGGLE(LD_PORT,LD_BLUE);
			Delay(1000);
    	}
    }
}

void BMP085_Calibration(void)
{
	// Read Calibration Information

    ac1 = bmp085ReadShort(0xAA);
    ac2 = bmp085ReadShort(0xAC);
    ac3 = bmp085ReadShort(0xAE);
    ac4 = bmp085ReadShort(0xB0);
    ac5 = bmp085ReadShort(0xB2);
    ac6 = bmp085ReadShort(0xB4);
    b1 = bmp085ReadShort(0xB6);
    b2 = bmp085ReadShort(0xB8);
    mb = bmp085ReadShort(0xBA);
    mc = bmp085ReadShort(0xBC);
    md = bmp085ReadShort(0xBE);

}

// bmp085ReadShort will read two sequential 8-bit registers, and return a 16-bit value
// the MSB register is read first
// Input: First register to read
// Output: 16-bit value of (first register value << 8) | (sequential register value)
short bmp085ReadShort(unsigned char address)
{
    char msb, lsb, dummy;
    short data;

	I2C_GenerateSTART(I2C2, ENABLE);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C2, BMP085_W,I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C2, address);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTART(I2C2, ENABLE);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C2, BMP085_R,I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	// Wait for data
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
    msb = I2C_ReceiveData(I2C2);

	/* Prepare an NACK for the next data received */
    I2C_AcknowledgeConfig(I2C2, DISABLE);

    // Wait for data
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
    lsb = I2C_ReceiveData(I2C2);

	I2C_GenerateSTOP(I2C2, ENABLE);

	Delay(10);    // max time is 4.5ms

    /* Prepare ACK for next I2C operation*/
    I2C_AcknowledgeConfig(I2C2, ENABLE);
	

    data = msb << 8 | lsb;

    return data;
}

void bmp085Convert(long* temperature, long* pressure)
{
    long ut;
    long up;
    long x1, x2, b5, b6, x3, b3, p;
    unsigned long b4, b7;

    ut = bmp085ReadTemp();

    up = bmp085ReadPressure();

    x1 = ((long)ut - ac6) * ac5 >> 15;
    x2 = ((long) mc << 11) / (x1 + md);
    b5 = x1 + x2;
    *temperature = (b5 + 8) >> 4;

    b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6 >> 12)) >> 11;
    x2 = ac2 * b6 >> 11;
    x3 = x1 + x2;
//    b3 = (((long) ac1 * 4 + x3) <<OSS + 2)/4;
    b3 = (((long) ac1 * 4 + x3) + 2)/4;
    x1 = ac3 * b6 >> 13;
    x2 = (b1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (unsigned long) (x3 + 32768)) >> 15;
//    b7 = ((unsigned long) up - b3) * (50000 >> OSS);
    b7 = ((unsigned long) up - b3) * (50000);
    p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    *pressure = p + ((x1 + x2 + 3791) >> 4);
}

long bmp085ReadTemp(void)
{

	I2C_GenerateSTART(I2C2, ENABLE);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C2, BMP085_W,I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C2, 0xF4);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_SendData(I2C2, 0x2E);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTOP(I2C2, ENABLE);

    Delay(10);    // max time is 4.5ms

    return (long) bmp085ReadShort(0xF6);
}

long bmp085ReadPressure(void)
{
    long pressure = 0;

	I2C_GenerateSTART(I2C2, ENABLE);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C2, BMP085_W,I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C2, 0xF4);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_SendData(I2C2, 0x34);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTOP(I2C2, ENABLE);

    Delay(10);    // max time is 4.5ms

    pressure = bmp085ReadShort(0xF6);
    pressure &= 0x0000FFFF;

    return pressure;

}


/*********************
 ****Initialize****
 *********************/

void  Init_GPIOs(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef I2C_InitStructure;

  /* Configure User Button pin as input */
  GPIO_InitStructure.GPIO_Pin = USER_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(BUTTON_GPIO_PORT, &GPIO_InitStructure);


/* Configure the GPIO pins  PB10 & PB11 for i2c*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; /* <--- is this needed? */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz; /*tried, 2, 10 and 40Mhz*/
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10,GPIO_AF_I2C2) ;
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11,GPIO_AF_I2C2) ;

  /* I2C2 configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = I2C2_SLAVE_ADDRESS7;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = ClockSpeed ;
  I2C_Init(I2C2, &I2C_InitStructure);
  I2C_Cmd(I2C2, ENABLE);

  /* Configure the GPIO_LED pins  LD3 & LD4*/
  GPIO_InitStructure.GPIO_Pin = LD_GREEN|LD_BLUE;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(LD_PORT, &GPIO_InitStructure);

  GPIO_LOW(LD_PORT,LD_GREEN);
  GPIO_LOW(LD_PORT,LD_BLUE);

/* Configure Output for LCD */
/* Port A */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_8 | GPIO_Pin_9 |GPIO_Pin_10 |GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init( GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource15,GPIO_AF_LCD) ;

/* Configure Output for LCD */
/* Port B */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_9 \
                                 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init( GPIOB, &GPIO_InitStructure);

/*Im not using all the LCD segments */
#ifdef USE_ALL_LCD_SEGMENTS
/**
 * Note!
 * PB3 is connected to C, M, COLON, and DP segments for the second digit on the LCD
 * PB3 is also the SWO pin used for the Serial Wire Viewer (SWV)
 * If PB3 is used by LCD then SWV will not work for the STM32L_DISCOVERY board
 **/
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource3,GPIO_AF_LCD) ;
#endif
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource12,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15,GPIO_AF_LCD) ;

/* Configure Output for LCD */
/* Port C*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 \
                                 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |GPIO_Pin_11 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init( GPIOC, &GPIO_InitStructure);


  GPIO_PinAFConfig(GPIOC, GPIO_PinSource0,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource1,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource2,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource3,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11,GPIO_AF_LCD) ;
}

void RCC_Configuration(void)
{
  /* Enable the GPIOs Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC| RCC_AHBPeriph_GPIOD| RCC_AHBPeriph_GPIOE| RCC_AHBPeriph_GPIOH, ENABLE);

  /* Enable comparator clock */
 /* I enable I2C1 and I2C2 clocks, only using I2C2, makes no difference */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_COMP | RCC_APB1Periph_I2C1 | RCC_APB1Periph_I2C2 | RCC_APB1Periph_LCD | RCC_APB1Periph_PWR,ENABLE);

  /* Enable SYSCFG */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG , ENABLE);

  /* Allow access to the RTC */
  PWR_RTCAccessCmd(ENABLE);

  /* Reset Backup Domain */
  RCC_RTCResetCmd(ENABLE);
  RCC_RTCResetCmd(DISABLE);

  /*!< LSE Enable */
  RCC_LSEConfig(RCC_LSE_ON);

  /*!< Wait till LSE is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {}

  /*!< LCD Clock Source Selection */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
}


/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 1 ms.
  * @retval None
  */
void Delay(unsigned int nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);

}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{

  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}

void i2cInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef I2C_InitStructure;

   /* Enable I2C clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    /* Enable GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  /* Configure PB.10 & PB.11 in alternate function -------------------------*/
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10,GPIO_AF_I2C2) ;
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11,GPIO_AF_I2C2) ;

  /* Enable I2C2 -------------------------------------------------------------*/
  I2C_DeInit(I2C2);
  I2C_Cmd(I2C2, ENABLE);

     /* Init I2C */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0xA0;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_ClockSpeed = IS_I2C_CLOCK_SPEED(400000);
  I2C_Init(I2C2,&I2C_InitStructure);

 }

/*
//From Bosch Sensortec BMP085_SMD500_API
int ac1, ac2, ac3, b1, b2, mb, mc, md;
unsigned int ac4, ac5, ac6;

void BMP085Calibrate()
{
    ac1 = 408;
    ac2 = -72;
    ac3 = -14383;
    ac4 = 32741;
    ac5 = 32757;
    ac6 = 23153;
    b1 = 6190;
    b2 = 4;
    mb = -32767;
    mc = -8711;
    md = 2868;
}

void BMP085(long up, long ut)
{
    long  tval, pval;
    long  x1, x2, x3, b3, b5, b6, p;
    unsigned long  b4, b7;
    unsigned short oss = 3;

    x1 = (ut - ac6) * ac5 >> 15;
    x2 = ((long ) mc << 11) / (x1 + md);
    b5 = x1 + x2;
    tval = (b5 + 8) >> 4;

    b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6 >> 12)) >> 11;
    x2 = ac2 * b6 >> 11;
    x3 = x1 + x2;
    b3 = (((long ) ac1 * 4 + x3)<<oss + 2) >> 2;
    x1 = ac3 * b6 >> 13;
    x2 = (b1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (unsigned long ) (x3 + 32768)) >> 15;
    b7 = ((unsigned long ) up - b3) * (50000 >> oss);
    p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    pval = p + ((x1 + x2 + 3791) >> 4);
}


void SMD500Calibrate()
{
    ac1 = 408;
    ac2 = -72;
    ac3 = -14383;
    ac4 = 32741;
    ac5 = 32757;
    ac6 = 23153;
    b1 = 6190;
    b2 = 4;
    mb = -32767;
    mc = -8711;
    md = 2868;
}
/*
void SMD500(long up, long ut)
{
    long  tval, pval;
    long  x1, x2, x3, b3, b5, b6, p;
    unsigned long  b4, b7;
    unsigned short oss = 3;

    x1 = (ut - ac6) * ac5 >> 15;
    x2 = ((long ) mc << 11) / (x1 + md);
    b5 = x1 + x2;
    tval = (b5 + 8) >> 4;

    b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6 >> 12)) >> 11;
    x2 = ac2 * b6 >> 11;
    x3 = x1 + x2;
    b3 = (((long ) ac1 * 4 + x3)<<oss + 2) >> 2;
    x1 = ac3 * b6 >> 13;
    x2 = (b1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (unsigned long ) (x3 + 32768)) >> 15;
    b7 = ((unsigned long ) up - b3) * (50000 >> oss);
    p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    pval = p + ((x1 + x2 + 3791) >> 4);
}
*/

//and from Sparkefun
/*
    BMP085 Test Code
    April 7, 2010
    by: Jim Lindblom

    Test code for the BMP085 Barometric Pressure Sensor.
    We'll first read all the calibration values from the sensor.
    Then the pressure and temperature readings will be read and calculated.
    Also attempts to calculate altitude (remove comments)
    The sensor is run in ultra low power mode.
    Tested on a 3.3V 8MHz Arduino Pro
    A4 (PC4) -> SDA
    A5 (PC5) -> SCL
    No Connection to EOC or XCLR pins
*/

