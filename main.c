#define ENABLE_LOG

#define ACC_PIN					P05
#define RED_LED 				P11
#define GREEN_LED 				P15
#define BUTTON_PIN				P30
#define REAR_PIN				P12
#define PLUG_PIN				P10
#define CHARGE_PIN				P17

#define CLR_BUTTON_PIN 			CLR_BIT0
#define CLR_ACC_PIN 			CLR_BIT5
#define CLR_CHG_PIN				CLR_BIT7
#define CLR_BATTERY_PIN 		CLR_BIT0 // 찾아낼 것

#define GYRO_ADDR				0x32
#define I2C_CLOCK				32
#define I2C_WR					0
#define I2C_RD					1

#define OUT_X					0x28
#define OUT_Y					0x2A
#define OUT_Z					0x2C

#define PERIOD_MIN              100
#define PERIOD_MULT             10
#define PERIOD_UNIT             (PERIOD_MULT * PERIOD_MIN)
#define LONG_PRESS              (4000 / PERIOD_MULT)


#define REARLIGHT_MODE_SIZE     5


typedef enum {
	REAR_MIN_STEADY = 0,
	REAR_MAX_STEADY = 1,
	REAR_MIN_BREAK = 2,
	REAR_OFF_BREAK = 3,
	REAR_OFF = 4
} REARLIGHT_MODE;

uint8_t low_bright[REARLIGHT_MODE_SIZE]  = { 0x80, 0xFF, 0x80, 0x00, 0x00 };
uint8_t high_bright[REARLIGHT_MODE_SIZE] = { 0x80, 0xFF, 0x80, 0x00, 0x00 };

REARLIGHT_MODE current_rear = REAR_OFF;
REARLIGHT_MODE prev_rear = REAR_MIN_STEADY;

uint16_t current_battery = 0;

#ifdef ENABLE_LOG
#	define LS_LOG(c) uart_log(c)
#	define LS_LOGN(n) uart_logn(n)
#else
#	define LS_LOG(c)
#	define LS_LOGN(c)
#endif


/*********************
 * PROGRAM CODE
 *********************/


/* Timer (퍼온 코드) */
void Timer0_Delay(unsigned long u32SYSCLK, unsigned int u16CNT, unsigned int u16DLYUnit)
{
      unsigned char TL0TMP, TH0TMP;

      TIMER0_FSYS_DIV12;                                  //T0M=0, Timer0 Clock = Fsys/12
      ENABLE_TIMER0_MODE1;                                   //Timer0 is 16-bit mode
      TL0TMP = LOBYTE(65535-((u32SYSCLK/1000000)*u16DLYUnit/12));
      TH0TMP = HIBYTE(65535-((u32SYSCLK/1000000)*u16DLYUnit/12));

    while (u16CNT != 0)
    {
      TL0=TL0TMP;
      TH0=TH0TMP;
      set_TCON_TR0;                                    //Start Timer0
      while (!TF0);                       //Check Timer0 Time-Out Flag
      clr_TCON_TF0;
      clr_TCON_TR0;                       //Stop Timer0
      u16CNT --;
    }
}


/* Log */
#ifdef ENABLE_LOG
void uart_log(char c)
{
	UART_Send_Data(UART0,c);
	Timer0_Delay(24000000, 10, 10);

}

void uart_logn(uint32_t n)
{
	if( n == 0 ) {
		uart_log('0');
		return;
	}

	uint32_t e = 10;

	while( e <= n ) e *= 10;

	while( e > 1 ) {
		e /= 10;
		uint32_t res = n / e;
		uart_log(res + '0');
		n = n % e;
	}
}

void log_init(void)
{
    P06_QUASI_MODE;
	UART_Open(24000000,UART0_Timer1,9600);
}
#endif



void peripheral_init(void)
{
    /* Prevent Restart */
    set_SFRS_SFRPAGE; // 이후에 테스트 해볼 것
    P2S|=0x81;
    clr_SFRS_SFRPAGE;


    /* Backlight Init */
	clr_CKCON_PWMCKS; // PWM in FSYS freq.
	PWM0_CLOCK_DIV_1; // PWM div = 1

	ENABLE_PWM0_CH0_P12_OUTPUT;

	P12_PUSHPULL_MODE;

    clr_PWMCON0_PWMRUN;
	set_SFRS_SFRPAGE;

	PWMPH = MAX_PWM; // 255 bit PWM Period High Byte
	PWMPL = MAX_PWM; // 255 bit PWM Period Low Byte

	PWM0H = low_bright[current_rear];  // PWM Channel 0~5 Duty High Byte n = 0,1,2,3,4,5
	PWM0L = high_bright[current_rear]; // PWM Channel 0~5 Duty Low  Byte n = 0,1,2,3,4,5

	clr_SFRS_SFRPAGE;
	set_PWMCON0_PWMRUN;


    /* Button Init */
	P30_INPUT_MODE; 		// button init - pin30
	ENABLE_INT0_INTERRUPT;
    INT0_FALLING_EDGE_TRIG;

    /* Charge Init */

    /* Battery Init */

    /* Board Led Init */
    P11_QUASI_MODE;
	P15_QUASI_MODE;

    /* I2C(Acc) Init */
    P13_OPENDRAIN_MODE; // SCL
    P14_OPENDRAIN_MODE; // SDA
    P05_INPUT_MODE;     // I2C Interrupt Pin

    I2CLK = I2C_CLOCK;
    set_I2CON_I2CEN;

    PICON |= 0x40;
    PINEN |= 0x20;
    PIPEN |= 0x20;

    i2c_send_data(0x20, 0x77); // CTRL_REG1 -> 400Hz
    i2c_send_data(0x21, 0x00); // CTRL_REG2 -> High pass filter
    i2c_send_data(0x22, 0x40); // CTRL_REG3 -> Interrupt 1 setting 0x40, 0x50
    i2c_send_data(0x23, 0x00); // CTRL_REG4 -> BDU(continouse update)
    i2c_send_data(0x24, 0x00); // CTRL_REG5 -> Latch Interrupt
    i2c_send_data(0x25, 0x00); // CTRL_REG6
    i2c_send_data(0x30, 0x08); // INT1_CFG
    i2c_send_data(0x32, 0x10); // INT1_THS -> Interrupt 1 threshold
    i2c_send_data(0x33, 0x02); // INT1_DURATION




    ENABLE_GLOBAL_INTERRUPT;
    ENABLE_PIN_INTERRUPT;
}



void main(void) 
{
    MODIFY_HIRC(HIRC_24);
    peripheral_init();

#ifdef ENABLE_LOG
    log_init();
#endif

    while(1)
    {

    }
}