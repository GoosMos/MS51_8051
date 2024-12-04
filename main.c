#include "ms51_16k_sdcc.h"
#include "stdbool.h"

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
#define MAX_PWM 				255


#define REARLIGHT_MODE_SIZE     5

#define BREAK_LOW				0xFF;
#define BREAK_HIGH				0xFF;


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


uint32_t button_pressed = 0;
uint32_t button_unpressed = 0;
uint16_t blink_counter = 0;
uint8_t battery_level = 0;
bool acc_inter = 0;
bool break_flag = 0;
bool timer2_flag = 0;

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


void Btn_Interrupt_ISR(void) __interrupt (0)
{
	PUSH_SFRS;
	if ( !button_pressed ) {
		button_pressed = 1;
	}
	CLEAR_INT0_INTERRUPT_FLAG;
	POP_SFRS;
}


void Timer2_Interrupt_ISR(void) __interrupt (5)
{
    SFRS_TMP = SFRS;              /* for SFRS page */
    clr_T2CON_TF2;

    if (!timer2_flag) timer2_flag = 1;

    if (SFRS_TMP)                 /* for SFRS page */
    {
    	ENABLE_SFR_PAGE1;
    }
}


void Pin_Interrupt_ISR(void) __interrupt (7)
{
	PUSH_SFRS;
	if ( !acc_inter ) {
		acc_inter = 1;
	}
	PIF &= CLR_ACC_PIN;
	POP_SFRS;
}


/* ====================
 *         LOG
   ====================*/
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


/* ====================
 *        I2C
   ====================*/

#define I2C_LOG
uint8_t check_count = 0;
uint8_t result[20] = { 0x00, };


void i2c_error(uint8_t t)
{
	switch(t) {
	case 0x00:
		LS_LOG('B'); break;
	case 0x08:
		LS_LOG('I'); break;
	case 0x10:
		LS_LOG('M'); break;
	case 0x18:
		LS_LOG('R'); break;
	case 0x20:
		LS_LOG('N'); break;
	case 0x28:
		LS_LOG('E'); break;
	case 0x30:
		LS_LOG('0'); break;
	case 0x38:
		LS_LOG('L'); break;
	case 0x40:
		LS_LOG('K'); break;
	case 0x48:
		LS_LOG('H'); break;
	case 0x50:
		LS_LOG('Y'); break;
	case 0x58:
		LS_LOG('X'); break;
	default:
		LS_LOG((char)I2STAT);
	}
}


void i2c_send_data(uint8_t reg_addr, uint8_t reg_val)
{
	check_count = 0;

	// Send Start Bit
	set_I2CON_STA;
	clr_I2CON_SI;
	while(!SI);
	clr_I2CON_STA;
	result[check_count++] = I2STAT;

	// Send Sensor Memory Address
	I2DAT = (GYRO_ADDR | I2C_WR);
	clr_I2CON_SI;
	while(!SI);
	result[check_count++] = I2STAT;

	// Send Reg Address
	I2DAT = reg_addr;
	clr_I2CON_SI;
	while(!SI);
	result[check_count++] = I2STAT;

	// Send Reg Value
	I2DAT = reg_val;
	clr_I2CON_SI;
	while(!SI);
	result[check_count++] = I2STAT;

    // Send Stop Bit
    set_I2CON_STO;
    clr_I2CON_SI;
    while (STO);

	for (int i = 0;i < check_count; i++) {
		i2c_error(result[i]);
	}
}

uint16_t i2c_read_data(uint8_t reg_addr)
{
	check_count = 0;
	uint16_t axis_data = 0;

	set_I2CON_STA; // Send Start Bit
	clr_I2CON_SI;
	while(!SI);
	result[check_count++] = I2STAT; // I
	clr_I2CON_STA;

	I2DAT = (GYRO_ADDR | I2C_WR); // Send Gyro Address + Write Bit
	clr_I2CON_SI;
	while(!SI);
	result[check_count++] = I2STAT; // R

	I2DAT = (reg_addr | 0x80); // Who Am I Reg Address
	clr_I2CON_SI;
	while(!SI);
	result[check_count++] = I2STAT; // E

    set_I2CON_STA; // Repeated Start Bit
    clr_I2CON_SI;
    while (!SI);
    result[check_count++] = I2STAT; // M
    clr_I2CON_STA;

    I2DAT = (GYRO_ADDR | I2C_RD); // Send Read Bit
    clr_I2CON_SI;
    while (!SI);
    result[check_count++] = I2STAT; // K

    // Read Data
    set_I2CON_AA;
    clr_I2CON_SI;
    while(!SI);
    axis_data = I2DAT;
    result[check_count++] = I2STAT;

    // Read Next Data
    set_I2CON_AA;
    clr_I2CON_SI;
    while(!SI);
    axis_data = (I2DAT << 8);
    result[check_count++] = I2STAT;

    clr_I2CON_AA; // Send NMAK to Slave
    clr_I2CON_SI;
    while(!SI);
    result[check_count++] = I2STAT; // X

    // Stop
    set_I2CON_STO;
    clr_I2CON_SI;
    while(STO);

//    for (int i = 0;i < check_count; i++) {
//		i2c_error(result[i]);
//	}

    return axis_data;
}



/* ====================
 *      Backlight
   ====================*/

void backlight_mode_change(void)
{
	switch (current_rear) {
	case REAR_MIN_STEADY:
		current_rear++;
		break;
	case REAR_MAX_STEADY:
		current_rear++;
		break;
	case REAR_MIN_BREAK:
		current_rear++;
		break;
	case REAR_OFF_BREAK:
		current_rear = 0;
		break;
	case REAR_OFF:
		current_rear = 0;
		break;
	}
}


bool break_action = 0;
void backlight_routine(uint8_t low_bright, uint8_t high_bright)
{
	clr_PWMCON0_PWMRUN;
	set_SFRS_SFRPAGE;

	PWMPH = MAX_PWM;	 // PWM Period High Byte
	PWMPL = MAX_PWM; 	 // PWM Period Low  Byte, 255 bit PWM
	PWM0L = low_bright;  // 0 channel low control
	PWM0H = high_bright; // 0 channel high control

	clr_SFRS_SFRPAGE;
	set_PWMCON0_PWMRUN;
}



/* ====================
 *       Charge
   ====================*/

bool is_plugged(void) {
	return (PLUG_PIN == 0);
}

bool is_charging(void) {
	return (CHARGE_PIN == 0);
}



/* ====================
 *       Battery
   ====================*/

void low_pass_filter(uint16_t input)
{
	uint16_t sampling_data = (input >> 3);
	uint16_t prev_sampling_data = current_battery - (current_battery >> 3);

	current_battery = (uint16_t)(sampling_data + prev_sampling_data);
}

void update_battery(void) {
	uint16_t ADCdataAIN;
    ENABLE_ADC;
    clr_ADCCON0_ADCF;
    set_ADCCON0_ADCS;
    while(!(ADCCON0&SET_BIT7));        // Wait ADC flag
    ADCdataAIN = ADCRH<<4;
    ADCdataAIN |= ADCRL&0x0F;
    DISABLE_ADC;

    low_pass_filter(ADCdataAIN);

//    LS_LOGN(ADCdataAIN);
//    LS_LOGN(current_battery);
}



/* ====================
 *       Process
   ====================*/


void process_button(void)
{
	if( BUTTON_PIN == 0 ) {
		button_pressed++;
		button_unpressed = 0;

		if( button_pressed == LONG_PRESS ) { // 길게 눌려졌다면
			LS_LOG('L');
			if( current_rear == REAR_OFF ) { // 전원 버튼의 역할을 한다.
				current_rear = prev_rear;     // 길게 눌러서 전원을 키는 동작 가장 최근 사용된 모드로 시작
//				i2c_active(); // REAR_MIN_BREAK, REAR_OFF_BREAK인 경우 100Hz로 구동
				if ( current_rear & 0x02 ) i2c_send_data(0x20, 0x57); // CTRL_REG1 -> 100Hz
			} else {
				prev_rear = current_rear; // 마지막 모드를 저장
				current_rear = REAR_OFF; // 길게 눌러서 전원을 끄는동작
//				i2c_inactive(); // REAR_OFF로 들어가면 power down
				i2c_send_data(0x20, 0x07);
			}
		}
	} else if( button_pressed ) {
		LS_LOG('U');
		button_unpressed ++;
		if( button_unpressed > 1 ) { // 두번이상 안눌려졌다고 판단했는데
			if( button_pressed < LONG_PRESS ) { // 짧게 눌렀던 상태라면 다음상태로
				LS_LOG('S');
				if (current_rear != REAR_OFF) {
					backlight_mode_change();
//					i2c_inactive(); // REAR_MIN_BREAK, REAR_OFF_BREAK가 아닌 경우 power down
//					i2c_active();
					if ( current_rear & 0x02 ) i2c_send_data(0x20, 0x57);
					else i2c_send_data(0x20, 0x07);
				}
			}
			button_pressed = button_unpressed = 0;
		}
	}
}

void onboard_routine(void)
{
	if( is_plugged() ) { // 충전 케이블이 연결되어있는 상태
		battery_level = 0;
		blink_counter = 0;

		if( is_charging() ) { // 배터리 충전 진행 중
			GREEN_LED = 1; // Green Led Off

			if ( blink_counter < 20 ) {
				RED_LED = 0;
			} else if ( blink_counter < 400 ) {
				RED_LED = 1;
			} else {
				blink_counter = 0;
			}
			blink_counter++;
			return;
		} else { // 배터리 완충 상태
			GREEN_LED = 0;
			RED_LED = 1;
			return;
		}
	} else { // 충전선이 연결되어있지 않은 상태
		GREEN_LED = 1;

		if( current_rear == REAR_OFF ) { // 현재 sleep 상태에서 충전선이 연결되어있지 않는 상태
			RED_LED = 1; // RED LED 끄기
			return;
		}
		if( battery_level == 0 && current_battery > 3250 ) { //3250 3300, 배터리 잔량이 70 ~ 100% 사이인 경우
			RED_LED = 1; // RED LED 끄기
			return;
		}
		if( battery_level <= 1 && current_battery > 3000 ) {
			battery_level = 1;
			blink_counter ++;
			if(blink_counter < 50) {
				RED_LED = 0;
			} else if( blink_counter < 1000 ) {
				RED_LED = 1;
			} else {
				blink_counter = 0;
			}
			return;
		}
		if ( battery_level <= 2 && current_battery > 2800 ) { // 10% 구간으로 수정 필요
			battery_level = 2;
			blink_counter++;
			if (blink_counter < 10) {
				RED_LED = 0;
			} else if (blink_counter < 200) {
				RED_LED = 1;
			} else {
				blink_counter = 0;
			}
			return;
		}
		battery_level = 3;
		RED_LED = 0; // 계속 점등
	}
}



/* ====================
 *   Peripheral Init
   ====================*/


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
	P15_OPENDRAIN_MODE; // P15번 확인해보기
    SFRS=0;
    ADCCON1 &= 0x8F;
    ADCCON1 |= (0x01&0x07)<<4; // 최적화 가능
    ADCCON2 &= 0xF1;
    ADCCON2 |= (0x05&0x07)<<1; // 최적화 가능


    ENABLE_ADC_AIN7;
    clr_ADCCON0_ADCF;
    set_ADCCON0_ADCS;
    while(!(ADCCON0&SET_BIT7));        // Wait ADC flag

    current_battery = ADCRH<<4; // 초기화 단계에서 현재 배터리 잔량을 획득
    current_battery |= ADCRL&0x0F;
    DISABLE_ADC;


    /* Board Led Init */
    P11_QUASI_MODE;
	P15_QUASI_MODE;


    /* I2C(Acc) Init */
#if 1
    P13_OPENDRAIN_MODE; // SCL
    P14_OPENDRAIN_MODE; // SDA
    P05_INPUT_MODE;     // I2C Interrupt Pin

    I2CLK = I2C_CLOCK;
    set_I2CON_I2CEN;

    PICON |= 0x40;
    PINEN |= 0x20;
    PIPEN |= 0x20;

    i2c_send_data(0x20, 0x57); // CTRL_REG1 -> 100Hz
    i2c_send_data(0x21, 0x00); // CTRL_REG2 -> High pass filter, autoreset, interrupt
    i2c_send_data(0x22, 0x40); // CTRL_REG3 -> Interrupt 1 setting 0x40, 0x50
    i2c_send_data(0x23, 0x00); // CTRL_REG4 -> BDU(continouse update)
    i2c_send_data(0x24, 0x00); // CTRL_REG5 -> Latch Interrupt
    i2c_send_data(0x25, 0x00); // CTRL_REG6
    i2c_send_data(0x30, 0x08); // INT1_CFG
    i2c_send_data(0x32, 0x10); // INT1_THS -> Interrupt 1 threshold
    i2c_send_data(0x33, 0x02); // INT1_DURATION
#endif

    /* Global Setting */
    ENABLE_GLOBAL_INTERRUPT;
    ENABLE_PIN_INTERRUPT;
}


uint32_t prev_acc = 0;
uint32_t curr_acc = 0;
uint32_t acc_gap = 0;
bool acc_timer = 0;



//uint32_t TIMER2_CT = 65536ul-(20000/256ul*24); // 0xF8B0
void Acc_Timer2_Init(void) {
    TIMER2_AUTO_RELOAD_DELAY_MODE;
    TIMER2_DIV_256;                 /* fix divider 256 */
    TH2 = 0x00;
    TL2 = 0xB0;
    RCMP2H = 0x00;
    RCMP2L = 0xB0;
    clr_T2CON_TF2;
    set_T2CON_TR2;                                   /* Start Timer2  */
    DISABLE_TIMER2_INTERRUPT;
}


bool timer_flag = 0;
bool mcu_flag = 0;
uint16_t timer2_counter = 0;
void main(void)
{
    MODIFY_HIRC(HIRC_24);
    peripheral_init();

#ifdef ENABLE_LOG
    log_init();
#endif

    Timer0_Delay(24000000, 1000, 1000);
    LS_LOG('S');

    Acc_Timer2_Init(); // 움직임에 대한 타이머, 일정 시간 동안 움직임이 없을 경우를 확인한다.

    while(1)
    {
    	if ( mcu_flag ) { // mcu가 활성화
    		GREEN_LED = 0;
    		prev_acc = curr_acc;
    		curr_acc = i2c_read_data(0x2A);

    		// 센서에서 값을 읽어서 gap을 획득
    		if ( curr_acc > prev_acc ) acc_gap = curr_acc - prev_acc;
    		else acc_gap = prev_acc - curr_acc;

    		if ( acc_gap <= 0x200 ) { // 움직임이 발생하지 않는 경우
    			RED_LED = 1; // RED LED 끄기
    			if ( timer2_flag ) { // 인터럽트가 발생했을 경우
    				timer2_counter++; // 카운터를 증가
    				timer2_flag = 0; // 인터럽트 플래그 세팅
    			}
    			LS_LOG('S');
    		}
    		else { // 계속해서 진동이 발생하는 경우
    			RED_LED = 0;
    			timer_flag = 0;
    			timer2_counter = 0;
        	    clr_T2CON_TF2;
        	    set_T2CON_TR2;
    		}

        	if ( timer2_counter >= 10 ) { // 카운터가 10을 넘긴 경우
        		timer2_counter = 0;
        		mcu_flag = 0; // mcu비활성화, i2c 활성화
        		// i2c에 high pass filter interrupt enable config
        		i2c_send_data(0x21, 0x01); // CTRL_REG2 -> High pass filter interrupt 1 enable
        		DISABLE_TIMER2_INTERRUPT; // 타이머2 인터럽트 비활성화
        		LS_LOG('!');
        	}

    	}
    	else { // i2c가 활성화
    		GREEN_LED = 1;
			RED_LED = 1; // RED LED OFF
			GREEN_LED = 1; // GREEN LED OFF
			clr_SCON_1_TI_1;
			clr_SCON_1_RI_1;
			set_PCON_PD;
			CALL_NOP;
			CALL_NOP;
			clr_PCON_PD;

			if ( acc_inter ) { // i2c에서 인터럽트가 발생한 경우 mcu활성화 i2c 비활성화
				// i2c high pass filter interrupt disable config
				mcu_flag = 1;
				i2c_send_data(0x21, 0x00); // CTRL_REG2 -> High pass filter interrupt 1 disable
				ENABLE_TIMER2_INTERRUPT; // 타이머2 인터럽트 활성화
				acc_inter = 0;
				LS_LOG('1');
			}
			else {
				LS_LOG('0');
			}
    	}
    	LS_LOG(',');
		Timer0_Delay(24000000, 1, PERIOD_UNIT);
    }
}
