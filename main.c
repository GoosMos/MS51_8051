#include "ms51_16k_sdcc.h"
#include "stdbool.h"

#define ENABLE_LOG

#define CHARGE_PIN				P00
#define BUTTON_PIN				P04
#define ACC_PIN					P05
#define PLUG_PIN				P07
#define REAR_ENABLE_PIN			P10
#define RED_LED 				P11
#define REAR_PIN				P12
#define GREEN_LED 				P15


#define CLR_CHG_PIN 			CLR_BIT0
#define CLR_BTN_PIN				CLR_BIT4
#define CLR_ACC_PIN 			CLR_BIT5
#define CLR_PLUG_PIN			CLR_BIT7

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
bool plug_inter = 0;
bool timer2_inter = 0;


uint32_t prev_acc = 0;
uint32_t curr_acc = 0;
uint32_t acc_gap = 0;

uint32_t movement_timer = 0;


bool mcu_flag = 0;
bool plug_flag = 0;
bool charge_flag = 0;
bool break_status = 0; // acc_gap에 따른 브레이크 판단 플래그


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


void Pin_Interrupt_ISR(void) __interrupt (7)
{
	PUSH_SFRS;

#if 0
	if ( PIF & 0x01 ) { // interrupt channel 0
		if ( !temp_chg ) temp_chg = 1;
	}

	PIF &= CLR_CHG_PIN;
#else
	if ( PIF & 0x01 ) {
		if ( !charge_flag ) charge_flag = 1;
	}
	if ( PIF & 0x10 ) { // interrupt channel 4
		if ( !button_pressed ) button_pressed = 1;
	}

	if ( PIF & 0x20 ) { // interrupt channel 5
		if ( !acc_inter ) acc_inter = 1;
	}

	if ( PIF & 0x80 ) { // interrupt channel 7
		if ( !plug_inter ) plug_inter = 1;
	}


	PIF &= CLR_BTN_PIN;
	PIF &= CLR_ACC_PIN;
	PIF &= CLR_PLUG_PIN;
#endif
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

	// Send Sensor Memory Address
	I2DAT = (GYRO_ADDR | I2C_WR);
	clr_I2CON_SI;
	while(!SI);

	// Send Reg Address
	I2DAT = reg_addr;
	clr_I2CON_SI;
	while(!SI);

	// Send Reg Value
	I2DAT = reg_val;
	clr_I2CON_SI;
	while(!SI);

    // Send Stop Bit
    set_I2CON_STO;
    clr_I2CON_SI;
    while (STO);
}

uint16_t i2c_read_data(uint8_t reg_addr)
{
	check_count = 0;
	uint16_t axis_data = 0;

	set_I2CON_STA; // Send Start Bit
	clr_I2CON_SI;
	while(!SI);
	clr_I2CON_STA;

	I2DAT = (GYRO_ADDR | I2C_WR); // Send Gyro Address + Write Bit
	clr_I2CON_SI;
	while(!SI);

	I2DAT = (reg_addr | 0x80); // Who Am I Reg Address
	clr_I2CON_SI;
	while(!SI);

    set_I2CON_STA; // Repeated Start Bit
    clr_I2CON_SI;
    while (!SI);
    clr_I2CON_STA;

    I2DAT = (GYRO_ADDR | I2C_RD); // Send Read Bit
    clr_I2CON_SI;
    while (!SI);

    // Read Data
    set_I2CON_AA;
    clr_I2CON_SI;
    while(!SI);
    axis_data = I2DAT;

    // Read Next Data
    set_I2CON_AA;
    clr_I2CON_SI;
    while(!SI);
    axis_data = (I2DAT << 8);

    clr_I2CON_AA; // Send NMAK to Slave
    clr_I2CON_SI;
    while(!SI);

    // Stop
    set_I2CON_STO;
    clr_I2CON_SI;
    while(STO);

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

void rearlight_pwm_enable(void) {
	REAR_ENABLE_PIN = 1;  // PWM ENABLE
}

void rearlight_pwm_disable(void) {
	REAR_ENABLE_PIN = 0;  // PWM ENABLE
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
//	LS_LOG(':');
//	LS_LOGN(input); // plugged = 1527 - 1528
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
				current_rear = prev_rear;    // 길게 눌러서 전원을 키는 동작 가장 최근 사용된 모드로 시작
			} else {
				prev_rear = current_rear; // 마지막 모드를 저장
				current_rear = REAR_OFF;  // 길게 눌러서 전원을 끄는동작
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
				}
			}
			button_pressed = button_unpressed = 0;
		}
	}
}

void onboard_routine(void)
{
	if( plug_flag ) { // 충전 케이블이 연결되어있는 상태
		battery_level = 0;
		if ( current_battery > 4200 ) {
			RED_LED = 1;
			GREEN_LED = 0;
			blink_counter = 0;
		}
		else {
			GREEN_LED = 1;
			if ( blink_counter < 50 ) {
				RED_LED = 0;
			} else if ( blink_counter < 500 ) {
				RED_LED = 1;
			} else blink_counter = 0;

			blink_counter++;
		}
	} else { // 충전선이 연결되어있지 않은 상태
		GREEN_LED = 1;

		if( current_rear == REAR_OFF ) { // 현재 sleep 상태에서 충전선이 연결되어있지 않는 상태
			RED_LED = 1; // RED LED 끄기
			return;
		}
		if( battery_level == 0 && current_battery > 4200 ) { //3250 3300, 배터리 잔량이 70 ~ 100% 사이인 경우
			RED_LED = 1; // RED LED 끄기
//			LS_LOG('A');
			return;
		}
		if( battery_level <= 1 && current_battery > 3200 ) {
			battery_level = 1;
			blink_counter ++;
			if(blink_counter < 100) {
				RED_LED = 0;
			} else if( blink_counter < 1000 ) {
				RED_LED = 1;
			} else {
				blink_counter = 0;
			}
//			LS_LOG('B');
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
//			LS_LOG('C');
			return;
		}
		battery_level = 3;
		RED_LED = 0; // 계속 점등
//		LS_LOG('D');
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

	P10_QUASI_MODE;
	P12_PUSHPULL_MODE;

    clr_PWMCON0_PWMRUN;
	set_SFRS_SFRPAGE;

	PWMPH = MAX_PWM; // 255 bit PWM Period High Byte
	PWMPL = MAX_PWM; // 255 bit PWM Period Low Byte

	PWM0H = 0;  // PWM Channel 0~5 Duty High Byte n = 0,1,2,3,4,5
	PWM0L = 0; // PWM Channel 0~5 Duty Low  Byte n = 0,1,2,3,4,5

	clr_SFRS_SFRPAGE;
	set_PWMCON0_PWMRUN;
	REAR_ENABLE_PIN = 1;  // PWM ENABLE


    /* Button Init */
	P04_INPUT_MODE; 		// button init - P04

	PICON |= 0x40;
	PINEN |= 0x10;
	PIPEN |= 0x00;

	/* Plug Init */
	P07_INPUT_MODE;

	PICON |= 0x80;
	PINEN |= 0x80;
	PIPEN |= 0x00;


	/* Charge Init */
	P00_INPUT_MODE;

	PICON |= 0x04;
	PINEN |= 0x01; // falling interrupt
	PIPEN |= 0x01; // rising interrupt


    /* Battery Init */
	P03_OPENDRAIN_MODE; // P03
    SFRS=0;
    ADCCON1 &= 0x8F;
    ADCCON1 |= (0x01&0x07)<<4; // 최적화 가능
    ADCCON2 &= 0xF1;
    ADCCON2 |= (0x05&0x07)<<1; // 최적화 가능


    ENABLE_ADC_CH6;
    clr_ADCCON0_ADCF;
    set_ADCCON0_ADCS;
    while(!(ADCCON0&SET_BIT6));        // Wait ADC flag

    current_battery = ADCRH << 4; // 초기화 단계에서 현재 배터리 잔량을 획득
    current_battery |= ADCRL&0x0F;
    DISABLE_ADC;


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

    i2c_send_data(0x20, 0x57); // CTRL_REG1 -> 100Hz
    i2c_send_data(0x21, 0x00); // CTRL_REG2 -> High pass filter, autoreset, interrupt
    i2c_send_data(0x22, 0x40); // CTRL_REG3 -> Interrupt 1 setting 0x40, 0x50
    i2c_send_data(0x23, 0x00); // CTRL_REG4 -> BDU(continouse update)
    i2c_send_data(0x24, 0x00); // CTRL_REG5 -> Latch Interrupt
    i2c_send_data(0x25, 0x00); // CTRL_REG6
    i2c_send_data(0x30, 0x08); // INT1_CFG
    i2c_send_data(0x32, 0x08); // INT1_THS -> Interrupt 1 threshold
    i2c_send_data(0x33, 0x02); // INT1_DURATION


    /* Global Setting */
    ENABLE_GLOBAL_INTERRUPT;
    ENABLE_PIN_INTERRUPT;
}


int32_t convert(uint32_t v) {
	if ( v & 0x8000 ) return (int32_t)(v | 0xFFFF0000);
	else return (int32_t)v;
}


// 센서에서 값을 읽어서 gap을 획득
void i2c_sampling(void) {
#if 0
	prev_acc = curr_acc;
	curr_acc = i2c_read_data(0x2A);
	if ( curr_acc > prev_acc ) acc_gap = curr_acc - prev_acc;
	else acc_gap = prev_acc - curr_acc;
#else
	prev_acc = curr_acc;
	curr_acc = i2c_read_data(0x2A);

	int32_t p_acc = convert(prev_acc);
	int32_t c_acc = convert(curr_acc);

	if ( c_acc > p_acc ) acc_gap = c_acc - p_acc;
	else acc_gap = p_acc - c_acc;
#endif
}


void break_degree(void) {
	uint32_t curr_deg = i2c_read_data(0x2A);
	if ( curr_deg > 16368 ) return;
	if ( curr_deg > 4096 ) {
		if ( curr_deg > 12032 ) break_status = 1;
	}
	else break_status = 0;
}



void main(void)
{
    MODIFY_HIRC(HIRC_24);
    peripheral_init();
#ifdef ENABLE_LOG
    log_init();
#endif
    Timer0_Delay(24000000, 1000, 1000);
    LS_LOG('i');

    while(1) {
    	process_button();

    	if ( mcu_flag ) { // mcu가 제어권을 가지는 상태
    		GREEN_LED = 0;

    		if ( plug_flag && !PLUG_PIN ) plug_flag = 0; // 충전 연결 상태를 확인

    		if ( acc_gap > 512 ) { // 움직임을 확인
    			movement_timer = 0;
    		}
    		else {
    			if ( !plug_flag ) { // 충전선이 연결되어있지 않은 경우
    				movement_timer++; // 움직임이 없는 경우
    			}
    			else movement_timer = 0; // 충전선이 연결되어있는 경우
    		}

    		break_degree(); // 각도 판단 -> break_status = 0 | 1

    		// mcu가 샘플링하는 루틴에서의 후미등 처리
			if ( (current_rear & 0x02) && break_status ) { // 브레이크를 사용하는 모드의 경우
				rearlight_pwm_enable();
				backlight_routine(0xFF, 0xFF);
			}
			else backlight_routine(low_bright[current_rear], high_bright[current_rear]);

			if ( movement_timer >= 0x0FFF ) { // 일정시간 움직임이 없는 경우
				movement_timer = 0;
				if ( !plug_flag ) { // 충전선이 연결되어있지 않은 경우
					mcu_flag = 0;
					// i2c에 high pass filter interrupt enable config
					i2c_send_data(0x21, 0x01); // CTRL_REG2 -> High pass filter interrupt 1 enable
					rearlight_pwm_disable(); // 후미등 끄기
					LS_LOG('!');
				}
			}
    	}
    	else {
			RED_LED = 1;   // RED LED OFF
			GREEN_LED = 1; // GREEN LED OFF
			clr_SCON_1_TI_1;
			clr_SCON_1_RI_1;
			set_PCON_PD;
			CALL_NOP;
			CALL_NOP;
			clr_PCON_PD;

			if ( acc_inter || plug_inter ) { // i2c에서 인터럽트가 발생한 경우 mcu활성화 i2c 비활성화
				// i2c high pass filter interrupt disable config
				mcu_flag = 1;
				i2c_send_data(0x21, 0x00); // CTRL_REG2 -> High pass filter interrupt 1 disable
				rearlight_pwm_enable();

				if ( acc_inter ) {
					LS_LOG('A');
					acc_inter = 0;
				}

				if ( plug_inter ) {
					LS_LOG('P');
					plug_flag = 1;
					plug_inter = 0;
				}
			}
    	}

		if( current_rear != REAR_OFF ) { // 켜져있는 상태
			update_battery();
			onboard_routine();
		}

    	Timer0_Delay(24000000, 1, PERIOD_UNIT);
    }
}
