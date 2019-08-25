/*
 * main.c
 *
 *  Created on: Oct 23, 2018
 *      Author: Daryl
 */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "lpc17xx_pinsel.h" //pinsel
#include "lpc17xx_gpio.h"	//gpio
#include "lpc17xx_i2c.h"	//i2c
#include "lpc17xx_ssp.h"	//UNKNOWN TO FIND OUT
#include "lpc17xx_timer.h"  //Systick
#include "lpc17xx_uart.h"	//uart
#include "led7seg.h" //7 Seg
#include "pca9532.h" //LED
#include "acc.h"     //Accelerometer
#include "oled.h"    //OLED
#include "rgb.h"     //RGB light
#include "light.h"   //light sensor
#include "pca9532.h" //led
#include "temp.h"	//temp

#define SBIT_TIMER	1
#define SBIT_MR0I	0
#define SBIT_MR0R	1
#define SBIT_CNTEN	0
#define	PCLK_TIMER 2
#define MicroToMilliSec(x)	(x*1000)

//global buffer to store strings to print
char OLED_String[80];
volatile char UART_STRING[80];

typedef enum{
	MODE_INITIAL, MODE_CLIMB, MODE_EMERGENCY, MODE_HIGHTEMP
}watch_mode;

uint8_t characters[16] ={0x24,0x7D,0xE0,0x70,0x39,0x32,0x22,0x7C,0x20,0x30,0x28,0x23,0xA6,0x61,0xA2,0xAA};
char saved[6] = {'.', '0', 'E', 'U', 'A', 'S'};
uint8_t emergency_message[5] = {'R','P','T','\\','r'};
// System Variables
volatile watch_mode mode = MODE_INITIAL;
volatile int climb_flag = 0;
volatile uint32_t msTicks;
int one_second;
int five_second = 0;
int time_taken = 0;
volatile uint8_t btnSW4 = 1;
volatile int uart_count = 0;
volatile uint8_t data[5]; //for string to verify interrupt from UART
volatile uint8_t junk[1];
uint32_t three_second = 0;
static int light_flag = 0;
volatile int emergency_flag = 0;
static uint32_t light_reading = 0;
static float temp_reading = 0;
volatile int RGB_flag = 1;
int8_t x=0, y=0, z=0, xoff=0, yoff=0, zoff=0, acix=0, aciy=0, aciz=0;
float x_g=0, y_g=0, z_g=0, acix_g=0, aciy_g=0, aciz_g=0, anx_g=0, any_g=0, anz_g=0, net_acc=0;

//constants
float ACC_THRESHOLD = 0.1;
float LIGHT_THRESHOLD = 3000;
float TEMP_THRESHOLD = 29;


void SysTick_Handler(void){
	msTicks++;
}

static uint32_t getTicks(void){
	return msTicks;
}

static void drawOled(char string[])
{
	oled_clearScreen(OLED_COLOR_BLACK);
    uint8_t x = 0,y = 0;

    uint8_t *tmpbuf;
    tmpbuf = strtok(string, ",");
    while(tmpbuf != NULL) {
    	oled_putString(x, y, tmpbuf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    	tmpbuf = strtok(NULL, ",");
    	y += 10;
    }

    return;
}

static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	LPC_I2C2->I2CONCLR = (I2C_I2CONCLR_AAC| I2C_I2CONCLR_STAC | I2C_I2CONCLR_I2ENC);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_GPIO(void){

	PINSEL_CFG_Type PinCfg;

	//switch 4
	PinCfg.Portnum = 1;
	PinCfg.Funcnum = 0;
	PinCfg.Pinnum = 31;
	PinCfg.OpenDrain = 0;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, (1<<31), 0); //set switch 4 as input


	// Initialize button SW3
	PinCfg.Portnum = 0;
	PinCfg.Funcnum = 0;
	PinCfg.Pinnum = 4;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(0, (1<<4), 0); //set switch 3 as input

	//rotary switch
	PinCfg.Portnum = 0;
	PinCfg.Funcnum = 0;
	PinCfg.Pinnum = 25;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(0, (1<<25), 0); //set rotary switch as input


	//Initialise the interrupt for sw3
	LPC_GPIOINT->IO0IntEnF |= 1<<4;
	LPC_GPIOINT->IO0IntEnF |= 1<<25; //for rotary switch
	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn);

	//Initailise RGB
	//blue
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 26;
	PinCfg.Funcnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(0, (1<<26), 1); //output

	//red
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 0;
	PinCfg.Funcnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1<<0, 1); //output
	GPIO_ClearValue(2, 1<<0);
}

static void init_ssp(void)
{
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

static void uart_init(void){
	UART_CFG_Type uartCfg;
	uartCfg.Baud_rate = 115200;
	uartCfg.Databits = UART_DATABIT_8;
	uartCfg.Parity = UART_PARITY_NONE;
	uartCfg.Stopbits = UART_STOPBIT_1;

	//pin select
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 0;
	PinCfg.Portnum = 0;
	PinCfg.OpenDrain = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);

	//supply power & setup working parameters for uart3
	UART_Init(LPC_UART3, &uartCfg);

	//enable transmit for uart3
	UART_TxCmd(LPC_UART3, ENABLE);
}

static void init_xbee(void){
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 1;
	PinCfg.Pinnum = 0;
	PinCfg.Portnum = 2;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 17;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 17;
	PINSEL_ConfigPin(&PinCfg);

}

unsigned int getPrescalarForTimer(uint8_t timerPclkBit){

	unsigned int pclk, preScalar;

	pclk = (LPC_SC->PCLKSEL0 >> timerPclkBit) & 0x03;	//get pclk info for for required timer
	switch(pclk){

	case 0x00:
		pclk = SystemCoreClock/4;
		break;

	case 0x01:
		pclk = SystemCoreClock;
		break;

	case 0x02:
		pclk = SystemCoreClock/2;
		break;

	case 0x03:
		pclk = SystemCoreClock/8;
		break;

	default:
		pclk = SystemCoreClock/4;
		break;
	}

	preScalar = pclk/1000000 - 1;
	return preScalar;
}

void init_timer0Interrupt(void){
	LPC_SC ->PCONP |= (1<<SBIT_TIMER); //Power on TIMER0
	LPC_TIM0 ->MCR = (1<<SBIT_MR0I) | (1<<SBIT_MR0R); //Clear TC on MRO match and generate interrupt
	LPC_TIM0 ->PR = getPrescalarForTimer(PCLK_TIMER);	//prescalar for 1us
	LPC_TIM0 ->MR0 = MicroToMilliSec(500);	//load timer value to generate 500ms delay
	LPC_TIM0 ->TCR = (1<<SBIT_CNTEN); //start timer counter b setting count enable
}

static void init_peripherals(void){
	 init_i2c();
	 init_ssp();
	 init_GPIO();
	 oled_init();
	 led7seg_init();
	 uart_init();
	 //init_xbee();
	 light_enable();
	 light_setRange(LIGHT_RANGE_4000);
	 pca9532_init(); //led
	 temp_init(&getTicks);
	 acc_init();
	 init_timer0Interrupt();

	 NVIC_SetPriorityGrouping(5);
	 NVIC_SetPriority(SysTick_IRQn, 0x00);
	 NVIC_SetPriority(UART3_IRQn, 0x40);
	 NVIC_SetPriority(EINT3_IRQn, 0x80);
	 NVIC_SetPriority(TIMER0_IRQn, 0xC0);
}

static void initialisation(void){
	sprintf(OLED_String, "Initialization,mode. Press,TOGGLE to climb,.");
	drawOled(OLED_String);
	char msg[50] = "Start \r\n";
	UART_Send(LPC_UART3, (uint8_t *)msg, strlen(msg), BLOCKING);
}

static void initialisation_complete(void){
	sprintf(OLED_String, "INITIALIZATION,COMPLETE.,ENTERING,CLIMB MODE.");
	drawOled(OLED_String);
}

static void initialisation_climb(void){
	sprintf(OLED_String, "CLIMB");
	drawOled(OLED_String);
	sprintf(UART_STRING, "CLIMB mode \r\n");
	UART_Send(LPC_UART3, (uint8_t *)UART_STRING, strlen(UART_STRING), BLOCKING);
}

static void initialisation_emergency(void){
	mode = MODE_EMERGENCY;
	sprintf(UART_STRING, "EMERGENCY! \r\n", light_reading, temp_reading, net_acc);
	UART_Send(LPC_UART3, (uint8_t *)UART_STRING, strlen(UART_STRING), BLOCKING);
	one_second = getTicks(); //reset counters
	time_taken = 0;
	five_second = 0;
	sprintf(OLED_String, "EMERGENCY");
	drawOled(OLED_String);
	climb_flag = 0; //reset flag
	emergency_flag = 1;

	//set interrupts that are used only in emergency mode
	NVIC_ClearPendingIRQ(TIMER0_IRQn); //clear pending interrupts
	NVIC_EnableIRQ(TIMER0_IRQn); //enable interrupt for alternate LED
	//enable interrupt for UART
	NVIC_ClearPendingIRQ(UART3_IRQn); //clear pending interrupts
	NVIC_EnableIRQ(UART3_IRQn);
	UART_IntConfig(LPC_UART3, UART_INTCFG_RBR, ENABLE);


}

static void climb_oled(void){
	if (mode == MODE_HIGHTEMP){
		sprintf(OLED_String, "REST NOW");
	}
	else if(light_flag == 1){
		sprintf(OLED_String, "CLIMB,DIM,Light:%u lux,Temp: %.1f,Acc: %.2f", light_reading, temp_reading, net_acc);
	}
	else{
		sprintf(OLED_String, "CLIMB,Light:%u lux,Temp: %.1f,Acc: %.2f", light_reading, temp_reading, net_acc);
	}
	drawOled(OLED_String);
}

static void emergency_oled(void){
	sprintf(OLED_String, "EMERGENCY Mode!,Temp: %.1f,Acc: %.2f,Time: %d", temp_reading, net_acc, time_taken);
	drawOled(OLED_String);
}

static void switch_mode(void){
	mode = MODE_CLIMB;
}

static void blink_blue(uint8_t counter){
	if(counter%2 == 0){
		GPIO_ClearValue(0, 1<<26); //nothing
	}
	else{
		GPIO_SetValue(0, 1<<26); //blue
	}
}

static void alternate_led(void){
	if(RGB_flag == 1){
		GPIO_ClearValue(0, 1<<26);
		GPIO_SetValue(2, 1<<0); //red
	}
	else{
		GPIO_ClearValue(2, 1<<0);
		GPIO_SetValue(0, 1<<26); //blue
	}
}

static void light_monitor(void){

	light_reading = light_read();
	if(light_reading < LIGHT_THRESHOLD){
		float NUM_LED;
		int i = 0;
		uint16_t LED_ARRAY = 0;
		NUM_LED = ((LIGHT_THRESHOLD - light_reading)/LIGHT_THRESHOLD) * 16;
		int TOTAL_NUM_LED = round(NUM_LED);
		light_flag = 1;
		for(i = 0;i<TOTAL_NUM_LED;i++){
			LED_ARRAY |= 1<<i;
		}
		pca9532_setLeds(LED_ARRAY, 0xFFFF); //set leds
	}
	else{
		pca9532_setLeds(0x0000, 0xFFFF); //clear leds
		light_flag = 0;
	}
}

static void temp_monitor(void){
	temp_reading = temp_read()/10.0;
	if(mode == MODE_EMERGENCY){
		return;
	}
	if(temp_reading > TEMP_THRESHOLD){
		mode = MODE_HIGHTEMP;
		three_second  = getTicks();
	}
}

static void acc_monitor(void){
	acc_read(&x, &y, &z);
	x_g = (x)/64.0;
	y_g = (y)/64.0;
	z_g = (z)/64.0;
	anx_g = x_g - acix_g;
	any_g = y_g - aciy_g;
	anz_g = z_g - aciz_g;
	net_acc = sqrt(anx_g*anx_g + any_g*any_g + anz_g*anz_g);
	if (net_acc > ACC_THRESHOLD) {
		emergency_flag = 1;
	}
}

void EINT3_IRQHandler(void){
	//SW3 is pressed
	if((LPC_GPIOINT->IO0IntStatF >> 4) & 1){
		climb_flag = 1; //switch mode to climb
		LPC_GPIOINT-> IO0IntClr = 1 << 4; //clearing
		if(mode == MODE_EMERGENCY){
			btnSW4 = (GPIO_ReadValue(1) >> 31) & 0x01;
			if(btnSW4 == 0){
				emergency_flag = 0;
			}
		}
	}
	//rotary switch
	if((LPC_GPIOINT->IO0IntStatF >> 25) & 1){
		sprintf(UART_STRING, "SOS!\r\n");
		UART_Send(LPC_UART3, (uint8_t *)UART_STRING, strlen(UART_STRING), BLOCKING);
		LPC_GPIOINT-> IO0IntClr = 1 << 25; //clearing
	}

}

void UART3_IRQHandler(void){

	UART_Receive(LPC_UART3, &junk[0], 1, BLOCKING); //to collect junk to clear interrupt

	if(junk[0] == '\r' || junk[0] == '\n'){
		uart_count = 0;
		sprintf(UART_STRING, "Entry Cleared!\r\n");
		UART_Send(LPC_UART3, (uint8_t *)UART_STRING, strlen(UART_STRING), BLOCKING);
		return;
	}

	data[uart_count] = junk[0];
	uart_count = (uart_count + 1) % 5;
	if(uart_count == 0){
		int i;
		for(i=0; i< 5; i++){
			if(data[i] != emergency_message[i]){
				return;
			}
		}
		sprintf(UART_STRING, "EMERGENCY UART Trigger Temp: %.1f deg Acc: %.2f g Time: %d sec\r\n", temp_reading, net_acc, time_taken);
		UART_Send(LPC_UART3, (uint8_t *)UART_STRING, strlen(UART_STRING), BLOCKING);
	}
}

void TIMER0_IRQHandler(void){
	unsigned int isrMask;
	isrMask = LPC_TIM0-> IR;
	LPC_TIM0->IR = isrMask;

	RGB_flag = -RGB_flag;
	alternate_led();

}

int main(void){

	SysTick_Config(SystemCoreClock/1000);
	init_peripherals();
	initialisation();
	uint8_t counter = 9;
	one_second = getTicks();

	while(1){
		switch(mode){
		case MODE_INITIAL:
			if(climb_flag == 1){
				//print to oled
				initialisation_complete();
				while(counter >= 0){
					if(getTicks() - one_second >= 1000){
						one_second = getTicks();
						led7seg_setChar(characters[counter], TRUE);
						blink_blue(counter); //green led and oled touch the same pin and ports
						if(counter == 0) break;
						counter--;
					}
				}
				switch_mode();
				initialisation_climb();
				acc_read(&acix, &aciy, &aciz);
				acix_g = (acix)/64.0;
				aciy_g = (aciy)/64.0;
				aciz_g = (aciz)/64.0;
			}
			break;

		case MODE_CLIMB:
			//every second
			if(getTicks() - one_second >= 1000){
				five_second++;
				one_second = getTicks();
				acc_monitor();
				light_monitor();
				temp_monitor();
				climb_oled();
			}
			//5 seconds
			if(five_second >= 5){
				five_second = 0;
				sprintf(UART_STRING, "CLIMB. Light: %u lux Temp: %.1f deg Acc: %.2f g \r\n", light_reading, temp_reading, net_acc);
				UART_Send(LPC_UART3, (uint8_t *)UART_STRING, strlen(UART_STRING), BLOCKING);

			}
			if(emergency_flag == 1){
				initialisation_emergency();
			}
			break;

		case MODE_HIGHTEMP:
			if(getTicks() - three_second <= 3000){
				break;
			}

			oled_clearScreen(OLED_COLOR_BLACK);
			temp_reading = temp_read()/10.0;
			if(temp_reading <= TEMP_THRESHOLD){
				mode = MODE_CLIMB;
				break;
			}
			if(getTicks() - one_second >= 1000){
				one_second = getTicks();
				sprintf(UART_STRING, "Resting now due to overheat. Temp: %.1f deg \r\n", temp_reading);
				UART_Send(LPC_UART3, (uint8_t *)UART_STRING, strlen(UART_STRING), BLOCKING);
			}
			break;

		case MODE_EMERGENCY:
			if(getTicks() - one_second >= 1000){
				time_taken++;
				five_second++;
				one_second = getTicks();
				acc_monitor();
				temp_monitor();
				emergency_oled();
			}

			//5 seconds
			if(five_second >= 5){
				five_second = 0;
				sprintf(UART_STRING, "EMERGENCY. Temp: %.1f deg Acc: %.2f g Time: %d sec\r\n", temp_reading, net_acc, time_taken);
				UART_Send(LPC_UART3, (uint8_t *)UART_STRING, strlen(UART_STRING), BLOCKING);
			}
			//if go back climb
			//reset emergency flag
			if(emergency_flag == 0){
				mode = MODE_CLIMB;
				NVIC_DisableIRQ(TIMER0_IRQn);
				NVIC_DisableIRQ(UART3_IRQn); //disable interrupts
				GPIO_ClearValue(2, 1<<0);
				sprintf(UART_STRING, "Emergency is cleared! Time consumed for recovery: %d sec\r\n", time_taken);
				UART_Send(LPC_UART3, (uint8_t *)UART_STRING, strlen(UART_STRING), BLOCKING);
				sprintf(OLED_String, "Emergency,cleared! Time:,%d sec", time_taken);
				drawOled(OLED_String);

				uint8_t count = 5;
				one_second = getTicks();
				while(count > 0){
					if(getTicks() - one_second >= 1000){
						one_second = getTicks();
						blink_blue(count);
						led7seg_setChar(saved[count], FALSE);
						count--;
					}
				}
				GPIO_ClearValue(0, 1<<26); //clear
			}

			break;
		}
	}

}
