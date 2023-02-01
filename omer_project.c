// PIN CONNECTIONS:
// -DISTANCE SENSOR PORTE0
// -Servo1 PORTC9 channel 5
// - Servo2 PortC8 channel 4
// -LCD PORTD 0-7
// -LCD RS PORTA 2
// -LCD RW PORTA 4
// -LCD EN PORTA 5
// -UART txd PORTA 1

#include <MKL25Z4.H>
#include <stdio.h>
#include <math.h>
#include <string.h>


#define RS 0x04     /* PTA2 mask */ 
#define RW 0x10     /* PTA4 mask */ 
#define EN 0x20     /* PTA5 mask */
#define SERVO_MAX_ANGLE 180
#define SERVO_MIN_ANGLE 0

void Delay(volatile unsigned int time_del);
void LCD_command(unsigned char command);
void LCD_data(unsigned char data);
void LCD_init(void);
void LCD_ready(void);
void servo_init(void);
void servo_angle(int angle, int servo_num);
void distance_sensor_init(void);
void LCD_print(char input[], int lenght);
void UART0_init(void);
void UART0_IRQHandler(void);
void PORTA_IRQHandler(void);
void get_direction(void);
void get_direction2(void);
void ADC0_init(void);

static volatile int counter = 0;
static volatile int angle = 60;
int lenght,lenght1,lenght2;
int result;
int resulta,resultb;

int lenght1 = sizeof("OT HOUSE");
int lenght2 = sizeof("WELCOME!");

int main(void){
	LCD_init();
	servo_init();
	UART0_init();
	ADC0_init();
	LCD_command(0x80);
	LCD_print("OT HOUSE",lenght1);
	Delay(300000);
	servo_angle(1,0x01);
	servo_angle(179,0x02);
	distance_sensor_init();
	while(1){
			get_direction();
			get_direction2();
		}
}

void UART0_init(void) { // state baglama, en ground, VCC 5V, GNDGND, TRX
SIM->SCGC4 |= 0x0400; /* enable clock for UART0 */
SIM->SOPT2 |= 0x04000000; /* use FLL output for UART Baud rategenerator */
UART0->C2 = 0; /* turn off UART0 while changing configurations */
UART0->BDH = 0x00;
UART0->BDL = 137; /* 9600 Baud */
UART0->C4 = 0x0F; /* Over Sampling Ratio 16 */
UART0->C1 = 0x00; /* 8-bit data */
UART0->C2 = 0x24; /* enable receive and receive interrupt*/
NVIC->ISER[0] |= 0x00001000; /* enable INT12 (bit 12 of ISER[0]) */
SIM->SCGC5 |= 0x0200; /* enable clock for PORTA */
PORTA->PCR[1] = 0x0200; /* make PTA1 UART0_Rx pin */
}

void servo_init(void){
SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; //Port C enable
PORTC->PCR[9] = 0x0300;
PORTC->PCR[8] = 0x0300;
SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK;
SIM->SOPT2 |= 0x01000000; /* Counter clock for TPM */
TPM0->SC = 0; /* disable timer */

TPM0->CONTROLS[5].CnSC = 0x20 | 0x08; /* center-aligned, pulse high */
TPM0->CONTROLS[4].CnSC = 0x20 | 0x08; /* center-aligned, pulse high */

}

void ADC0_init(void)
{
SIM->SCGC5 |= 0x2000; /* clock to PORTE */
PORTE->PCR[20] = 0; /* PTE20 analog input */
PORTE->PCR[21] = 0; /* PTE21 analog input */
SIM->SCGC6 |= 0x8000000; /* clock to ADC0 */
ADC0->SC2 &= ~0x40; /* software trigger */
/* clock div by 4, long sample time, single ended 12 bit, bus clock */
ADC0->CFG1 = 0x40 | 0x10 | 0x04 | 0x00;
}


void distance_sensor_init(void){ // DEGISTIR PORTA 16
	__disable_irq(); // disable all IRQs /
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;  //PORT A CLOCK
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;  //PORT E CLOCK
  PORTA->PCR[16] &= ~PORT_PCR_MUX(1);// make gpio
  PORTA->PCR[16] |= PORT_PCR_MUX(1);// make gpio
	PORTE->PCR[29] &= ~PORT_PCR_MUX(1);// make gpio
  PORTE->PCR[29] |= PORT_PCR_MUX(1);// make gpio
  PTA->PDDR &= ~(1UL<<16); // make input
	PTE->PDDR |= (1UL<<29); // make OUTPUT
	PORTA->PCR[16] &= ~0xF0000; // clear interrupt selection /
	PORTA->PCR[16] |= 0xA0000; // enable falling edge interrupt */
	NVIC->ISER[0] |= 0x40000000; /* enable IRQ 30 in NVIC */
	__enable_irq(); /* enable interrupts */
}


void PORTA_IRQHandler(void) {//MESAFE
  if (PORTA->ISFR  == 0x10000) {
		PTE-> PDOR |= (1UL<<29);
		
}
	PORTA->ISFR = (1UL << 16); // clear interrupt flags for  PTA16
}

void UART0_IRQHandler(void){//UART
char c;
c = UART0->D;
if(c=='F'){
	 servo_angle(50,0x01);
	 servo_angle(50,0x02);
	 PTE-> PCOR |= (1UL<<29);
}
}

void servo_angle(int angle, int servo_num){
	
	float duty; 
	int mod, cnv=0;
	mod = (int)(20971520/(2*16*50));

	// servo 1
	if(servo_num == 0x1){
	duty = (13*angle - 2*angle + 360) / 180;
	cnv = duty*mod/100;
	
	TPM0->MOD = mod;
	TPM0->CONTROLS[5].CnV = cnv;// channel 5 port C 9
  TPM0->SC = 0x0C | 0x20;}
	
	// servo 2
	if(servo_num == 0x2){
	duty = (13*angle - 2*angle + 360) / 180;
	cnv = duty*mod/100;
	
	TPM0->MOD = mod;
	TPM0->CONTROLS[4].CnV = cnv;// channel 4 port C 8
  TPM0->SC = 0x0C | 0x20;}
}


void get_direction(void){
ADC0->SC1[0] = 0; /* start conversion on channel 4 */
while(!(ADC0->SC1[0] & 0x80)) { } /* wait for conversion complete */
resulta = ADC0->R[0]; /* read conversion result and clear COCO flag */
if (resulta<2460){
servo_angle(50,0x01);
PTE-> PCOR |= (1UL<<29);
LCD_command(0x01);
LCD_command(0x80);
LCD_print("WELCOME!",lenght2);
}
}


void get_direction2(void){
ADC0->SC1[0] = 4; /* start conversion on channel 4 */
while(!(ADC0->SC1[0] & 0x80)) { } /* wait for conversion complete */
resultb = ADC0->R[0]; /* read conversion result and clear COCO flag */
if (resultb<2460){
	servo_angle(50,0x02);
	PTE-> PCOR |= (1UL<<29);
}
}



void LCD_init(void)
{
    SIM->SCGC5 |= 0x1000;       /* enable clock to Port D */ 
    PORTD->PCR[0] = 0x100;      /* make PTD0 pin as GPIO */
    PORTD->PCR[1] = 0x100;      /* make PTD1 pin as GPIO */
    PORTD->PCR[2] = 0x100;      /* make PTD2 pin as GPIO */
    PORTD->PCR[3] = 0x100;      /* make PTD3 pin as GPIO */
    PORTD->PCR[4] = 0x100;      /* make PTD4 pin as GPIO */
    PORTD->PCR[5] = 0x100;      /* make PTD5 pin as GPIO */
    PORTD->PCR[6] = 0x100;      /* make PTD6 pin as GPIO */
    PORTD->PCR[7] = 0x100;      /* make PTD7 pin as GPIO */
    PTD->PDDR = 0xFF;           /* make PTD7-0 as output pins */
    
    SIM->SCGC5 |= 0x0200;       /* enable clock to Port A */ 
    PORTA->PCR[2] = 0x100;      /* make PTA2 pin as GPIO */
    PORTA->PCR[4] = 0x100;      /* make PTA4 pin as GPIO */
    PORTA->PCR[5] = 0x100;      /* make PTA5 pin as GPIO */
    PTA->PDDR |= 0x34;          /* make PTA5, 4, 2 as output pins */
    
    LCD_command(0x38);      /* set 8-bit data, 2-line, 5x7 font */
    LCD_command(0x01);      /* clear screen, move cursor to home */
    LCD_command(0x0F);      /* turn on display, cursor blinking */
}

/* This function waits until LCD controller is ready to
 * accept a new command/data before returns.
 */
void LCD_ready(void)
{
    uint32_t status;
    
    PTD->PDDR = 0x00;          /* PortD input */
    PTA->PCOR = RS;         /* RS = 0 for status */
    PTA->PSOR = RW;         /* R/W = 1, LCD output */
    
    do {    /* stay in the loop until it is not busy */
			  PTA->PCOR = EN;
			  Delay(500);
        PTA->PSOR = EN;     /* raise E */
        Delay(500);
        status = PTD->PDIR; /* read status register */
        PTA->PCOR = EN;
        Delay(500);			/* clear E */
    } while (status & 0x80UL);    /* check busy bit */
    
    PTA->PCOR = RW;         /* R/W = 0, LCD input */
    PTD->PDDR = 0xFF;       /* PortD output */
}

void LCD_command(unsigned char command)
{
    LCD_ready();			/* wait until LCD is ready */
    PTA->PCOR = RS | RW;    /* RS = 0, R/W = 0 */
    PTD->PDOR = command;
    PTA->PSOR = EN;         /* pulse E */
    Delay(500);
    PTA->PCOR = EN;
}

void LCD_data(unsigned char data)
{
    LCD_ready();			/* wait until LCD is ready */
    PTA->PSOR = RS;         /* RS = 1, R/W = 0 */
    PTA->PCOR = RW;
    PTD->PDOR = data;
    PTA->PSOR = EN;         /* pulse E */
    Delay(500);
    PTA->PCOR = EN;
}

/* Delay n milliseconds
 * The CPU core clock is set to MCGFLLCLK at 41.94 MHz in SystemInit().
 */

/* delay n microseconds
 * The CPU core clock is set to MCGFLLCLK at 41.94 MHz in SystemInit().
 */


void Delay(volatile unsigned int time_del) {
  while (time_del--) 
		{
  }
}

void LCD_print(char input[],int lenght){
	for (uint8_t i = 0; i < lenght-1; i++) 
	{
		LCD_data(input[i]);
  }
}

