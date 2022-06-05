#include "SSD1306.h"
#include "Font5x8.h"
#include "Font6x8.h"
#include "Skull.h"
#include "ArrowRight.h"
#include "ArrowLeft.h"
#include "ArialNarrowDig18x32.h"
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
volatile int speedTimer = 0;
volatile int speedDisp = 0;
volatile char refresh = 1;			//флаг обновления показаний спидометра
volatile unsigned long total_rotations = 0;	//счетчик оборотов колеса (по нему будут определяться показания одометра)
double T = 0;						
double C = 0;//окружность колеса
volatile double speedKMH = 0;			//скорость в км/час
volatile uint8_t arrowMoving = 0;
int newSteps = 0;
uint8_t signalOn = 0;//если включен поворот или аварийка, = 1
uint8_t firstMeasure = 0;

uint8_t contrast = 0;
uint8_t magnets = 6;
double ratio = 1; //передаточное число (нужно, если датчикустановлен не на колесе)
double WEEL_D = 0.70;			//диаметр колеса в метрах
unsigned int PWM_arrow = 1024;// из 1000
unsigned int PWM_digits = 1024;// из 1000
double DEG_PER_KMH = 1.275;	//градусов на км/час на шкале
uint8_t SCALE_MAX	=190;		//максимум на спидометре
uint8_t SHUT_DOWN_VOLTAGEx10= 80;
uint8_t step_interval =150;
uint8_t SM_STEPS=	96;		//количество шагов ШД (ШД от сканера Musteck - 96 шагов)

unsigned int signalCounter = 0;//счетчик длительности интервала сигнала поворота
unsigned long dist = 0;
unsigned long newDist = 0;
double kmh_per_step = 0;

//uint8_t debug1 = 0;
//uint8_t debug2 = 0;
//uint8_t debug3 = 0;


#define pi 3.141592653		//число пи
#define PRESCALER 256
#define TIC  20			//количество тиков Timer2 (при F_CPU 16Mhz и presc=256 1 тик = 16 мкс). Количество тиков определяет минимальный период отсчета времени между проходами магнита 
#define AREF 2.5			//опорное напряжение
#define DEVIDER 6			//коэффициент делителя (для измерения напряжения на батарее)




#define FULL_STEP_ONE_PHASE 0
#define HALF_STEP 1

char step_mode = HALF_STEP;


int readADC(unsigned char mux, unsigned char cycles);
void step(char mode);
void calc_speed();
void DrawArrow (char dir);
void DrawSkull ();
unsigned char eeprom_read_byte(unsigned int uiAddress);
void eeprom_write_byte(unsigned int uiAddress, unsigned char ucData);
void main_screen();
void speed_arrow_update();
void voltage_monitor();
void signal_monitor();
uint8_t button_monitor();
void menu_screen();
void arrow_calibration();


volatile signed int phase = 0;
uint8_t dir = 0;
volatile int steps = 0;
char btn_pressed = 0;
int voltage = 0;
int newVoltage = 0;




const unsigned char phase_arr_full_step_1phase [] = {
	0b00000111,
	0b00001110,
	0b00000001,
	0b00001000
};								//ONE PHASE



const unsigned char phase_arr_half_step [] = {
	0b00000101,
	0b00000110,
	0b00001010,
	0b00000010,
	0b00000001,
	0b00000000,
	0b00001000,
	0b00000100			//HALF STEP
};




void presets (void){


C = WEEL_D * pi/magnets;					//длина окружности между магнитами
T = 1.0/(F_CPU/PRESCALER);//период отсчета времени в секундах (16 мкс) (prsc=256)

	
DDRA|=_BV(3); //ENABLE 1
DDRA|=_BV(0); //ENABLE 2
DDRA|=_BV(2); //PHASE 1
DDRA|=_BV(1); //PHASE 2
DDRD|=_BV(4);//PWM DIGITS LIGHT
DDRD|=_BV(5);//PWM ARROW LIGHT


PORTA|=_BV(3);//ENABLE 1 high (disabled)
PORTA|=_BV(0);//ENABLE 2 high (disabled)

PORTB|=_BV(5);//внутренняя подтяжка для внешних кнопок
PORTB|=_BV(6);
PORTB|=_BV(7);
	
		//=======================АЦП
		ADCSRA |= _BV(ADEN);
		//=======================
		ADCSRA |= _BV(ADPS0);		//
		ADCSRA |= _BV(ADPS1);		// Прескейлер 128
		ADCSRA |= _BV(ADPS2);		//
		//=======================
	
	//инициализация дисплея
	GLCD_Setup();
	GLCD_Clear();	
	GLCD_Render();
	//================= чтение сохраненных в EEPROM данных
	
	
	
	
	if(eeprom_read_byte(10)){
		//====общее количество оборотов, 4 байта
		//eeprom_write_byte(20, 0);
		eeprom_write_byte(21, 1);
		eeprom_write_byte(22, 204);
		eeprom_write_byte(23, 204);//117964
		//====заполнение ШИМ подсветки стрелки
		eeprom_write_byte(30, 255);
		eeprom_write_byte(31, 200);
		//====заполнение ШИМ подсветки циферблата
		eeprom_write_byte(32, 0);
		eeprom_write_byte(33, 50);
		//====контраст дисплея
		eeprom_write_byte(34, 255);
		//====
		eeprom_write_byte(10, 0);
	}
	
	//==================
	//unsigned long a = eeprom_read_byte(20);
	//a=(a<<24);
	unsigned long b = eeprom_read_byte(21);
	b=(b<<16);
	unsigned long c = eeprom_read_byte(22);
	c=(c<<8);
	unsigned long d = eeprom_read_byte(23);
	
	total_rotations=b+c+d;  //a+
	
	//total_rotations=117964;
	/*
	b=eeprom_read_byte(30);
	b=(b<<8);
	c=eeprom_read_byte(31);
	
	PWM_arrow = b+c;
	
	b=eeprom_read_byte(32);
	b=(b<<8);
	c=eeprom_read_byte(33);
	
	PWM_digits = b+c;
	*/
	contrast= eeprom_read_byte(34);
	
	//ШИМ подсветки===============
	TCCR1A = _BV(WGM10)|_BV(WGM11)|_BV(COM1B1)|_BV(COM1A1);
	TCCR1B = _BV(WGM12)|_BV(CS10)|_BV(CS11);
	OCR1A= PWM_arrow;
	OCR1B= PWM_digits;
	//============================
	
arrow_calibration();



MCUCR|= _BV(ISC11); // Внешнее прерывание по падающему фронту на INT1
GICR|=_BV(INT1); // Разрешение внешнего прерывания на INT1

TCCR2|=_BV(CS21)|_BV(CS22)|_BV(WGM21);
OCR2=TIC;




sei();


	
//	total_rotations=0;

}

ISR( TIMER0_COMP_vect ){

step(step_mode);		
	if (steps == newSteps){
	arrowMoving=0;
	TCCR0=0;
	OCR0=0;
	TIMSK&=~_BV(OCIE0);
	}
	
	


}


void step(char mode){
	char tempPort=0;;
	signed int tempPhase = 0;
	
	if (dir)
	{
		
	phase++;
	steps++;
	
	}
	else
	{
		
		phase--;
		steps--;
	}
	if(mode == FULL_STEP_ONE_PHASE){
		if (phase<0) phase = 3;
		if (phase>3) phase = 0;
	}
	if(mode == HALF_STEP){
		if (phase<0) phase = 7;
		if (phase>7) phase = 0;
	} 
	tempPhase=phase;





	tempPort=PORTA;
	tempPort&=~_BV(0);
	tempPort&=~_BV(1);
	tempPort&=~_BV(2);
	tempPort&=~_BV(3);
	if(mode == FULL_STEP_ONE_PHASE) tempPort|=phase_arr_full_step_1phase[tempPhase];
	if(mode == HALF_STEP) tempPort|=phase_arr_half_step[tempPhase];

	PORTA=tempPort;
	
	/*
	GLCD_Clear();
	GLCD_GotoXY(0, 31);
	GLCD_PrintInteger(steps);
	GLCD_Render();
	*/
}




ISR( TIMER2_COMP_vect ){
	speedTimer++; //инкрементация speedTimer каждый период Т (320 мкс)
	}
ISR (TIMER1_OVF_vect){
	if (signalOn) signalCounter++;
}
ISR(INT1_vect){

if (firstMeasure==0){
	TIMSK|=_BV(OCIE2);
	TCNT2=0;
	firstMeasure = 1;
	//при первом срабатывании датчика запускается TIMER2
}
else
	{
			
			
			
		speedDisp=(speedTimer*TIC)+TCNT2;
		TCNT2=0;	
		speedTimer=0;
		refresh = 1;
		total_rotations++;
	}
}

void menu_screen(){
uint8_t offset = 75;	
static int8_t menu_item;
static int8_t page;
if (page<0) page = 0;
if (menu_item >5){page++;menu_item=0;}
if (menu_item <0){page--;menu_item=5;}
GLCD_Clear();
GLCD_FillRectangle(0,0+menu_item*8-1+8,5,7+menu_item*8+8,GLCD_Black);
GLCD_FillRectangle(122,0+menu_item*8-1+8,127,7+menu_item*8+8,GLCD_Black);
GLCD_DrawLine(0,menu_item*8-2+8,127,menu_item*8-2+8,GLCD_Black);
GLCD_DrawLine(0,menu_item*8+8+8,127,menu_item*8+8+8,GLCD_Black);

	GLCD_SetFont(Font5x8, 5, 8, GLCD_Merge);
if(page==0){
GLCD_GotoX(10);	
GLCD_GotoLine(1);
GLCD_PrintString("Dig_PWM");
GLCD_GotoX(offset);
GLCD_PrintInteger(PWM_digits);

GLCD_GotoX(10);
GLCD_GotoLine(2);
GLCD_PrintString("Arr_PWM");
GLCD_GotoX(offset);
GLCD_PrintInteger(PWM_arrow);

GLCD_GotoX(10);
GLCD_GotoLine(3);
GLCD_PrintString("Weel D");	
GLCD_GotoX(offset);
GLCD_PrintDouble(WEEL_D,100);
	
GLCD_GotoX(10);
GLCD_GotoLine(4);
GLCD_PrintString("Ratio");	
GLCD_GotoX(offset);
GLCD_PrintDouble(ratio,100);

GLCD_GotoX(10);
GLCD_GotoLine(5);
GLCD_PrintString("Magnets");
GLCD_GotoX(offset);
GLCD_PrintInteger(magnets);

GLCD_GotoX(10);
GLCD_GotoLine(6);
GLCD_PrintString("Sdown V");
GLCD_GotoX(offset);
GLCD_PrintDouble(SHUT_DOWN_VOLTAGEx10/10.0,10);

}

if (page==1){
	
	GLCD_GotoX(10);
	GLCD_GotoLine(1);
	GLCD_PrintString("Max speed");
	GLCD_GotoX(offset);
	GLCD_PrintInteger(SCALE_MAX);
	
	GLCD_GotoX(10);
	GLCD_GotoLine(2);
	GLCD_PrintString("Deg/kmh");
	GLCD_GotoX(offset);
	GLCD_PrintDouble(DEG_PER_KMH,1000);
	
	GLCD_GotoX(10);
	GLCD_GotoLine(3);
	GLCD_PrintString("SM steps");
	GLCD_GotoX(offset);
	GLCD_PrintInteger(SM_STEPS);
	
	GLCD_GotoX(10);
	GLCD_GotoLine(4);
	GLCD_PrintString("steps");
	GLCD_GotoX(offset);
	GLCD_PrintInteger(step_interval);
	
		GLCD_GotoX(10);
		GLCD_GotoLine(5);
		GLCD_PrintString("Calibrate");
		GLCD_GotoX(offset);
		
	


}
if (page==2){
	GLCD_GotoX(10);
	GLCD_GotoLine(1);
	GLCD_PrintString("odometer reset");
	
	GLCD_GotoX(10);
	GLCD_GotoLine(2);
	GLCD_PrintString("load defaults");
}

GLCD_Render();


while(1){
		uint8_t current_button=button_monitor();
		if(current_button){
			if(current_button==2)menu_item++;
			if(current_button==3)menu_item--;
			if (current_button==1)
				while(current_button==1){
					current_button=button_monitor();}
				while(1){
					current_button=button_monitor();
					if(current_button==1)break;
					if(current_button==2) PWM_digits++;
					if(current_button==3) PWM_digits--;
				}
		while (button_monitor());
		menu_screen();
		
		_delay_ms(100);
		}
	}
	
}

void main_screen()
{
	if (signalOn==0){
	
	uint8_t offsetX =10;
	uint8_t offsetY=11;
		GLCD_Clear();
		GLCD_DrawRectangle(offsetX,offsetY,26+offsetX,12+offsetY,GLCD_Black);
		GLCD_DrawRectangle(27+offsetX,3+offsetY,28+offsetX,9+offsetY,GLCD_Black);
		
		
		if (voltage>=105)GLCD_FillRectangle(2+offsetX,2+offsetY,6+offsetX,10+offsetY,GLCD_Black);
		if (voltage>=110)GLCD_FillRectangle(8+offsetX,2+offsetY,12+offsetX,10+offsetY,GLCD_Black);
		if (voltage>=115)GLCD_FillRectangle(14+offsetX,2+offsetY,18+offsetX,10+offsetY, GLCD_Black);
		if (voltage>=120)GLCD_FillRectangle(20+offsetX,2+offsetY,24+offsetX,10+offsetY,GLCD_Black);
		
		GLCD_GotoXY(33+offsetX, 4+offsetY);
		GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
		GLCD_PrintDouble(voltage/10.0,10);
		GLCD_PrintString("V");
	GLCD_GotoXY(0, 16);
		//GLCD_PrintInteger(total_rotations);
		
		
	GLCD_SetFont(Arial_Narrow18x32, 18, 32, GLCD_Overwrite);
	GLCD_GotoXY(2+2, 31);
		long tempdist=0;
	
		if (dist>99){tempdist=dist/10;} else{tempdist=100;}
		uint8_t l=0;
		
			while(tempdist){
			tempdist/=10;
			l++;
			}
	
	int zeros=6-l;
	if (dist<100)zeros=4;
		if (zeros>0){
			for (int i=0;i<zeros;i++){
		
				GLCD_PrintString("0");
		
			}	
		}
	GLCD_PrintDouble((double)dist/100.0,10);
	
	
		GLCD_Render();	
		}
		
}


int main(void)
{
	presets();
/*
	main_screen();
	
for(int i=0;i<=contrast;i++){
	GLCD_SetContrast(i);
	_delay_ms(3);
}
*/
	
	
	while(1){
		
	voltage_monitor();
	calc_speed();
	speed_arrow_update();
	signal_monitor();
	if(button_monitor()) menu_screen();
		
		
		
			
		 
		
	}
	
	return 0;
}

void speed_arrow_update(){
				if (step_mode==FULL_STEP_ONE_PHASE)	 newSteps = speedKMH/kmh_per_step;//(12,75 градуса на 10 км/ч)
				if (step_mode==HALF_STEP) newSteps = speedKMH/kmh_per_step;
				int shiftSteps = steps-newSteps;//разница в показаниях спидометра (на сколько нужно сместить стрелку)
				if (shiftSteps>0){dir = 0;}else {dir = 1;}
				if (abs(shiftSteps)){
					arrowMoving=1;
					
					TCCR0|=_BV(CS02)|_BV(CS00)|_BV(WGM01);
					OCR0=step_interval;//интервал между шагами ШД
					TIMSK|=_BV(OCIE0);
				}
				
}

void calc_speed(){
 
			if(speedTimer>1000)
						{
						TIMSK&=~_BV(OCIE2);
						TCNT2=0;
						speedTimer = 0;
						speedDisp=speedTimer;
						speedKMH=0;
						firstMeasure = 0;
						}
			if((refresh)&&(speedDisp))
							{
									
										if (speedDisp>400) speedKMH = 1.0/(T*speedDisp)*3.6*C;			
									
						}
		
			
		if (speedKMH>SCALE_MAX)speedKMH=SCALE_MAX;
		refresh=0;
														/*DEBUG info
														GLCD_SetFont(Arial_Narrow18x32, 18, 32, GLCD_Overwrite);
														GLCD_PrintDouble(speedKMH,10);
														GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
														GLCD_GotoXY(64, 0);
														GLCD_PrintInteger(steps);
														GLCD_GotoXY(64, 10);
														GLCD_PrintInteger(newSteps);
														GLCD_Render();
														*/
														GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
														GLCD_GotoXY(64, 0);
														GLCD_PrintDouble(speedKMH,10);
														GLCD_Render();
		
	
}
void signal_monitor(){
		
		if((!(PINB&_BV(4)))&&(PINB&_BV(3))){
			DrawArrow(0);
			signalOn=1;
			signalCounter=0;
		}
		
		if((!(PINB&_BV(3)))&&(PINB&_BV(4))){
			DrawArrow(1);
			signalOn=1;
			signalCounter=0;
		}
		
		if((!(PINB&_BV(4)))&&(!(PINB&_BV(3)))){
			DrawSkull();
			signalOn=1;
			signalCounter=0;
		}
		
		
		if (signalOn){
			if((PINB&_BV(3))&&(PINB&_BV(4))){
				GLCD_Clear();
				GLCD_Render();
				TIMSK|=_BV(TOIE1);// Зачем? А-а, понял. Если был включен поворот (стрелка), а в данный момент повороты не горят, запускается таймер.
				//Это нужно, чтобы понять, это интервал между миганием поворотников, или сигнал поворота уже выключен.
			}
			
			if (signalCounter>300)
			{
				signalOn=0;
				signalCounter=0;
				TIMSK&=~_BV(TOIE1);
				main_screen();
			}
		}
		
}
void voltage_monitor(){

	newVoltage = (readADC(4,10)/102.3)*AREF*DEVIDER;
	
	if (newVoltage<SHUT_DOWN_VOLTAGEx10){
		cli();
		TCCR1A=0;
		TCCR1B=0;
		
		PORTA|=_BV(3); //ENABLE 1
		PORTA|=_BV(0); //ENABLE 2
		PORTA|=_BV(2); //PHASE 1
		PORTA|=_BV(1); //PHASE 2
		
		unsigned long x= 0;
		x = total_rotations;
		x=(x<<24);
		x=(x>>24);
		//debug1=x;
		eeprom_write_byte(23,x);
		x = total_rotations;
		x=(x<<16);
		x=(x>>24);
		//	debug2=x;
		eeprom_write_byte(22,x);
		x = total_rotations;
		x=(x<<8);
		x=(x>>24);
		//	debug3=x;
		eeprom_write_byte(21,x);
		//x = total_rotations;
		//x=x>>24;
		//eeprom_write_byte(20,x);
		
		while (newVoltage<SHUT_DOWN_VOLTAGEx10){
			newVoltage = (readADC(4,10)/102.3)*AREF*DEVIDER;
		}
		sei();
	}
	
	
	if (newVoltage!=voltage)
	{
		voltage=newVoltage;
		main_screen();
	}
	
	newDist=(round(total_rotations)*C)/10.0;
	if (newDist!=dist)
	{
		dist=newDist;
		main_screen();
	}
}

uint8_t button_monitor(){
if ((PINB&_BV(5))&&(PINB&_BV(6))&&(PINB&_BV(7))){
	btn_pressed=0;
}

if((!(PINB&_BV(5)))&&(btn_pressed==0)){
	_delay_ms(50);
	if(!(PINB&_BV(5))){
		btn_pressed=1;
	}
}

if((!(PINB&_BV(6)))&&(btn_pressed==0)){
	_delay_ms(50);
	if(!(PINB&_BV(6))){
	btn_pressed=2;
	}
	
}
if((!(PINB&_BV(7)))&&(btn_pressed==0)){
	_delay_ms(50);
	if(!(PINB&_BV(7))){
	btn_pressed=3;
	}
	
}
return btn_pressed;
}
void arrow_calibration(){
	int calibration_steps = 0;
	if (step_mode==FULL_STEP_ONE_PHASE){
		kmh_per_step=(360.0/SM_STEPS)/DEG_PER_KMH;
		calibration_steps=SM_STEPS;
	}
	if (step_mode==HALF_STEP){
		kmh_per_step=(180.0/SM_STEPS)/DEG_PER_KMH;
		calibration_steps=SM_STEPS*2;
	}
/*
	dir=1;
	for (int i=0;i<=calibration_steps/2;i++){
		step(step_mode);
		_delay_ms(4);
	}
	dir=0;
	for (int i=0;i<=calibration_steps;i++){
		step(step_mode);
		_delay_ms(4);
	}
	*/

steps=0;
phase=0;
	newSteps = calibration_steps/4;
	dir = 1;
	
	
		arrowMoving=1;
		
		TCCR0|=_BV(CS02)|_BV(CS00)|_BV(WGM01);
		OCR0=step_interval;//интервал между шагами ШД
		TIMSK|=_BV(OCIE0);
	

while (arrowMoving)
{
}
	_delay_ms(150);

phase=0;
newSteps=0;
steps = calibration_steps;
dir = 0;

arrowMoving=1;

TCCR0|=_BV(CS02)|_BV(CS00)|_BV(WGM01);
OCR0=step_interval;//интервал между шагами ШД
TIMSK|=_BV(OCIE0);

while (arrowMoving)
{
}
steps=0;
newSteps=0;
phase=0;



}
	
void DrawArrow (char dir){
	if (dir){
		GLCD_Clear();
		GLCD_GotoXY(22, 7);
		GLCD_DrawBitmap(arrowRight,92,55,GLCD_Black);
		GLCD_Render();
		}else{
		GLCD_Clear();
		GLCD_GotoXY(22, 7);
		GLCD_DrawBitmap(arrowLeft,92,55,GLCD_Black);
		GLCD_Render();	
		}
}

void DrawSkull (void)
{
GLCD_Clear();
GLCD_GotoXY(21+6, 7);
GLCD_DrawBitmap(skull,86,52,GLCD_Black);
GLCD_Render();
}
	
	
int readADC(unsigned char mux, unsigned char cycles)
{
	ADMUX = mux;
	int tmp = 0;
	for (int i=0;i<cycles;i++)
	{
		ADCSRA |= (1<<ADSC);
		while((ADCSRA &(1<<ADSC)));
		tmp += ADCW;
	}
	return tmp/cycles;
}	
	
	
	
	void eeprom_write_byte(unsigned int uiAddress, unsigned char ucData)
{
/* 
Wait for completion of previous write
 */
while(EECR & (1<<EEWE))
;
/* Set up address and data registers */
EEAR = uiAddress;
EEDR = ucData;
/* 
Write logical one to EEMWE */
cli();
EECR |= (1<<EEMWE);
/* Start eeprom write by setting EEWE */
EECR |= (1<<EEWE);
sei();
}


unsigned char eeprom_read_byte(unsigned int uiAddress)
{
	/* Wait for completion of previous write */
	while(EECR & (1<<EEWE))
	;
	/* Set up address register */
	cli();
	EEAR = uiAddress;
	/*
	Start eeprom read by writing EERE */
	EECR |= (1<<EERE);
	sei();
	/* Return data from data register */
	return EEDR;
}

