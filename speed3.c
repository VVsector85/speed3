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
#include <avr/eeprom.h>
volatile int speedTimer = 0;
volatile int speedTimerRecent = 0;
volatile char speedRefresh = 1;			//speedometer value refresh flag
volatile uint32_t totalRotations = 0;	//rotation counter
double timePerTic = 0;		// duration of Counter1 tic in seconds		
double circLength = 0;//wheel circumference
volatile double speedKmh = 0;			//speed km/h
volatile uint8_t arrowMoving = 0;
int newSteps = 0;
uint8_t signalOn = 0;//if turn or hazard lights on, = 1
uint8_t firstMeasure = 0;

uint8_t lcdContrast = 250;
uint8_t magnets = 6; //number of magnets on the wheel
double ratio = 1.0; //gear ratio (needed if magnets are not on the wheel)
double wheelDiameter = 0.70;			//wheel diameter in meters
double degreesPerKmh = 1.275;	//degrees per km/h
uint16_t pwmArrow = 1024;// of 1024
uint16_t pwmDial = 1024;// of 1024

uint8_t scaleMax	= 190;		//speed max value
uint8_t shutDownVoltageX10 = 80; //if voltage is below this value totalRotations is being saved to EEPROM
uint8_t stepInterval = 150; //interval between steps (Affects Stepper Motor Rotation Speed)
uint8_t smSteps =	96;		//stepper motor steps

unsigned int signalCounter = 0;//counter of turn lights interval
unsigned long distance = 0;
unsigned long newDistance = 0;
double kmhPerStep = 0;
volatile signed int phase = 0;
uint8_t dir = 0;
volatile int steps = 0;
char btnPressed = 0;
int voltage = 0;
int newVoltage = 0;

#define pi 3.141592653		//pi
#define PRESCALER 256		//prescaler for Timer2
#define TIC  20				//amount of Timer2 tics (F_CPU 16Mhz è presc=256 1 tic = 16 us). Defines the minimum period for counting time between Hall sensor triggering
#define AREF 2.5			//reference voltage
#define DEVIDER 6			//divider (for battery voltage measurement)



#define FULL_STEP_ONE_PHASE 0
#define HALF_STEP 1

uint8_t stepMode = HALF_STEP;


int read_ADC(unsigned char mux, unsigned char cycles);
void step(char mode);
void calculate_speed();
void DrawArrow (char dir);
void DrawSkull ();
void main_screen();
void speed_arrow_update();
void data_monitor();
void signal_monitor();
uint8_t button_monitor();
void menu_screen();
void arrow_calibration();

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


circLength = wheelDiameter * pi/magnets;			//circumferential length between the magnets
timePerTic = 1.0/(F_CPU/PRESCALER);			//counter tic time interval in seconds (16 us, presc=256)

	
DDRA|=_BV(3); //ENABLE 1
DDRA|=_BV(0); //ENABLE 2
DDRA|=_BV(2); //PHASE 1
DDRA|=_BV(1); //PHASE 2
DDRD|=_BV(4);//PWM DIAL LIGHT
DDRD|=_BV(5);//PWM ARROW LIGHT


PORTA|=_BV(3);//ENABLE 1 high (disabled)
PORTA|=_BV(0);//ENABLE 2 high (disabled)

PORTB|=_BV(5);//internal pull-up for external buttons
PORTB|=_BV(6);
PORTB|=_BV(7);
	
		//=======================ADC
		ADCSRA |= _BV(ADEN);
		//=======================
		ADCSRA |= _BV(ADPS0);		//
		ADCSRA |= _BV(ADPS1);		// ADC prescaler 128
		ADCSRA |= _BV(ADPS2);		//
		//=======================
	
	
	//================= reading data from EEPROM
	
	uint8_t firstEepRead;
	uint16_t eepAddress = 8;
	uint8_t eepAddrShift = 8;
	firstEepRead = eeprom_read_byte((uint8_t*)eepAddress);//if the device is starting for the first time the default values have to be written to EEPROM
	if (firstEepRead){
		eeprom_write_byte((uint8_t*)eepAddress,0);
	    eeprom_write_word((uint16_t*)(eepAddress+=eepAddrShift),pwmArrow);
		eeprom_write_word((uint16_t*)(eepAddress+=eepAddrShift),pwmDial);
		eeprom_write_byte((uint8_t*)(eepAddress+=eepAddrShift),scaleMax);
		eeprom_write_byte((uint8_t*)(eepAddress+=eepAddrShift),shutDownVoltageX10);
		eeprom_write_byte((uint8_t*)(eepAddress+=eepAddrShift),stepInterval);
		eeprom_write_byte((uint8_t*)(eepAddress+=eepAddrShift),smSteps);
		eeprom_write_byte((uint8_t*)(eepAddress+=eepAddrShift),lcdContrast);
		eeprom_write_byte((uint8_t*)(eepAddress+=eepAddrShift),magnets);
		eeprom_write_byte((uint8_t*)(eepAddress+=eepAddrShift),stepMode);
		eeprom_write_float((float*)(eepAddress+=eepAddrShift),wheelDiameter);
		eeprom_write_float((float*)(eepAddress+=eepAddrShift),ratio);
		eeprom_write_float((float*)(eepAddress+=eepAddrShift),degreesPerKmh);
		eeprom_write_dword((uint32_t*)(eepAddress+=eepAddrShift),totalRotations);
		}
		eepAddress = 8;
		
		pwmArrow = eeprom_read_word((uint16_t*)(eepAddress+=eepAddrShift));
		pwmDial = eeprom_read_word((uint16_t*)(eepAddress+=eepAddrShift));
		scaleMax = eeprom_read_byte((uint8_t*)(eepAddress+=eepAddrShift));
		shutDownVoltageX10 = eeprom_read_byte((uint8_t*)(eepAddress+=eepAddrShift));
		stepInterval = eeprom_read_byte((uint8_t*)(eepAddress+=eepAddrShift));
		smSteps = eeprom_read_byte((uint8_t*)(eepAddress+=eepAddrShift));
		lcdContrast = eeprom_read_byte((uint8_t*)(eepAddress+=eepAddrShift));
		magnets = eeprom_read_byte((uint8_t*)(eepAddress+=eepAddrShift));
		stepMode = eeprom_read_byte((uint8_t*)(eepAddress+=eepAddrShift));
		wheelDiameter = eeprom_read_float((float*)(eepAddress+=eepAddrShift));
		ratio = eeprom_read_float((float*)(eepAddress+=eepAddrShift));
		degreesPerKmh = eeprom_read_float((float*)(eepAddress+=eepAddrShift));
		totalRotations = eeprom_read_dword((uint32_t*)(eepAddress+=eepAddrShift));
	
		

	//dial and arrow light PWM===============
	TCCR1A = _BV(WGM10)|_BV(WGM11)|_BV(COM1B1)|_BV(COM1A1);
	TCCR1B = _BV(WGM12)|_BV(CS10)|_BV(CS11);
	OCR1A = pwmArrow;
	OCR1B = pwmDial;
	//============================
	if (stepMode==FULL_STEP_ONE_PHASE){
		kmhPerStep=(360.0/smSteps)/degreesPerKmh;
		}
	if (stepMode==HALF_STEP){
		kmhPerStep=(180.0/smSteps)/degreesPerKmh;
		}
//arrow_calibration();

MCUCR|= _BV(ISC11); // External falling edge interrupt INT1
GICR|=_BV(INT1); // External Interrupt Enable INT1

TCCR2|=_BV(CS21)|_BV(CS22)|_BV(WGM21);
OCR2 = TIC; //upper limit of Timer2

//display initialization
GLCD_Setup();
GLCD_Clear();
GLCD_SetContrast(lcdContrast);
GLCD_Render();
sei();
}

ISR( TIMER0_COMP_vect ){

step(stepMode);		
	if (steps == newSteps){
	arrowMoving = 0;
	TCCR0=0;
	OCR0=0;
	TIMSK&=~_BV(OCIE0);
	}
}


void step(char mode){
	char tempPort = 0;
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
		if (phase < 0) phase = 3;
		if (phase > 3) phase = 0;
	}
	if(mode == HALF_STEP){
		if (phase < 0) phase = 7;
		if (phase > 7) phase = 0;
	} 
	tempPhase=phase;

	tempPort=PORTA;
	tempPort&=~_BV(0);
	tempPort&=~_BV(1);
	tempPort&=~_BV(2);
	tempPort&=~_BV(3);
	if(mode == FULL_STEP_ONE_PHASE) tempPort|=phase_arr_full_step_1phase[tempPhase];
	if(mode == HALF_STEP) tempPort|=phase_arr_half_step[tempPhase];

	PORTA = tempPort;
	
}


ISR( TIMER2_COMP_vect ){
	speedTimer++; //speedTimer increments each period timePerTic*TIC
	}
ISR (TIMER1_OVF_vect){
	if (signalOn) signalCounter++;
}
ISR(INT1_vect){
//interrupt occurs when Hall sensor is triggered
if (firstMeasure==0)
	{
		TIMSK|=_BV(OCIE2);
		TCNT2 = 0;
		firstMeasure = 1;
		//first triggering of the sensor starts TIMER2
	}
else
	{
		speedTimerRecent = (speedTimer*TIC)+TCNT2;
		TCNT2 = 0;	
		speedTimer = 0;
		speedRefresh = 1;
		totalRotations++;
	}
}

void menu_screen(){
uint8_t offset = 75;	
static int8_t menuItem;
static int8_t page;
if (page < 0) page = 0;
if (menuItem > 5){page++;menuItem=0;}
if (menuItem < 0){page--;menuItem=5;}
if ((page == 2)&&(menuItem > 1)){
	page = 0;
	menuItem = 0;
}
GLCD_Clear();
GLCD_FillRectangle(0,0+menuItem*8-1+8,5,7+menuItem*8+8,GLCD_Black);
GLCD_FillRectangle(122,0+menuItem*8-1+8,127,7+menuItem*8+8,GLCD_Black);
GLCD_DrawLine(0,menuItem*8-2+8,127,menuItem*8-2+8,GLCD_Black);
GLCD_DrawLine(0,menuItem*8+8+8,127,menuItem*8+8+8,GLCD_Black);

	GLCD_SetFont(Font5x8, 5, 8, GLCD_Merge);
if(page==0){
GLCD_GotoX(10);	
GLCD_GotoLine(1);
GLCD_PrintString("Dig_PWM");
GLCD_GotoX(offset);
GLCD_PrintInteger(pwmDial);

GLCD_GotoX(10);
GLCD_GotoLine(2);
GLCD_PrintString("Arr_PWM");
GLCD_GotoX(offset);
GLCD_PrintInteger(pwmArrow);

GLCD_GotoX(10);
GLCD_GotoLine(3);
GLCD_PrintString("Weel D");	
GLCD_GotoX(offset);
GLCD_PrintDouble(wheelDiameter,100);
	
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
GLCD_PrintDouble(shutDownVoltageX10/10.0,10);

}

if (page==1){
	
	GLCD_GotoX(10);
	GLCD_GotoLine(1);
	GLCD_PrintString("Max speed");
	GLCD_GotoX(offset);
	GLCD_PrintInteger(scaleMax);
	
	GLCD_GotoX(10);
	GLCD_GotoLine(2);
	GLCD_PrintString("Deg/kmh");
	GLCD_GotoX(offset);
	GLCD_PrintDouble(degreesPerKmh,1000);
	
	GLCD_GotoX(10);
	GLCD_GotoLine(3);
	GLCD_PrintString("SM steps");
	GLCD_GotoX(offset);
	GLCD_PrintInteger(smSteps);
	
	GLCD_GotoX(10);
	GLCD_GotoLine(4);
	GLCD_PrintString("steps");
	GLCD_GotoX(offset);
	GLCD_PrintInteger(stepInterval);
	
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
		uint8_t currentButton=button_monitor();
		if(currentButton){
			if(currentButton == 2)	menuItem++;
					
			if(currentButton == 3)	menuItem--;
							
		while (button_monitor());
		menu_screen();
		
		}
	}
	
}

void main_screen()
{
	if (!signalOn){
	
	uint8_t offsetX = 10;
	uint8_t offsetY = 11;
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
		
		
	GLCD_SetFont(Arial_Narrow18x32, 18, 32, GLCD_Overwrite);
	GLCD_GotoXY(2+2, 31);
		long tempDistance=0;
	
		if (distance>99){tempDistance=distance/10;} else{tempDistance=100;}
		uint8_t l=0;
		
			while(tempDistance){
			tempDistance/=10;
			l++;
			}
	
	int zeros = 6-l;
	if (distance < 100)zeros = 4;
		if (zeros > 0){
			for (int i=0;i<zeros;i++){
		
				GLCD_PrintString("0");
		
			}	
		}
	GLCD_PrintDouble((double)distance/100.0,10);
	
	GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
	GLCD_GotoXY(64, 0);
	GLCD_PrintDouble(speedKmh,10);
	GLCD_Render();
	
	
		GLCD_Render();	
		}
		
}


int main(void)
{
	presets();
	
	while(1){
		data_monitor();
		calculate_speed();
		speed_arrow_update();
		signal_monitor();
		if(button_monitor()) menu_screen();
			}
	return 0;
}

void speed_arrow_update(){
				if (stepMode == FULL_STEP_ONE_PHASE)	 newSteps = speedKmh/kmhPerStep;//(12,75 degrees per 10 km/h)
				if (stepMode == HALF_STEP) newSteps = speedKmh/kmhPerStep;
				int shiftSteps = steps - newSteps;//difference in speedometer readings (how much the arrow should be shifted)
				if (shiftSteps > 0){dir = 0;}else {dir = 1;}
				if (abs(shiftSteps)){
					arrowMoving = 1;
					
					TCCR0|=_BV(CS02)|_BV(CS00)|_BV(WGM01);
					OCR0 = stepInterval;//interval between steps (Affects Stepper Motor Rotation Speed)
					TIMSK|=_BV(OCIE0);
				
				main_screen();
				
				}
				
}

void calculate_speed(){
 
			if(speedTimer>1000){
						TIMSK&=~_BV(OCIE2);  //if Hall sensor was not triggered for too long (0,32s) it means that vehicle does not move
						TCNT2 = 0;
						speedTimer = 0;
						speedTimerRecent = 0;//speedTimer;//?
						speedKmh = 0;
						firstMeasure = 0;
						}
			if((speedRefresh)&&(speedTimerRecent)){
						if (speedTimerRecent>400) speedKmh = 1.0/(timePerTic*speedTimerRecent)*3.6*circLength;			
						}
		
			
		if (speedKmh>scaleMax)speedKmh=scaleMax;
		speedRefresh=0;
														
														
		}
void signal_monitor(){
		
		if((!(PINB&_BV(4)))&&(PINB&_BV(3))){
			DrawArrow(0);
			signalOn = 1;
			signalCounter = 0;
		}
		
		if((!(PINB&_BV(3)))&&(PINB&_BV(4))){
			DrawArrow(1);
			signalOn = 1;
			signalCounter = 0;
		}
		
		if((!(PINB&_BV(4)))&&(!(PINB&_BV(3)))){
			DrawSkull();
			signalOn = 1;
			signalCounter = 0;
		}
		
		
		if (signalOn){
			if((PINB&_BV(3))&&(PINB&_BV(4))){
				GLCD_Clear();
				GLCD_Render();
				TIMSK|=_BV(TOIE1);// If the turn signal (arrow) was switched on, and at the moment the turn signals are not lit, the timer is started.
				//This is to see if this is the interval between the blinking of the turn signals, or if the turn signal is already off.
			}
			
			if (signalCounter > 300) //if interval is longer then the interval between the blinks - stop displaying turn/hazard sign
			{
				signalOn = 0;
				signalCounter = 0;
				TIMSK&=~_BV(TOIE1);
				main_screen();
			}
		}
		
}
void data_monitor(){

	newVoltage = (read_ADC(4,10)/102.3)*AREF*DEVIDER;
	
	if (newVoltage<shutDownVoltageX10){  
		cli();
		TCCR1A = 0;
		TCCR1B = 0;
											//shutting down all the power consumers
		PORTA|=_BV(3); //ENABLE 1
		PORTA|=_BV(0); //ENABLE 2
		PORTA|=_BV(2); //PHASE 1
		PORTA|=_BV(1); //PHASE 2
		
		eeprom_write_dword((uint32_t*)112,totalRotations);
		
		while (newVoltage<shutDownVoltageX10){
			newVoltage = (read_ADC(4,10)/102.3)*AREF*DEVIDER;
		}
		main();
	}
	
	
	if (newVoltage!=voltage) //if voltage value changes - refresh data on the screen
	{
		voltage=newVoltage;
		main_screen();
	}
	
	newDistance=(round(totalRotations)*circLength)/10.0;
	if (newDistance!=distance) //when the distance value changes by 100 meters - update the data on the screen
	{
		distance=newDistance;
		main_screen();
	}
}

uint8_t button_monitor(){
if ((PINB&_BV(5))&&(PINB&_BV(6))&&(PINB&_BV(7))){
	btnPressed = 0;
	return 0;
}

if((!(PINB&_BV(5)))&&(!btnPressed)){
	_delay_ms(50);
	if(!(PINB&_BV(5))){
		btnPressed = 1;
	}
}

if((!(PINB&_BV(6)))&&(!btnPressed)){
	_delay_ms(50);
	if(!(PINB&_BV(6))){
	btnPressed=2;
	}
	
}
if((!(PINB&_BV(7)))&&(!btnPressed)){
	_delay_ms(50);
	if(!(PINB&_BV(7))){
	btnPressed=3;
	}
	
}
return btnPressed;
}
void arrow_calibration(){
	int calibrationSteps = 0;
	if (stepMode==FULL_STEP_ONE_PHASE){
		calibrationSteps=smSteps;
	}
	if (stepMode==HALF_STEP){
		calibrationSteps=smSteps*2;
	}


steps = 0;
phase = 0;
	
	newSteps = calibrationSteps/4; //turning arrow 90 clockwise
	dir = 1;
	
	
		arrowMoving=1;
		
		TCCR0|=_BV(CS02)|_BV(CS00)|_BV(WGM01);
		OCR0=stepInterval;
		TIMSK|=_BV(OCIE0);
	

while (arrowMoving);

	_delay_ms(50);

phase=0;
newSteps=0;
steps = calibrationSteps;	 //turning arrow 360 clockwise to be sure that arrow is in the zero position
dir = 0;

arrowMoving=1;

TCCR0|=_BV(CS02)|_BV(CS00)|_BV(WGM01);
OCR0 = stepInterval;
TIMSK|=_BV(OCIE0);

while (arrowMoving);

steps = 0;
newSteps = 0;
phase = 0;
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
	
	
int read_ADC(unsigned char mux, unsigned char cycles)
{
	ADMUX = mux;
	int tmp = 0;
	for (int i = 0;i<cycles;i++)
	{
		ADCSRA |= (1<<ADSC);
		while((ADCSRA &(1<<ADSC)));
		tmp += ADCW;
	}
	return tmp/cycles;
}	
	
	
	