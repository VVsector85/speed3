#include "SSD1306.h"
#include "Skull.h"
#include "ArrowRight.h"
#include "ArrowLeft.h"
#include "Font5x8.h"
#include "ArialNarrowDig18x32.h"
#include "ArialDig12x17.h"
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>


#define pi 3.141592653		//pi
#define TIMER2_PRESCALER 256
#define TIC  20				//amount of Timer2 tics  (16Mhz è presc=256 1 tic = 16 us). Defines the minimum period for counting time between Hall sensor triggering
#define AREF 2.5			//reference voltage
#define DEVIDER 6			//divider (for battery voltage measurement)
#define FULL_STEP 0
#define HALF_STEP 1
#define EEPROM_START_ADDRESS 8
#define EEP_ODOMETER_START_ADDRESS 128
#define EEPROM_ADDRESS_SHIFT 4
#define EEP_WRITE 1
#define EEP_READ 0
#define EEP_ODOMETER_WRITE 3
#define EEP_ODOMETER_READ 2
#define ODOMETER_EEP_CELLS 50
#define BOUNCE_DELAY 1

volatile int speedTimer = 0;
volatile int speedTimerRecent = 0;
volatile uint8_t speedRefresh = 1;			//speedometer value refresh flag
volatile uint32_t totalRotations = 0;	//rotation counter
volatile uint8_t arrowMoving = 0;
volatile int8_t phase = 0;
volatile int16_t steps = 0;

double timePerTic = 0;		// duration of Counter1 tic in seconds
double circLength = 0;//wheel circumference
double speedKmh = 0;

uint16_t newSteps = 0;
uint8_t signalOn = 0;//if turn or hazard lights on, = 1
uint8_t firstMeasure = 0;
uint8_t debugMode = 0;
uint8_t lcdContrast = 250;
uint8_t magnetsOnWheel = 6;
double gearRatio = 1.0; //needed if magnets are not on the wheel
double wheelDiameter = 0.70;			//wheel diameter in meters
double degreesPerKmh = 1.275;	//degrees per km/h
uint16_t pwmArrow = 1024;// of 1024
uint16_t pwmDial = 1024;// of 1024
uint8_t scaleMax	= 190;		//speed max value
uint8_t stepInterval = 150; //interval between steps (Affects Stepper Motor Rotation Speed)
uint16_t smSteps =	96;		//stepper motor steps

uint8_t dir = 0;
uint16_t signalCounter = 0;//counter of turn lights interval
uint32_t distance = 0;
uint32_t newDistance = 0;
double kmhPerStep = 0;


int16_t voltage = 0;
int16_t newVoltage = 0;
uint8_t stepMode = HALF_STEP;
uint8_t odometerCurrentAddress = 0;

int main(void);
int read_ADC(uint8_t mux, uint8_t cycles);
void step(uint8_t mode);
void calculate_speed();
void draw_arrow (uint8_t arrowDir);
void draw_skull ();
void main_screen();
void speed_arrow_update();
void data_monitor();
void signal_monitor();
uint8_t button_monitor();
void menu_screen();
void arrow_calibration();
void eep_operations (uint16_t eepStartAddress, uint8_t eepAddrShift, uint8_t writeOrRead);
uint16_t set_value (uint16_t maxValue, uint16_t minValue, uint16_t currValue, uint8_t tens, const char *text);


const unsigned char phaseArrayFullStep [] = {
	0b00000111,
	0b00001110,
	0b00000001,
	0b00001000
};								//ONE PHASE

const unsigned char phaseArrayHalfStep [] = {
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


circLength = wheelDiameter * pi/magnetsOnWheel;			//circumferential length between the magnets
timePerTic = 1.0/(F_CPU/TIMER2_PRESCALER);			//counter tic time interval in seconds (16 us, presc=256)


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

	firstEepRead = eeprom_read_byte((uint8_t*)EEPROM_START_ADDRESS);//if the device is starting for the first time the default values have to be written to EEPROM
	if (firstEepRead){
		eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
			for (uint8_t i = 0;i<=ODOMETER_EEP_CELLS;i++){
			eep_operations(EEP_ODOMETER_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_ODOMETER_WRITE);
			}
		}
	eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_READ);
	if (odometerCurrentAddress>ODOMETER_EEP_CELLS)odometerCurrentAddress = 0;
	eep_operations(EEP_ODOMETER_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_ODOMETER_READ);



TCCR2|=_BV(CS21)|_BV(CS22)|_BV(WGM21);
OCR2 = TIC; //upper limit of Timer2

	//dial and arrow light PWM===============
	TCCR1A = _BV(WGM10)|_BV(WGM11)|_BV(COM1B1)|_BV(COM1A1);
	TCCR1B = _BV(WGM12)|_BV(CS10)|_BV(CS11);
	OCR1A = pwmArrow;
	OCR1B = pwmDial;
	//============================
	if (stepMode==FULL_STEP){
		kmhPerStep=(360.0/smSteps)/degreesPerKmh;
		}
	if (stepMode==HALF_STEP){
		kmhPerStep=(180.0/smSteps)/degreesPerKmh;
		}

//display initialization
GLCD_Setup();
GLCD_Clear();
GLCD_SetContrast(lcdContrast);
GLCD_Render();
sei();
//if (!arrowCalibrated)
//arrow_calibration();
MCUCR|= _BV(ISC11); //External falling edge interrupt INT1
GICR|=_BV(INT1); //External Interrupt Enable INT1
}

ISR( TIMER0_COMP_vect ){
	step(stepMode);
	if (steps == newSteps){
	arrowMoving = 0;
	TCCR0 = 0;
	OCR0 = 0;
	TIMSK&=~_BV(OCIE0);
	}
}


void step(uint8_t mode){
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
	if(mode == FULL_STEP){
		if (phase < 0) phase = 3;
		if (phase > 3) phase = 0;
	}
	if(mode == HALF_STEP){
		if (phase < 0) phase = 7;
		if (phase > 7) phase = 0;
	}
	tempPhase = phase;

	tempPort=PORTA;
	tempPort&=~_BV(0);
	tempPort&=~_BV(1);
	tempPort&=~_BV(2);
	tempPort&=~_BV(3);
	if(mode == FULL_STEP) tempPort|=phaseArrayFullStep[tempPhase];
	if(mode == HALF_STEP) tempPort|=phaseArrayHalfStep[tempPhase];

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
if (firstMeasure)
	{
		speedTimerRecent = (speedTimer*TIC) + TCNT2;
		TCNT2 = 0;
		speedTimer = 0;
		speedRefresh = 1;
		totalRotations++;
	}
else
	{
		//first triggering of the sensor starts TIMER2
		TIMSK|=_BV(OCIE2);
		TCNT2 = 0;
		firstMeasure = 1;
	}
}


void menu_screen(){
	while (button_monitor());
uint8_t offset = 75;
static int8_t menuItem;
static int8_t page;


if (menuItem > 5){page++;menuItem=0;}
if (menuItem < 0){page--;menuItem=5;}
if ((page == 2)&&(menuItem > 3)){
	page = 0;
	menuItem = 0;
}
if (page<0){page =2;menuItem=3;}
GLCD_Clear();

	GLCD_SetFont(Font5x8, 5, 8, GLCD_Merge);
if(page==0){
//item 0
GLCD_GotoX(10);
GLCD_GotoLine(1);
GLCD_PrintString("Dial PWM");
GLCD_GotoX(offset);
GLCD_PrintInteger(pwmDial);
//item 1
GLCD_GotoX(10);
GLCD_GotoLine(2);
GLCD_PrintString("Arrow PWM");
GLCD_GotoX(offset);
GLCD_PrintInteger(pwmArrow);
//item 2
GLCD_GotoX(10);
GLCD_GotoLine(3);
GLCD_PrintString("Wheel D");
GLCD_GotoX(offset);
GLCD_PrintDouble(wheelDiameter,1000);
//item 3
GLCD_GotoX(10);
GLCD_GotoLine(4);
GLCD_PrintString("Ratio");
GLCD_GotoX(offset);
GLCD_PrintDouble(gearRatio,1000);
//item 4
GLCD_GotoX(10);
GLCD_GotoLine(5);
GLCD_PrintString("Magnets");
GLCD_GotoX(offset);
GLCD_PrintInteger(magnetsOnWheel);
//item 5
GLCD_GotoX(10);
GLCD_GotoLine(6);
GLCD_PrintString("Step mode");
GLCD_GotoX(offset);
GLCD_PrintInteger(stepMode);

}

if (page==1){
//item 6
	GLCD_GotoX(10);
	GLCD_GotoLine(1);
	GLCD_PrintString("Max speed");
	GLCD_GotoX(offset);
	GLCD_PrintInteger(scaleMax);
//item 7
	GLCD_GotoX(10);
	GLCD_GotoLine(2);
	GLCD_PrintString("Deg/kmh");
	GLCD_GotoX(offset);
	GLCD_PrintDouble(degreesPerKmh,1000);
//item 8
	GLCD_GotoX(10);
	GLCD_GotoLine(3);
	GLCD_PrintString("SM steps");
	GLCD_GotoX(offset);
	GLCD_PrintInteger(smSteps);
//item 9
	GLCD_GotoX(10);
	GLCD_GotoLine(4);
	GLCD_PrintString("Step interval");
	GLCD_GotoX(offset);
	GLCD_PrintInteger(stepInterval);
//item 10
	GLCD_GotoX(10);
	GLCD_GotoLine(5);
	GLCD_PrintString("LCD contrast");
	GLCD_GotoX(offset);
	GLCD_PrintInteger(lcdContrast);
//item 11
	GLCD_GotoX(10);
	GLCD_GotoLine(6);
	GLCD_PrintString("Debug mode");
	GLCD_GotoX(offset);
	


}
if (page==2){
//item 12
	GLCD_GotoX(10);
	GLCD_GotoLine(1);
	GLCD_PrintString("Calibrate arrow");
	GLCD_GotoX(offset);
//item 13
	GLCD_GotoX(10);
	GLCD_GotoLine(2);
	GLCD_PrintString("Odometer reset");
//item 14
	GLCD_GotoX(10);
	GLCD_GotoLine(3);
	GLCD_PrintString("Load defaults");
//item 15
	GLCD_GotoX(10);
	GLCD_GotoLine(4);
	GLCD_PrintString("Exit");
}

GLCD_InvertRect(0,menuItem*8+7,127,menuItem*8+15);
GLCD_Render();

uint8_t currentButton = 0;
uint16_t newValue = 0;
while(1){
		 currentButton = button_monitor();
		if(currentButton){
			if (currentButton == 1){
								switch(menuItem+6*page)		{
											case 0:
											{
												newValue = set_value(1024,0,pwmDial,0,"Dial light PWM");
												if (newValue!=pwmDial){
													pwmDial = newValue;
													eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
												}
												break;
											}
											case 1:
											{
												newValue = set_value(1024,0,pwmArrow,0,"Arrow light PWM");
												if (newValue!=pwmArrow){
													pwmArrow = newValue;
													eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
												}
												break;
											}
											case 2:
											{
												newValue = set_value(1500,150,wheelDiameter*1000,3,"Wheel Diameter (m)");
												if (newValue!=wheelDiameter*1000){
													wheelDiameter = newValue/1000.0;
													eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
												}
												break;
											}
											case 3:
											{
												newValue = set_value(1000,150,gearRatio*1000,3,"Gear ratio");
												if (newValue!=gearRatio*1000){
													gearRatio = newValue/1000.0;
													eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
												}
												break;
											}
											case 4:
											{
												newValue = set_value(16,1,magnetsOnWheel,0,"Magnets on the wheel");
												if (newValue!=magnetsOnWheel){
													magnetsOnWheel = newValue;
													eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
												}
												break;
											}
											case 5:
											{
												newValue = set_value(1,0,stepMode,0,"0-full, 1-half");
												if (newValue!=stepMode){
													stepMode = newValue;
													eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
												}
												break;
											}
											case 6:
											{
												newValue = set_value(400,40,scaleMax,0,"Maximum speed");
												if (newValue!=scaleMax){
													scaleMax = newValue;
													eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
												}
												break;
											}
											case 7:
											{
												newValue = set_value(4000,100,degreesPerKmh*1000,3,"Degrees per km/h");
												if (newValue!=degreesPerKmh*1000){
													degreesPerKmh = newValue/1000.0;
													eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
												}
												break;
											}
											case 8:
											{
												newValue = set_value(400,16,smSteps,0,"Stepper motor steps");
												if (newValue!=smSteps){
													smSteps = newValue;
													eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
												}
												break;
											}
											case 9:
											{
												newValue = set_value(500,50,stepInterval,0,"Time between steps");
												if (newValue!=stepInterval){
													stepInterval = newValue;
													eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
												}
												break;
											}
											case 10:
											{
												newValue = set_value(255,10,lcdContrast,0,"LCD contrast");
												if (newValue!=lcdContrast){
													lcdContrast = newValue;
													GLCD_SetContrast(lcdContrast);
													eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
												}
												break;
											}
											case 11:
											{
												debugMode = 1;
												while (button_monitor());
												main();
												break;
											}
											case 12:
											{
												arrow_calibration();
												break;
											}
											case 13:
											{
											//odometer reset
												totalRotations = 0;
												for (uint8_t i = 0;i<=ODOMETER_EEP_CELLS;i++){
													eep_operations(EEP_ODOMETER_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_ODOMETER_WRITE);
												}
												break;
											}
											case 14:
											{
											//load defaults
											break;
											}
											case 15:
											{
												//exit menu
												while (button_monitor());
												main();
											}
										}
//
			}

			else if(currentButton == 2)	menuItem++;

			else if(currentButton == 3)	menuItem--;

		//while (button_monitor());
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
		uint32_t tempDistance = 0;

		if (distance>99){tempDistance = distance/10;} else{tempDistance = 100;}
		uint8_t l = 0;

			while(tempDistance){
			tempDistance/=10;
			l++;
			}

	int8_t zeros = 6-l;
	if (distance < 100)zeros = 4;
		if (zeros > 0){
			for (int8_t i=0;i<zeros;i++){

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
	main_screen();
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
				if (stepMode == FULL_STEP)	 newSteps = speedKmh/kmhPerStep;	
				else if (stepMode == HALF_STEP) newSteps = speedKmh/kmhPerStep;
				int16_t shiftSteps = steps - newSteps;	//difference in speedometer readings (how much the arrow should be shifted)
				if (shiftSteps > 0){dir = 0;}else {dir = 1;}
				if (abs(shiftSteps)){
					arrowMoving = 1;

					TCCR0|=_BV(CS02)|_BV(CS00)|_BV(WGM01);
					OCR0 = stepInterval;	//interval between steps (Affects Stepper Motor Rotation Speed)
					TIMSK|=_BV(OCIE0);

				main_screen();

				}

}

void calculate_speed(){

			if(speedTimer>400){
						//if(speedRefresh)
						
						TIMSK&=~_BV(OCIE2);	//if Hall sensor was not triggered for too long (0,32s) it means that vehicle does not move
						TCNT2 = 0;
						speedTimer = 0;
						speedTimerRecent = 0;	//speedTimer;//?
						speedKmh = 0;
						firstMeasure = 0;
						cli();
						eep_operations(EEP_ODOMETER_START_ADDRESS, EEPROM_ADDRESS_SHIFT,EEP_ODOMETER_WRITE);	//if speed equals zero - save odometer data to EEPROM
						sei();
						}
			if((speedRefresh)&&(speedTimerRecent)){
						speedKmh = 1.0/(timePerTic*speedTimerRecent)*3.6*circLength;
						}

		if (speedKmh>scaleMax)speedKmh = scaleMax;
		speedRefresh = 0;
		}
void signal_monitor(){

		if((!(PINB&_BV(4)))&&(PINB&_BV(3))){
			draw_arrow(0);
			signalOn = 1;
			signalCounter = 0;
		}

		else if((!(PINB&_BV(3)))&&(PINB&_BV(4))){
			draw_arrow(1);
			signalOn = 1;
			signalCounter = 0;
		}

		else if((!(PINB&_BV(4)))&&(!(PINB&_BV(3)))){
			draw_skull();
			signalOn = 1;
			signalCounter = 0;
		}


		if (signalOn){
			if((PINB&_BV(3))&&(PINB&_BV(4))){
				GLCD_Clear();
				GLCD_Render();
				TIMSK|=_BV(TOIE1);	// If the turn signal (arrow) was switched on, and at the moment the turn signals are not lit, the timer is started.
									//This is to see if this is the interval between the blinking of the turn signals, or if the turn signal is already off.
			}

			if (signalCounter > 300)	//if interval is longer then the interval between the blinks - stop displaying turn/hazard sign
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



	if (newVoltage!=voltage)	//if voltage value changes - update data on the screen
	{
		voltage = newVoltage;
		main_screen();
	}

	newDistance=(round(totalRotations)*circLength)/10.0;
	if (newDistance!=distance)	//when the distance value changes by 100 meters - update the data on the screen
	{
		distance = newDistance;
		main_screen();
	}
}

uint8_t button_monitor(){
	uint8_t btnPressed = 0;
	if ((PINB&_BV(5))&&(PINB&_BV(6))&&(PINB&_BV(7))){
		btnPressed = 0;
		return 0;
	}
	else if((!(PINB&_BV(5)))&&(!btnPressed)){
		_delay_ms(BOUNCE_DELAY);
		if(!(PINB&_BV(5))){
			btnPressed = 1;
		}
	}
	else if((!(PINB&_BV(6)))&&(!btnPressed)){
		_delay_ms(BOUNCE_DELAY);
		if(!(PINB&_BV(6))){
		btnPressed = 2;
		}
	}
	else if((!(PINB&_BV(7)))&&(!btnPressed)){
		_delay_ms(BOUNCE_DELAY);
		if(!(PINB&_BV(7))){
		btnPressed = 3;
		}
	}
	return btnPressed;
}
void arrow_calibration(){
	int calibrationSteps = 0;
	if (stepMode==FULL_STEP){
		calibrationSteps = smSteps;
	}else{
		calibrationSteps = smSteps*2;
	}


steps = 0;
phase = 0;

	newSteps = calibrationSteps/4;	//moving arrow 90 degrees clockwise
		dir = 1;
		arrowMoving = 1;

		TCCR0|=_BV(CS02)|_BV(CS00)|_BV(WGM01);
		OCR0 = stepInterval;
		TIMSK|=_BV(OCIE0);


while (arrowMoving);

	_delay_ms(150);
	steps = calibrationSteps;
	newSteps = 0;  //moving arrow 360 degrees counterclockwise to make sure it is in zero position

	dir = 0;
	arrowMoving = 1;


	TCCR0|=_BV(CS02)|_BV(CS00)|_BV(WGM01);
	OCR0 = stepInterval;
	TIMSK|=_BV(OCIE0);

while (arrowMoving);
steps = 0;
newSteps = 0;
//arrowCalibrated = 1;
}

void draw_arrow (uint8_t arrowDir){
	if (arrowDir){
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

void draw_skull (void)
{
GLCD_Clear();
GLCD_GotoXY(21+6, 7);
GLCD_DrawBitmap(skull,86,52,GLCD_Black);
GLCD_Render();
}


int read_ADC(uint8_t mux, uint8_t cycles)
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


	void eep_operations (uint16_t eepStartAddress, uint8_t eepAddrShift, uint8_t eepAction){

		if (eepAction==EEP_WRITE){
			eeprom_update_byte((uint8_t*)eepStartAddress,0);
			eeprom_update_word((uint16_t*)(eepStartAddress+=eepAddrShift),pwmArrow);
			eeprom_update_word((uint16_t*)(eepStartAddress+=eepAddrShift),pwmDial);
			eeprom_update_byte((uint8_t*)(eepStartAddress+=eepAddrShift),scaleMax);
			eeprom_update_byte((uint8_t*)(eepStartAddress+=eepAddrShift),stepInterval);
			eeprom_update_word((uint16_t*)(eepStartAddress+=eepAddrShift),smSteps);
			eeprom_update_byte((uint8_t*)(eepStartAddress+=eepAddrShift),lcdContrast);
			eeprom_update_byte((uint8_t*)(eepStartAddress+=eepAddrShift),magnetsOnWheel);
			eeprom_update_byte((uint8_t*)(eepStartAddress+=eepAddrShift),stepMode);
			eeprom_update_float((float*)(eepStartAddress+=eepAddrShift),wheelDiameter);
			eeprom_update_float((float*)(eepStartAddress+=eepAddrShift),gearRatio);
			eeprom_update_float((float*)(eepStartAddress+=eepAddrShift),degreesPerKmh);
			}
			if(eepAction==EEP_READ){
			pwmArrow = eeprom_read_word((uint16_t*)(eepStartAddress+=eepAddrShift));
			pwmDial = eeprom_read_word((uint16_t*)(eepStartAddress+=eepAddrShift));
			scaleMax = eeprom_read_byte((uint8_t*)(eepStartAddress+=eepAddrShift));
			stepInterval = eeprom_read_byte((uint8_t*)(eepStartAddress+=eepAddrShift));
			smSteps = eeprom_read_word((uint16_t*)(eepStartAddress+=eepAddrShift));
			lcdContrast = eeprom_read_byte((uint8_t*)(eepStartAddress+=eepAddrShift));
			magnetsOnWheel = eeprom_read_byte((uint8_t*)(eepStartAddress+=eepAddrShift));
			stepMode = eeprom_read_byte((uint8_t*)(eepStartAddress+=eepAddrShift));
			wheelDiameter = eeprom_read_float((float*)(eepStartAddress+=eepAddrShift));
			gearRatio = eeprom_read_float((float*)(eepStartAddress+=eepAddrShift));
			degreesPerKmh = eeprom_read_float((float*)(eepStartAddress+=eepAddrShift));
			}
		if (eepAction==EEP_ODOMETER_READ){
			uint32_t tempTotalRotations = 0;
			for (uint8_t i = 0;i<=ODOMETER_EEP_CELLS;i++){
			tempTotalRotations = eeprom_read_dword((uint32_t*)(eepStartAddress+(eepAddrShift*i)));
			if(tempTotalRotations>totalRotations){
				totalRotations = tempTotalRotations;
				odometerCurrentAddress = i + 1;
				}
			}
		}
		if (eepAction==EEP_ODOMETER_WRITE){
			eeprom_write_dword((uint32_t*)(eepStartAddress+(odometerCurrentAddress*eepAddrShift)),totalRotations);
			odometerCurrentAddress++;
			if (odometerCurrentAddress>ODOMETER_EEP_CELLS)odometerCurrentAddress = 0;
		}


	}

	uint16_t set_value (uint16_t maxValue, uint16_t minValue, uint16_t currValue, uint8_t tens, const char *text){

		GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
		GLCD_Clear();
		GLCD_GotoLine(1);
		GLCD_GotoX(10);
		GLCD_PrintString(text);
		GLCD_GotoY(8+16);
		GLCD_GotoX(90);
		GLCD_PrintString("Edit");
		GLCD_GotoY(20+16);
		GLCD_GotoX(90);
		GLCD_PrintString("Back");
		GLCD_GotoY(32+16);
		GLCD_GotoX(90);
		GLCD_PrintString("Save");

		if (tens){
			uint16_t devider = 1;
			for (int8_t i = 0;i<tens;i++){devider*=10;}
			GLCD_SetFont(Font5x8,5,8,GLCD_Overwrite);
			GLCD_GotoY(40);
			GLCD_GotoX(6);
			GLCD_PrintString("min:");
			
			GLCD_PrintDouble(minValue/(double)devider,devider);
			GLCD_GotoY(50);
			GLCD_GotoX(6);
			GLCD_PrintString("max:");
			GLCD_PrintDouble(maxValue/(double)devider,devider);
			}else{
			GLCD_GotoY(40);
			GLCD_GotoX(6);
			GLCD_PrintString("min:");
			GLCD_PrintInteger(minValue);
			GLCD_GotoY(50);
			GLCD_GotoX(6);
			GLCD_PrintString("max:");
			GLCD_PrintInteger(maxValue);
		}
		
		
		GLCD_GotoY(24-4);
		GLCD_GotoX(5);
		GLCD_SetFont(Arial12x17, 12, 17, GLCD_Overwrite);

//

    uint16_t tempValue;
	int8_t digitIndex;
	uint8_t valueLength;
	uint8_t maxValueLength;
	int8_t *digitsArr;
	int8_t currentItem = 0;
	tempValue = currValue;
	valueLength = 0;


	uint16_t new_value (void)
	{
		uint16_t newValue = 0;
		//gathering digits back to the integer
		for(digitIndex = maxValueLength - 1;digitIndex>=0;digitIndex--){
			uint16_t tenPower = digitsArr[digitIndex];
			for (uint8_t j = 0;j<digitIndex;j++){
				tenPower*=10;
			}
			newValue += tenPower;
		}
		return newValue;
	}
	uint8_t check_value(uint16_t checkValue){
			if((checkValue<=maxValue)&&(checkValue>=minValue)){return 1;}else{return 0;}
	}

	while(tempValue)       //finding the number of digits for current value
	   {
		   tempValue = tempValue / 10;
		   valueLength++;
		        }
	tempValue = maxValue;
	maxValueLength = 0;
	while(tempValue)       //finding the number of digits for maximum value allowed
		{
			tempValue = tempValue / 10;
			maxValueLength++;
				}

		digitsArr = (int8_t*)malloc(maxValueLength * sizeof(int8_t));
		tempValue = currValue;
		for(digitIndex = 0;digitIndex<maxValueLength;digitIndex++){	//putting digits to array
			if (digitIndex<valueLength){
				digitsArr[digitIndex] = tempValue % 10;
				tempValue = tempValue / 10;
			}else{
				digitsArr[digitIndex] = 0;	//if current value is shorter then maximum value - set extra digits to zero
			}
		}

		int8_t rectShift = (5+(maxValueLength-1)*13)-currentItem*13;
		for(digitIndex = maxValueLength - 1;digitIndex>=0;digitIndex--){
				GLCD_PrintInteger(digitsArr[digitIndex]);
					if ((digitIndex==tens)&&(tens)){
						GLCD_PrintString(".");
					rectShift+=5;
					}
				}



		GLCD_InvertRect(rectShift,24-4,rectShift+12,40-4);
		GLCD_Render();



		while (button_monitor());
		uint8_t currentButton = 0;
			while(1){
				currentButton = button_monitor();
				if(currentButton){
					rectShift = 0;
						if (currentButton == 1) {
							currentItem++;
							if (currentItem==maxValueLength){
								currentItem = 0;

								//=====
									while(button_monitor());
									int8_t menuItem = 0;
									GLCD_InvertRect(rectShift+5,24-4,rectShift+5+12,40-4);
									while(1){
									currentButton = button_monitor();

										if(currentButton){
											if (currentButton == 3) {
												menuItem--;
												if (menuItem<0)menuItem = 2;
											}
											else if (currentButton == 2) {
												menuItem++;
												if (menuItem>2)menuItem=0;
											}
											else if (currentButton == 1) {
													if (!menuItem){	//getting back to value edit
														currentItem = 0;
														for (int8_t i = 0;i<3;i++){GLCD_DrawRectangle(86,5+16+i*12,116,17+16+i*12,GLCD_White);}
														GLCD_Render();
														break;
													}
													else if (menuItem == 1){
														return currValue; //if changes are discarded - return initial value
													}
													else if (menuItem == 2){
														return new_value();
													}
											}

									}
							for (int8_t i=0;i<3;i++){GLCD_DrawRectangle(86,5+16+i*12,116,17+16+i*12,GLCD_White);}

									GLCD_DrawRectangle(86,5+16+menuItem*12,116,17+16+menuItem*12,GLCD_Black);
									GLCD_Render();
							while(button_monitor());
							while(!button_monitor());
							}
							//=============
						}

					}

					if (currentButton == 3) {
						int8_t prevValue = digitsArr[currentItem];
						digitsArr[currentItem]++;
						if (digitsArr[currentItem]>9)digitsArr[currentItem] = 0;
						if (!check_value(new_value()))digitsArr[currentItem] = prevValue;
					}
					else if (currentButton == 2) {
						int8_t prevValue = digitsArr[currentItem];
						digitsArr[currentItem]--;
						if (digitsArr[currentItem]<0)digitsArr[currentItem] = 9;
						if (!check_value(new_value()))digitsArr[currentItem] = prevValue;
					}
			GLCD_GotoY(24-4);
			GLCD_GotoX(5);
			for(digitIndex = maxValueLength - 1;digitIndex>=0;digitIndex--){
				GLCD_PrintInteger(digitsArr[digitIndex]);
				if ((digitIndex==tens)&&(tens)) {GLCD_PrintString(".");}
			}
			rectShift = (5+(maxValueLength-1)*13)-currentItem*13;
			if (tens){
					if(currentItem<tens){
						rectShift+=5;
					}
				}
			GLCD_InvertRect(rectShift,24-4,rectShift+12,40-4);
			GLCD_Render();
			}
		while(button_monitor());
		while(!button_monitor());
		}
	}