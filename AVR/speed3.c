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

#define pi 3.141592653		
#define TIMER2_PRESCALER 256
#define TIC  255		//amount of Timer2 tics between speedTimerRough incrementation (F_CPU=16Mhz, prescaler=256 1 tic = 16 us). 
#define AREF 2.5		//reference voltage
#define DEVIDER 6		//divider (for battery voltage measurement)
#define SLEEP_VOLTAGE 100 //10V
#define ADC_CYCLES 10
#define ADC_MUX 4
#define FULL_STEP 1
#define HALF_STEP 2
#define EEPROM_START_ADDRESS 8
#define EEP_ODOMETER_START_ADDRESS 128
#define EEPROM_ADDRESS_SHIFT 4
#define EEP_WRITE 1
#define EEP_READ 0
#define EEP_ODOMETER_WRITE 3
#define EEP_ODOMETER_READ 2
#define ODOMETER_EEP_CELLS 24  //+1 (counting from zero)
#define BOUNCE_DELAY 50	//50 ms is usually enough
#define MAX_PERIOD 1500	//MAX_PERIOD*TIC*16us Speed is considered to be zero if delay between Hall sensor actuations is longer
#define PERIOD_INCREASE_TRESHOLD 60
#define SIGNAL_COUNTER_MAX 300	//SIGNAL_COUNTER_MAX*1024*4us ~1.2s
#define RIGHT 1
#define LEFT 0
#define PIN_RIGHT	PINB&_BV(4)
#define PIN_LEFT	PINB&_BV(3)
#define PIN_SET		PINB&_BV(5)
#define PIN_DOWN	PINB&_BV(6)
#define PIN_UP		PINB&_BV(7)
#define BRICK_1 105
#define BRICK_2 110
#define BRICK_3 115
#define BRICK_4 120

volatile uint16_t speedTimerRough = 0;
volatile uint16_t speedTimerRoughPrevious = 0;
volatile uint8_t previousTCNT2 = 0;
volatile uint16_t sensorActations = 0;
volatile uint8_t arrowMoving = 0;
volatile int8_t phase = 0; 
volatile int16_t steps = 0;	//actual stepper motor shaft position
uint32_t speedTimerPrecise = 0;
double frequency = 0;
double timePerTic = 0;	//duration of Counter1 tic in seconds
double circumference = 0;	//wheel circumference between magnets considering gear ratio
double speedKmh = 0;
double kmhPerStep = 0;
uint16_t newSteps = 0;	//required stepper motor shaft position
uint8_t signalOn = 0;	//if turn or hazard lights on, = 1
uint8_t firstMeasure = 0;
uint8_t debugMode = 0;	//0
int16_t voltage = 0;
int16_t newVoltage = 0;
uint8_t odometerCurrentAddress = 0;
uint8_t dir = 0;
uint16_t signalCounter = 0;//counter of turn lights interval
uint32_t milage = 0;	//milage in meters
int8_t arrowCalibrated = 0;	//0
			//values of these variables are stored in EEPROM and could be changed from GUI
uint8_t lcdContrast;
#define LCD_CONTRAST_DEFAULT 250
#define LCD_CONTRAST_MIN 50
#define LCD_CONTRAST_MAX 255
uint8_t magnetsOnWheel;
#define MAGNETS_ON_WHEEL_DEFAULT 6
#define MAGNETS_ON_WHEEL_MIN 1
#define MAGNETS_ON_WHEEL_MAX 16
double gearRatio; //needed if magnets are not on the wheel
#define GEAR_RATIO_DEFAULT 1.0
#define GEAR_RATIO_MIN 0.1
#define GEAR_RATIO_MAX 1
double wheelDiameter;	//wheel diameter in meters
#define WHEEL_DIAMETER_DEFAULT 0.70
#define WHEEL_DIAMETER_MIN 0.1
#define WHEEL_DIAMETER_MAX 2.0
double degreesPerKmh;
#define DEGREES_PER_KMH_DEFAULT 1.275
#define DEGREES_PER_KMH_MIN 0.1
#define DEGREES_PER_KMH_MAX 5
uint16_t pwmArrowLight;
#define PWM_ARROW_DEFAULT 1024
#define PWM_ARROW_MIN 0
#define PWM_ARROW_MAX 1024
uint16_t pwmDialLight;
#define PWM_DIAL_DEFAULT 1024
#define PWM_DIAL_MIN 0
#define PWM_DIAL_MAX 1024
uint8_t maxSpeedOnDial;	
#define MAX_SPEED_DEFAULT 190
#define MAX_SPEED_MIN 20
#define MAX_SPEED_MAX 255
uint8_t stepInterval; //interval between steps = stepInterval*64us (prescaler = 1024)
#define STEP_INTERVAL_DEFAULT 150
#define STEP_INTERVAL_MIN 50
#define STEP_INTERVAL_MAX 255
uint16_t stepperMotorSteps;
#define  STEPPER_MOTOR_STEPS_DEFAULT 96
#define  STEPPER_MOTOR_STEPS_MIN 16
#define  STEPPER_MOTOR_STEPS_MAX 400
uint8_t stepMode;
#define STEP_MODE_DEFAULT HALF_STEP
#define STEP_MODE_MIN FULL_STEP
#define STEP_MODE_MAX HALF_STEP
		//
int main(void);
uint16_t read_ADC(uint8_t mux, uint8_t cycles);
void step(uint8_t mode);
void calculate_speed();
void draw_arrow (uint8_t arrowDir);
void draw_skull ();
void main_screen();
void arrow_position_update();
void data_monitor();
void signal_monitor();
uint8_t button_monitor();
void menu_screen();
void arrow_calibration();
void eep_operations (uint16_t eepStartAddress, uint8_t eepAddrShift, uint8_t writeOrRead);
uint16_t set_value (uint16_t maxValue, uint16_t minValue, uint16_t currValue, uint8_t tens, const char *text);
void debug_screen();
void set_default_values();
const uint8_t phaseArrayFullStep [] = {	//FULL-STEP TWO PHASE
	0b00000110,	
	0b00000010,
	0b00000000,
	0b00000100,
	0b00000110,
	0b00000010,
	0b00000000,
	0b00000100
};								
const uint8_t phaseArrayHalfStep [] = {	//HALF STEP
	0b00000101,			
	0b00000110,
	0b00001010,
	0b00000010,
	0b00000001,
	0b00000000,
	0b00001000,
	0b00000100			
};
void set_default_values(){
	 lcdContrast = LCD_CONTRAST_DEFAULT;
	 magnetsOnWheel = MAGNETS_ON_WHEEL_DEFAULT;
	 gearRatio = GEAR_RATIO_DEFAULT;
	 wheelDiameter = WHEEL_DIAMETER_DEFAULT;	
	 degreesPerKmh = DEGREES_PER_KMH_DEFAULT;
	 pwmArrowLight = PWM_ARROW_DEFAULT;
	 pwmDialLight = PWM_DIAL_DEFAULT;
	 maxSpeedOnDial	= MAX_SPEED_DEFAULT;
	 stepInterval = STEP_INTERVAL_DEFAULT;
	 stepperMotorSteps =	STEPPER_MOTOR_STEPS_DEFAULT;
	 stepMode = STEP_MODE_DEFAULT;
}
void presets (void){
	DDRA|=_BV(0); //ENABLE 2
	DDRA|=_BV(1); //PHASE 2
	DDRA|=_BV(2); //PHASE 1
	DDRA|=_BV(3); //ENABLE 1
	DDRD|=_BV(4);//PWM DIAL LIGHT
	DDRD|=_BV(5);//PWM ARROW LIGHT
	PORTA|=_BV(3);//ENABLE 1 high (disabled)
	PORTA|=_BV(0);//ENABLE 2 high (disabled)
	PORTB|=_BV(5);//internal pull-up for external buttons on PB5, PB6, PB7
	PORTB|=_BV(6);
	PORTB|=_BV(7);
	//ADC setup
	ADCSRA |= _BV(ADEN);
	ADCSRA |= _BV(ADPS0);		//
	ADCSRA |= _BV(ADPS1);		// ADC prescaler 128
	ADCSRA |= _BV(ADPS2);		//
	//reading data from EEPROM
	uint8_t	firstEepRead = eeprom_read_byte((uint8_t*)EEPROM_START_ADDRESS);//if the device is started for the first time the default values have to be written to EEPROM
	if (firstEepRead){
		set_default_values();
		eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
		for (uint8_t i = 0;i<=ODOMETER_EEP_CELLS;i++){
			eep_operations(EEP_ODOMETER_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_ODOMETER_WRITE);
		}
	}
	eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_READ);
	eep_operations(EEP_ODOMETER_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_ODOMETER_READ);
	TCCR2|=_BV(CS21)|_BV(CS22)|_BV(WGM21);//Timer2 is used to measure time between Hall sensor actuation
	OCR2 = TIC-1; //upper limit of Timer2
	//dial and arrow light PWM (Timer1)
	TCCR1A = _BV(WGM10)|_BV(WGM11)|_BV(COM1B1)|_BV(COM1A1);	//Fast PWM 10-bit
	TCCR1B = _BV(WGM12)|_BV(CS10)|_BV(CS11);	//prescaler = 64
	OCR1A = pwmArrowLight;
	OCR1B = pwmDialLight;
	//
	circumference = gearRatio * wheelDiameter * pi/magnetsOnWheel;	
	timePerTic = 1.0/(F_CPU/TIMER2_PRESCALER);			//counter tic time interval in seconds (16 us, Timer2 prescaler=256)
	kmhPerStep = (360.0/(stepperMotorSteps*stepMode))/degreesPerKmh;
	//display initialization
	GLCD_Setup();
	GLCD_Clear();
	GLCD_SetContrast(lcdContrast);
	GLCD_Render();
	sei();
	if (!arrowCalibrated) arrow_calibration();
	MCUCR|= _BV(ISC11); //Falling edge interrupt INT1 (Hall sensor)
	GICR|=_BV(INT1); //External Interrupt Enable INT1
}
int main(void)
{
	presets();
	if (!debugMode)main_screen();
	while(1){
		data_monitor();
		calculate_speed();
		arrow_position_update();
		if(!debugMode)signal_monitor();else debug_screen();
		if(button_monitor()) menu_screen();
	}
	return 0;
}
ISR( TIMER0_COMP_vect ){
	step(stepMode);
	if (steps == newSteps){	//if the required arrow position is reached - stop Stepper Motor rotation
		arrowMoving = 0;
		TIMSK&=~_BV(OCIE0);
		TCCR0 = 0;
		OCR0 = 0;
	}
}
void step(uint8_t mode){
	uint8_t tempPort = 0;
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
	if		(phase < 0) phase = 7;
	else if (phase > 7) phase = 0;
	tempPort = PORTA&~0x0F; 
	if(mode == HALF_STEP)		tempPort|=phaseArrayHalfStep[phase];
	else if(mode == FULL_STEP)  tempPort|=phaseArrayFullStep[phase];
	PORTA = tempPort;
}
ISR( TIMER2_COMP_vect ){
	speedTimerRough++; //speedTimer increments each period timePerTic*TIC
}
ISR (TIMER1_OVF_vect){
	if (signalOn) signalCounter++;
}
ISR(INT1_vect){
//interrupt occurs when Hall sensor is actuated
if (firstMeasure)
	{
		previousTCNT2 = TCNT2;
		TCNT2 = 0;
		speedTimerRoughPrevious = speedTimerRough;
		speedTimerRough = 0;
		sensorActations++;
	}
else
	{
		TCNT2 = 0;
		TIMSK|=_BV(OCIE2);	//first actuation of the Hall sensor enables TIMER2 compare match interrupt
		firstMeasure = 1;
	}
}
void menu_screen(){
	uint8_t yes_no_menu(const char *textLine1, uint8_t offsetLine1, const char *textLine2, uint8_t offsetLine2){
		uint8_t yesOrNo = 0;
		GLCD_Clear();
		GLCD_GotoLine(2);
		GLCD_GotoX(offsetLine1);
		GLCD_PrintString(textLine1);
		GLCD_GotoLine(4);
		GLCD_GotoX(offsetLine2);
		GLCD_PrintString(textLine2);
		
		GLCD_GotoLine(6);
		GLCD_GotoX(28);
		GLCD_PrintString("NO");
		GLCD_GotoX(84);
		GLCD_PrintString("YES");
		
		GLCD_DrawRectangle(20+59*yesOrNo,45,46+59*yesOrNo, 57,GLCD_Black);
		GLCD_Render();
		while(button_monitor());
		while(1){
			uint8_t button = button_monitor();
			if ((button==2)||(button==3)){
				yesOrNo = yesOrNo^_BV(0);
				GLCD_DrawRectangle(20+59*yesOrNo,45,46+59*yesOrNo, 57,GLCD_Black);
				GLCD_DrawRectangle(20+59*!yesOrNo,45,46+59*!yesOrNo, 57,GLCD_White);
				GLCD_Render();
				while(button_monitor());
			}
			else if(button==1) return yesOrNo;
		}
	}
	uint8_t offset = 88;
	static int8_t menuItem;
	static int8_t page;
	if (menuItem > 5){page++;menuItem = 0;}
	if (menuItem < 0){page--;menuItem= 5;}
	if ((page == 2)&&(menuItem > 3)){page = 0; menuItem = 0;}
	if (page < 0){page = 2;menuItem = 3;}
	GLCD_Clear();
	GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
	if(page==0){
		//item 0
		GLCD_GotoX(10);
		GLCD_GotoLine(1);
		GLCD_PrintString("Dial PWM");
		GLCD_GotoX(offset);
		GLCD_PrintInteger(pwmDialLight);
		//item 1
		GLCD_GotoX(10);
		GLCD_GotoLine(2);
		GLCD_PrintString("Arrow PWM");
		GLCD_GotoX(offset);
		GLCD_PrintInteger(pwmArrowLight);
		//item 2
		GLCD_GotoX(10);
		GLCD_GotoLine(3);
		GLCD_PrintString("Wheel diam.");
		GLCD_GotoX(offset);
		GLCD_PrintDouble(wheelDiameter,1000);
		//item 3
		GLCD_GotoX(10);
		GLCD_GotoLine(4);
		GLCD_PrintString("Gear ratio");
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
	else if (page==1){
		//item 6
		GLCD_GotoX(10);
		GLCD_GotoLine(1);
		GLCD_PrintString("Max speed");
		GLCD_GotoX(offset);
		GLCD_PrintInteger(maxSpeedOnDial);
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
		GLCD_PrintInteger(stepperMotorSteps);
		//item 9
		GLCD_GotoX(10);
		GLCD_GotoLine(4);
		GLCD_PrintString("Step interv.");
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
	else if (page==2){
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
	while (button_monitor());
	uint8_t currentButton = 0;
	uint16_t newValue = 0;
	while(1){
		currentButton = button_monitor();
		if(currentButton){
			if (currentButton == 1){
				switch(menuItem+6*page)		{
					case 0:
					{
						newValue = set_value(PWM_DIAL_MAX,PWM_DIAL_MIN,pwmDialLight,0,"Dial light PWM");
						if (newValue!=pwmDialLight){
							pwmDialLight = newValue;
							eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
							OCR1B = pwmDialLight;
						}
						break;
					}
					case 1:
					{
						newValue = set_value(PWM_DIAL_MAX,PWM_ARROW_MIN,pwmArrowLight,0,"Arrow light PWM");
						if (newValue!=pwmArrowLight){
							pwmArrowLight = newValue;
							eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
							OCR1A = pwmArrowLight;
						}
						break;
					}
					case 2:
					{
						newValue = set_value(WHEEL_DIAMETER_MAX*1000,WHEEL_DIAMETER_MIN*1000,wheelDiameter*1000,3,"Wheel Diameter (m)");
						if (newValue!=wheelDiameter*1000){
							wheelDiameter = newValue/1000.0;
							eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
						}
						break;
					}
					case 3:
					{
						newValue = set_value(GEAR_RATIO_MAX*1000,GEAR_RATIO_MIN*1000,gearRatio*1000,3,"Gear ratio");
						if (newValue!=gearRatio*1000){
							gearRatio = newValue/1000.0;
							eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
						}
						break;
					}
					case 4:
					{
						newValue = set_value(MAGNETS_ON_WHEEL_MAX,MAGNETS_ON_WHEEL_MIN,magnetsOnWheel,0,"Magnets on wheel");
						if (newValue!=magnetsOnWheel){
							magnetsOnWheel = newValue;
							eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
						}
						break;
					}
					case 5:
					{
						newValue = set_value(STEP_INTERVAL_MAX,STEP_MODE_MIN,stepMode,0,"1-full, 2-half");
						if (newValue!=stepMode){
							stepMode = newValue;
							eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
						}
						break;
					}
					case 6:
					{
						newValue = set_value(MAX_SPEED_MAX,MAX_SPEED_MIN,maxSpeedOnDial,0,"Maximum speed");
						if (newValue!=maxSpeedOnDial){
							maxSpeedOnDial = newValue;
							eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
						}
						break;
					}
					case 7:
					{
						newValue = set_value(DEGREES_PER_KMH_MAX*1000,DEGREES_PER_KMH_MIN*1000,degreesPerKmh*1000,3,"Degrees per km/h");
						if (newValue!=degreesPerKmh*1000){
							degreesPerKmh = newValue/1000.0;
							eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
						}
						break;
					}
					case 8:
					{
						newValue = set_value(STEPPER_MOTOR_STEPS_MAX,STEPPER_MOTOR_STEPS_MIN,stepperMotorSteps,0,"Stepper motor steps");
						if (newValue!=stepperMotorSteps){
							stepperMotorSteps = newValue;
							eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
						}
						break;
					}
					case 9:
					{
						newValue = set_value(STEP_INTERVAL_MAX,STEP_INTERVAL_MIN,stepInterval,0,"Step interval X64us");
						if (newValue!=stepInterval){
							stepInterval = newValue;
							eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
						}
						break;
					}
					case 10:
					{
						newValue = set_value(LCD_CONTRAST_MAX,LCD_CONTRAST_MIN,lcdContrast,0,"LCD contrast");
						if (newValue!=lcdContrast){
							lcdContrast = newValue;
							GLCD_SetContrast(lcdContrast);
							eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
						}
						break;
					}
					case 11:
					{
						debugMode = debugMode^_BV(0);
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
 						if(yes_no_menu("RESET",45,"ODOMETER?",34)){
							GLCD_Clear();
							GLCD_GotoLine(1);
							GLCD_GotoX(6);
							GLCD_PrintString("RESETTING ODOMETER");
														
							GLCD_GotoLine(3);
							GLCD_GotoX(2);
							GLCD_PrintString("Cells cleared");
							GLCD_Render();
							//odometer reset
							milage = 0;
							odometerCurrentAddress = 0;
							sensorActations = 0;
							for (uint8_t i = 0;i<=ODOMETER_EEP_CELLS;i++){
								eep_operations(EEP_ODOMETER_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_ODOMETER_WRITE);
								GLCD_GotoX(83);
								GLCD_PrintInteger(i+1);
								GLCD_PrintString("/");
								GLCD_PrintInteger(ODOMETER_EEP_CELLS+1);
								GLCD_Render();
							}
							odometerCurrentAddress = 0;
							GLCD_GotoLine(6);
							GLCD_GotoX(45);
							GLCD_PrintString("DONE!");
							GLCD_Render();
							while(button_monitor());
							while(!button_monitor());
 							break;
						}else{
 							break;
 						}
						break;
					}
					case 14:
					{
					//load defaults
						if (yes_no_menu("RESTORE",40,"DEFAULT values?",20)){
							GLCD_Clear();
							GLCD_GotoLine(3);
							GLCD_GotoX(8);
							GLCD_PrintString("LOADING DEFAULTS...");
							GLCD_Render();
							set_default_values();
							eep_operations(EEPROM_START_ADDRESS,EEPROM_ADDRESS_SHIFT,EEP_WRITE);
							GLCD_GotoLine(5);
							GLCD_GotoX(45);
							GLCD_PrintString("DONE!");
							GLCD_Render();
							while(button_monitor());
							while(!button_monitor());
							break;
							}else{
							break;
						}
						break;
					}
					case 15:
					{
						//exit menu
						while (button_monitor());
						main();
					}
				}//
			}
		else if(currentButton == 2)	menuItem++;
		else if(currentButton == 3)	menuItem--;
		while (!button_monitor());
		menu_screen();
		}
	}
	
}
void main_screen()
{
	if (signalOn) return;
	uint8_t offsetX = 10;
	uint8_t offsetY = 11;
	GLCD_Clear();
	GLCD_DrawRectangle(offsetX,offsetY,26+offsetX,12+offsetY,GLCD_Black);
	GLCD_DrawRectangle(27+offsetX,3+offsetY,28+offsetX,9+offsetY,GLCD_Black);

	if (voltage>=BRICK_1)GLCD_FillRectangle(2+offsetX,2+offsetY,6+offsetX,10+offsetY,GLCD_Black);
	if (voltage>=BRICK_2)GLCD_FillRectangle(8+offsetX,2+offsetY,12+offsetX,10+offsetY,GLCD_Black);
	if (voltage>=BRICK_3)GLCD_FillRectangle(14+offsetX,2+offsetY,18+offsetX,10+offsetY, GLCD_Black);
	if (voltage>=BRICK_4)GLCD_FillRectangle(20+offsetX,2+offsetY,24+offsetX,10+offsetY,GLCD_Black);

	GLCD_GotoXY(33+offsetX, 4+offsetY);
	GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
	GLCD_PrintDouble(voltage/10.0,10);
	GLCD_PrintString("V");

	GLCD_SetFont(Arial_Narrow18x32, 18, 32, GLCD_Overwrite);
	GLCD_GotoXY(4, 31);

	uint32_t tempMilage = milage/100;
	int8_t l = 0;
		
	while(tempMilage){
		tempMilage/=10;
		l++;
	}
	if (l<2) l = 2;
	int8_t zeros = 6-l;
			for (int8_t i = 0;i<zeros;i++){
				GLCD_PrintString("0");
			}
	GLCD_PrintInteger(milage/1000);
	if(milage<100000000){	
		GLCD_PrintString(".");
		GLCD_PrintInteger((milage%1000)/100);
	}
	GLCD_Render();
}
void arrow_position_update(){
	newSteps = speedKmh/kmhPerStep;
	int16_t shiftSteps = steps - newSteps;	//difference in speedometer readings (for how many steps arrow should be shifted)
	if (shiftSteps==0)return;
	if (shiftSteps > 0) dir = 0; else dir = 1;
	arrowMoving = 1;
	//Timer0 is used to generate pulses for Stepper Motor driver
	TCCR0|=_BV(CS02)|_BV(CS00)|_BV(WGM01); //prescaler 1024 (1 tic = 64us)
	OCR0 = stepInterval;	//interval between steps (Affects Stepper Motor Rotation Speed)
	TIMSK|=_BV(OCIE0);
	
}
void calculate_speed(){
	if(speedTimerRough>speedTimerRoughPrevious+PERIOD_INCREASE_TRESHOLD/magnetsOnWheel){	//If speed suddenly reduces to zero then next actuation of Hall sensor is not going to happen, so speedTimerRoughPrevious and previousTCNT2 will not be updated
			speedTimerPrecise = speedTimerRough*(uint32_t)TIC;								//so if speedTimerRough increases significantly in comparison to previous period, speedTimerPrecise is updated using current speedTimerRough value
	}else{
			speedTimerPrecise = (speedTimerRoughPrevious*(uint32_t)TIC) + previousTCNT2;
	}
	if (speedTimerPrecise){
			frequency = 1.0/(timePerTic*speedTimerPrecise);
			speedKmh = frequency*circumference*3.6; //3.6 is for converting m/s to km/h
	}
	if(speedTimerRough>(MAX_PERIOD/magnetsOnWheel)){	//if Hall sensor was not actuated for too long (MAX_PERIOD*TIC/magnetsOnWheel*0.16us) it means that vehicle does not move
			TIMSK&=~_BV(OCIE2);
			TCNT2 = 0;
			speedTimerRough = 0;
			speedTimerRoughPrevious = 0;
			speedTimerPrecise = 0;
			speedKmh = 0;
			firstMeasure = 0;
			frequency = 0;
			previousTCNT2 = 0;
			milage = milage + sensorActations*circumference;
			sensorActations = 0;
			cli();
			eep_operations(EEP_ODOMETER_START_ADDRESS, EEPROM_ADDRESS_SHIFT,EEP_ODOMETER_WRITE);	//if speed equals zero - save odometer data to EEPROM
			sei();
	}
	if (speedKmh>maxSpeedOnDial)speedKmh = maxSpeedOnDial;
}
void signal_monitor(){
	if(!(PIN_RIGHT)&&(PIN_LEFT)){
		draw_arrow(LEFT);
		signalOn = 1;
		signalCounter = 0;
	}
	else if(!(PIN_LEFT)&&(PIN_RIGHT)){
		draw_arrow(RIGHT);
		signalOn = 1;
		signalCounter = 0;
	}
	else if(!(PIN_RIGHT)&&!(PIN_LEFT)){
		draw_skull();
		signalOn = 1;
		signalCounter = 0;
	}
	if (!signalOn) return;
	if((PIN_LEFT)&&(PIN_RIGHT)){
		GLCD_Clear();
		GLCD_Render();
		TIMSK|=_BV(TOIE1);		// If the turn signal (arrow) was switched on, and at the moment the turn signals are not lit, the timer is started.
	}							// In order to define if this is the interval between the blinking of the turn signals, or if the turn signal is already off.
	if (signalCounter > SIGNAL_COUNTER_MAX)	// if interval is longer then the interval between the blinks - stop displaying turn/hazard sign
	{
		signalOn = 0;
		signalCounter = 0;
		TIMSK&=~_BV(TOIE1);
		main_screen();
	}
}
void data_monitor(){
	newVoltage = (read_ADC(ADC_MUX,ADC_CYCLES)/102.3)*AREF*DEVIDER;
	if ((newVoltage<SLEEP_VOLTAGE)&&(!debugMode)){	//Disabling power consumers during ignition to prevent MCU from reboot
		PORTA|=_BV(3);
		PORTA|=_BV(0);
		OCR1A = 0;
		OCR1B = 0;
		GLCD_Clear();
		GLCD_Render();
		while((read_ADC(ADC_MUX,ADC_CYCLES)/102.3)*AREF*DEVIDER<SLEEP_VOLTAGE);
		OCR1A = pwmArrowLight;
		OCR1B = pwmDialLight;
	}
	if (newVoltage!=voltage)	//if voltage value changes - update data on the screen
	{
		voltage = newVoltage;
		if (!debugMode)main_screen();
	}
	uint32_t newMilage = sensorActations*circumference;
	if (newMilage>99)	
	{
		milage+= newMilage;
		sensorActations = 0;
		if (!debugMode)main_screen();//when the milage value changes by 100 meters - update data on the screen
	}
}
uint8_t button_monitor(){
	uint8_t btnPressed = 0;
	if ((PIN_SET)&&(PIN_DOWN)&&(PIN_UP)){
		btnPressed = 0;
		return 0;
	}
	else if((!(PIN_SET))&&(!btnPressed)){
		_delay_ms(BOUNCE_DELAY);
		if(!(PIN_SET)){
			btnPressed = 1;
		}
	}
	else if((!(PIN_DOWN))&&(!btnPressed)){
		_delay_ms(BOUNCE_DELAY);
		if(!(PIN_DOWN)){
			btnPressed = 2;
		}
	}
	else if((!(PIN_UP))&&(!btnPressed)){
		_delay_ms(BOUNCE_DELAY);
		if(!(PIN_UP)){
			btnPressed = 3;
		}
	}
	return btnPressed;
}
void arrow_calibration(){
	steps = 0;
	phase = 0;
	newSteps = stepperMotorSteps*stepMode/4;	//moving arrow 90 degrees clockwise
	dir = 1;
	arrowMoving = 1;
	TCCR0|=_BV(CS02)|_BV(CS00)|_BV(WGM01);
	OCR0 = stepInterval;
	TIMSK|=_BV(OCIE0);
	while (arrowMoving);
	_delay_ms(150);
	steps = stepperMotorSteps*stepMode;
	newSteps = 0;  //moving arrow 360 degrees counterclockwise to make sure it is in zero position
	dir = 0;
	arrowMoving = 1;
	TCCR0|=_BV(CS02)|_BV(CS00)|_BV(WGM01);
	OCR0 = stepInterval;
	TIMSK|=_BV(OCIE0);
	while (arrowMoving);
	steps = 0;
	newSteps = 0;
	arrowCalibrated = 1;
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
	GLCD_GotoXY(27, 7);
	GLCD_DrawBitmap(skull,86,52,GLCD_Black);
	GLCD_Render();
}
uint16_t read_ADC(uint8_t mux, uint8_t cycles){
	ADMUX = mux;
	uint16_t tmp = 0;
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
		eeprom_update_word((uint16_t*)(eepStartAddress+=eepAddrShift),pwmArrowLight);
		eeprom_update_word((uint16_t*)(eepStartAddress+=eepAddrShift),pwmDialLight);
		eeprom_update_byte((uint8_t*)(eepStartAddress+=eepAddrShift),maxSpeedOnDial);
		eeprom_update_byte((uint8_t*)(eepStartAddress+=eepAddrShift),stepInterval);
		eeprom_update_word((uint16_t*)(eepStartAddress+=eepAddrShift),stepperMotorSteps);
		eeprom_update_byte((uint8_t*)(eepStartAddress+=eepAddrShift),lcdContrast);
		eeprom_update_byte((uint8_t*)(eepStartAddress+=eepAddrShift),magnetsOnWheel);
		eeprom_update_byte((uint8_t*)(eepStartAddress+=eepAddrShift),stepMode);
		eeprom_update_float((float*)(eepStartAddress+=eepAddrShift),wheelDiameter);
		eeprom_update_float((float*)(eepStartAddress+=eepAddrShift),gearRatio);
		eeprom_update_float((float*)(eepStartAddress+=eepAddrShift),degreesPerKmh);
	}
	else if(eepAction==EEP_READ){
		pwmArrowLight = eeprom_read_word((uint16_t*)(eepStartAddress+=eepAddrShift));
		if ((pwmArrowLight>PWM_ARROW_MAX)||(pwmArrowLight<PWM_ARROW_MIN))pwmArrowLight=PWM_ARROW_DEFAULT;
		pwmDialLight = eeprom_read_word((uint16_t*)(eepStartAddress+=eepAddrShift));
		if ((pwmDialLight>PWM_DIAL_MAX)||(pwmDialLight<PWM_DIAL_MIN))pwmDialLight=PWM_ARROW_DEFAULT;
		maxSpeedOnDial = eeprom_read_byte((uint8_t*)(eepStartAddress+=eepAddrShift));
		if ((maxSpeedOnDial>MAX_SPEED_MAX)||(maxSpeedOnDial<MAX_SPEED_MIN))maxSpeedOnDial=MAX_SPEED_DEFAULT;
		stepInterval = eeprom_read_byte((uint8_t*)(eepStartAddress+=eepAddrShift));
		if ((stepInterval>STEP_INTERVAL_MAX)||(stepInterval<STEP_INTERVAL_MIN))stepInterval=STEP_INTERVAL_DEFAULT;
		stepperMotorSteps = eeprom_read_word((uint16_t*)(eepStartAddress+=eepAddrShift));
		if ((stepperMotorSteps>STEPPER_MOTOR_STEPS_MAX)||(stepperMotorSteps<STEPPER_MOTOR_STEPS_MIN))stepperMotorSteps=STEPPER_MOTOR_STEPS_DEFAULT;
		lcdContrast = eeprom_read_byte((uint8_t*)(eepStartAddress+=eepAddrShift));
		if ((lcdContrast>LCD_CONTRAST_MAX)||(lcdContrast<LCD_CONTRAST_MIN))lcdContrast=STEP_INTERVAL_DEFAULT;
		magnetsOnWheel = eeprom_read_byte((uint8_t*)(eepStartAddress+=eepAddrShift));
		if ((magnetsOnWheel>MAGNETS_ON_WHEEL_MAX)||(magnetsOnWheel<MAGNETS_ON_WHEEL_MIN))magnetsOnWheel=MAGNETS_ON_WHEEL_DEFAULT;
		stepMode = eeprom_read_byte((uint8_t*)(eepStartAddress+=eepAddrShift));
		if ((stepMode>STEP_MODE_MAX)||(stepMode<STEP_MODE_MIN))stepMode=STEP_MODE_DEFAULT;
		wheelDiameter = eeprom_read_float((float*)(eepStartAddress+=eepAddrShift));
		if ((wheelDiameter>WHEEL_DIAMETER_MAX)||(wheelDiameter<WHEEL_DIAMETER_MIN))wheelDiameter=WHEEL_DIAMETER_DEFAULT;
		gearRatio = eeprom_read_float((float*)(eepStartAddress+=eepAddrShift));
		if ((gearRatio>GEAR_RATIO_MAX)||(gearRatio<GEAR_RATIO_MIN))gearRatio=GEAR_RATIO_DEFAULT;
		degreesPerKmh = eeprom_read_float((float*)(eepStartAddress+=eepAddrShift));
		if ((degreesPerKmh>DEGREES_PER_KMH_MAX)||(degreesPerKmh<DEGREES_PER_KMH_MIN))degreesPerKmh=DEGREES_PER_KMH_DEFAULT;
	}
	else if (eepAction==EEP_ODOMETER_READ){
		uint32_t tempMilage = 0;
		odometerCurrentAddress = 0;
		for (uint8_t i = 0;i<=ODOMETER_EEP_CELLS;i++){
			tempMilage = eeprom_read_dword((uint32_t*)(eepStartAddress+(eepAddrShift*i)));
			if(tempMilage>milage){	//finding the latest record (the highest value)
				milage = tempMilage;
				odometerCurrentAddress = i + 1;	//address for the next record
				if (odometerCurrentAddress>ODOMETER_EEP_CELLS)odometerCurrentAddress = 0;
			}
		}
	}
	else if (eepAction==EEP_ODOMETER_WRITE){
			eeprom_write_dword((uint32_t*)(eepStartAddress+(odometerCurrentAddress*eepAddrShift)),milage);
			odometerCurrentAddress++;
			if (odometerCurrentAddress>ODOMETER_EEP_CELLS)odometerCurrentAddress = 0;
	}
}
uint16_t set_value (uint16_t maxValue, uint16_t minValue, uint16_t currValue, uint8_t tens, const char *text){
	GLCD_Clear();
	GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
	GLCD_GotoLine(1);
	GLCD_GotoX(10);
	GLCD_PrintString(text);
	GLCD_GotoXY(90,24);
	GLCD_PrintString("Edit");
	GLCD_GotoXY(90,36);
	GLCD_PrintString("Exit");
	GLCD_GotoXY(90,48);
	GLCD_PrintString("Save");
	void print_min_max(){
			GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
			GLCD_GotoXY(6,41);
			GLCD_PrintString("min:");
			GLCD_GotoXY(6,51);
			GLCD_PrintString("max:");
			if (tens){
				uint16_t devider = 1;
				for (int8_t i = 0;i<tens;i++){devider*=10;}
				GLCD_GotoXY(30,41);
				GLCD_PrintDouble(minValue/(double)devider,devider);
				GLCD_GotoXY(30,51);
				GLCD_PrintDouble(maxValue/(double)devider,devider);
				}else{
				GLCD_GotoXY(30,41);
				GLCD_PrintInteger(minValue);
				GLCD_GotoXY(30,51);
				GLCD_PrintInteger(maxValue);
			}
			GLCD_SetFont(Arial12x17, 12, 17, GLCD_Overwrite);
		}
    uint16_t tempValue;
	int8_t digitIndex;
	uint8_t valueLength;
	uint8_t maxValueLength;
	int8_t *digitsArr;
	int8_t currentItem = 0;
restore_initial_value:
	tempValue = currValue;
	valueLength = 0;
	uint16_t new_value (void)	//gathering digits back to the integer
	{
		uint16_t newValue = 0;
		for(digitIndex = maxValueLength - 1;digitIndex>=0;digitIndex--){
			uint16_t tenPower = digitsArr[digitIndex];
			for (uint8_t j = 0;j<digitIndex;j++){
				tenPower*=10;
			}
			newValue += tenPower;
		}
		return newValue;
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
	GLCD_GotoXY(5,20);
	GLCD_SetFont(Arial12x17, 12, 17, GLCD_Overwrite);
	int8_t rectShift = (5+(maxValueLength-1)*13)-currentItem*13;
	for(digitIndex = maxValueLength - 1;digitIndex>=0;digitIndex--){
		GLCD_PrintInteger(digitsArr[digitIndex]);
		if ((digitIndex==tens)&&(tens)){
			GLCD_PrintString(".");
			rectShift+=5;
		}
	}
	GLCD_InvertRect(rectShift,20,rectShift+12,36);
	print_min_max();
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
								uint16_t checkValue = new_value();
								if((checkValue>maxValue)||(checkValue<minValue))goto restore_initial_value;
								int8_t menuItem = 0;
								GLCD_InvertRect(rectShift+5,20,rectShift+5+12,36);
								while(button_monitor());
								while(1){
										currentButton = button_monitor();
										if (currentButton == 3) {
											menuItem--;
											if (menuItem<0)menuItem = 2;
										}
										else if (currentButton == 2) {
											menuItem++;
											if (menuItem>2)menuItem = 0;
										}
										else if (currentButton == 1) {
												if (!menuItem){	//getting back to value edit
													currentItem = 0;
													for (int8_t i = 0;i<3;i++){GLCD_DrawRectangle(86,21+i*12,116,33+i*12,GLCD_White);}
													break;
												}
												else if (menuItem == 1){
													return currValue; //if changes discarded - return initial value
												}
												else if (menuItem == 2){
													return new_value();
												}
										}
							for (int8_t i=0;i<3;i++)GLCD_DrawRectangle(86,21+i*12,116,33+i*12,GLCD_White);
							GLCD_DrawRectangle(86,21+menuItem*12,116,33+menuItem*12,GLCD_Black);
							GLCD_Render();
							while(button_monitor());
							while(!button_monitor());
							}
						}
					}
					if (currentButton == 3) {
						digitsArr[currentItem]++;
						if (digitsArr[currentItem]>9)digitsArr[currentItem] = 0;
					}
					else if (currentButton == 2) {
						digitsArr[currentItem]--;
						if (digitsArr[currentItem]<0)digitsArr[currentItem] = 9;
					}
			GLCD_GotoXY(5,20);
			for(digitIndex = maxValueLength - 1;digitIndex>=0;digitIndex--){
				GLCD_PrintInteger(digitsArr[digitIndex]);
				if ((tens)&&(digitIndex==tens)) {GLCD_PrintString(".");}
			}
			rectShift = (5+(maxValueLength-1)*13)-currentItem*13;
			if ((tens)&&(currentItem<tens))	rectShift+=5;
			GLCD_InvertRect(rectShift,20,rectShift+12,36);
			print_min_max();
			GLCD_Render();
		}
		while(button_monitor());
		while(!button_monitor());
	}
}
void debug_screen(){
	uint8_t xOfset1 = 4;
	uint8_t xOfset2 = 26;
	uint8_t xOfset3 = 60;
	uint8_t xOfset4 = 86;
	GLCD_Clear();
	GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
	GLCD_GotoLine(0);
	GLCD_GotoX(xOfset1);
	GLCD_PrintString("Spd");
	GLCD_GotoX(xOfset2);
	GLCD_PrintDouble(speedKmh,10);
	
	GLCD_GotoX(xOfset3);
	GLCD_PrintString("Ubt");
	GLCD_GotoX(xOfset4);
	GLCD_PrintDouble(newVoltage/10.0,100);
	
	GLCD_GotoLine(1);
	GLCD_GotoX(xOfset1);
	GLCD_PrintString("Frq");
	GLCD_GotoX(xOfset2);
	GLCD_PrintDouble(frequency,10);
	
	GLCD_GotoX(xOfset3);
	GLCD_PrintString("StP");
	GLCD_GotoX(xOfset4);
	GLCD_PrintInteger(speedTimerPrecise);
	
	GLCD_GotoLine(2);
	GLCD_GotoX(xOfset1);
	GLCD_PrintString("Stp");
	GLCD_GotoX(xOfset2);
	GLCD_PrintInteger(steps);
	
	GLCD_GotoX(xOfset3);
	GLCD_PrintString("StR");
	GLCD_GotoX(xOfset4);
	GLCD_PrintInteger(speedTimerRough);
	
	GLCD_GotoLine(3);
	GLCD_GotoX(xOfset1);
	GLCD_PrintString("nSt");
	GLCD_GotoX(xOfset2);
	GLCD_PrintInteger(newSteps);
	
	GLCD_GotoX(xOfset3);
	GLCD_PrintString("tRP");
	GLCD_GotoX(xOfset4);
	GLCD_PrintInteger(speedTimerRoughPrevious);
	
	GLCD_GotoLine(4);
	GLCD_GotoX(xOfset1);
	GLCD_PrintString("Phs");
	GLCD_GotoX(xOfset2);
	GLCD_PrintInteger(phase);
	
	GLCD_GotoX(xOfset3);
	GLCD_PrintString("Trn");
	GLCD_GotoX(xOfset4);
	GLCD_PrintString("L");
	GLCD_PrintInteger(!(PIN_LEFT));
	GLCD_PrintString(" R");
	GLCD_PrintInteger(!(PIN_RIGHT));
	
	GLCD_GotoLine(5);
	GLCD_GotoX(xOfset1);
	GLCD_PrintString("SAc");
	GLCD_GotoX(xOfset2);
	GLCD_PrintInteger(sensorActations);
	
	GLCD_GotoLine(6);	
	GLCD_GotoX(xOfset1);
	GLCD_PrintString("Mil");
	GLCD_GotoX(xOfset2);
	GLCD_PrintDouble(milage/1000.0,100);
 	GLCD_Render();
}