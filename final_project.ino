#include <LiquidCrystal.h>
#include <RTClib.h>
#include <DHT.h>
#include <Stepper.h>

//TODO figure out port stuff and fix it in the main code too

//checked
// define pins for the stepper motor
#define STEPPER_PIN1 10
#define STEPPER_PIN2 11
#define STEPPER_PIN3 12
#define STEPPER_PIN4 13

// keep this set to 18 as it is one of the pins avalible for interputs
#define BUTTON_ON_OFF 0

// when in error need to be able to reset to idle
#define BUTTON_RESET 1

// define pins for the buttons
#define BUTTON_LIMIT_TOP 2
#define BUTTON_LIMIT_BOTTOM 3


#define BUTTON_STEPPER_UP 4
#define BUTTON_STEPPER_DOWN 5

// fan motor
#define MOTOR_PIN 6

// humidity and temp sensor pins
#define DHT_PIN 7
#define DHT_TYPE DHT11

//checked
#define WATER_LEVEL 5

// define led pins
#define LED_PINR 3
#define LED_PINY 3
#define LED_PING 5
#define LED_PINB 5

#define RDA 0x80
#define TBE 0x20  

//checked
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(30, 31, 32, 33, 34, 35);

//checked
// clock
RTC_DS1307 RTC;

// temp humidity
DHT dht(DHT_PIN, DHT_TYPE);

// checked
// create a Stepper object
Stepper stepper(2048, STEPPER_PIN1, STEPPER_PIN2, STEPPER_PIN3, STEPPER_PIN4);

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

// define the possible states for the state machine
enum States {
  IDLE,
  DISABLED,
  RUNNING,
  ERROR,
  START,
};


#define WATER_THRESHOLD 320 // the threshold for that sensor
#define TEMP_THRESHOLD 10 // the temp threshold for that sensor

// define the initial state for the state machine
States currentState = DISABLED;
States prevState = START;

void setup()
{
  
  // initialize the pins used for I2C communication with the RTC module
  PORTD |= (1 << PD0) | (1 << PD1);
  RTC.begin();

  // set the time on the RTC module
  DateTime now = DateTime(2022, 12, 9, 0, 0, 0);
  RTC.adjust(now);

  DDRE |= (0x01 << LED_PING | 0x01 << LED_PINY);
  DDRG |= (0x01 << LED_PINB);
  DDRH |= (0x01 << LED_PINR | 0x01 << WATER_LEVEL | 0x01 << MOTOR_PIN);


  // init read stuff
  adc_init();

  // initialize the serial port on USART0:
  U0init(9600);

  dht.begin();
  
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.print("System Starting");
  

  // limit switch button setting
  DDRA &= ~(0x01 << BUTTON_RESET | 0x01 << BUTTON_ON_OFF | 0x01 << BUTTON_LIMIT_TOP | 0x01 << BUTTON_LIMIT_BOTTOM | 0x01 << BUTTON_STEPPER_UP | 0x01 << BUTTON_STEPPER_DOWN);
  

  
  // attach an interrupt to the button pin
  attachInterrupt(digitalPinToInterrupt(BUTTON_ON_OFF), handleOnOff, RISING);
  stepper.setSpeed(10);
}


// loop update values
int lastTempPrint = 0;
float temp = 0;
float hum = 0;

// speed of stepper motor
int stepperRate = 2048;

// state machine flags 
bool fanOn = false;
int ledC = -1;
bool displayTempHum = false;
bool stepperAllowed = false;
bool monitorWater = false;


void loop()
{
  // read the time from the RTC module
  DateTime now = RTC.now();
  
  if(displayTempHum){
    // read the temperature and humidity from the DHT11 sensor
    temp = dht.readTemperature();
    hum = dht.readHumidity();
  }

  currentState = decideState(temp, hum, currentState);

  if(pinRead(BUTTON_ON_OFF)){
    handleOnOff();
  }
  // when state is switching exec
  if(currentState != prevState){
    
    writeTimeStampTransition(now, prevState, currentState);
    
    // check the current state of the state machine
    switch (currentState) {
      case DISABLED:
        fanOn = false;
        ledC = 3;
        displayTempHum = false;
        stepperAllowed = true;
        monitorWater = false;

        break;

      case IDLE:
        fanOn = false;
        ledC = 2;
        displayTempHum = true;
        stepperAllowed = true;
        monitorWater = true;

        break;

      case RUNNING:
        fanOn = true;
        ledC = 1;
        displayTempHum = true;
        stepperAllowed = true;
        monitorWater = true;

        break;

      case ERROR:
        lcd.clear();
        lcd.print("Error, low water level");
        fanOn = false;
        ledC = 0;
        displayTempHum = true;
        stepperAllowed = false;
        monitorWater = true;


        break;
      case START:
        break;
    }
  }
  
  // set stepper rate
  if(stepperAllowed){
    //determine direction the user wants, the limits it can go, then set that speed
    int stepperDirection = stepperRate * (pinRead(BUTTON_STEPPER_UP) ? 1 : pinRead(BUTTON_STEPPER_DOWN) ? -1 : 0);
    //preform limit checks
    stepperDirection = (pinRead(BUTTON_LIMIT_TOP) ? min(stepperDirection, 0) : (pinRead(BUTTON_LIMIT_BOTTOM) ? max(stepperDirection ,0) : stepperDirection));
    if(stepperDirection != 0){
      writeStepperPos(now, prevState, currentState);
    }
    setStepperMotor(stepperDirection);
  }
  // set fan rate
  setFanMotor(fanOn);

  // set led
  turnLEDOn(ledC);

  // display temp hum, if we are allowed to, and if the time since the last one has been greater then one minute
  if(displayTempHum && abs(lastTempPrint - now.minute()) >= 1){
    
    lcd.clear();
    lastTempPrint = now.minute(); // update prev
    temp = dht.readTemperature();
    hum = dht.readHumidity();
    lcd.print("Temp, Humidity");
    delay(1000);
    lcd.clear();
    lcd.print(temp); // write temp to lcd
    lcd.print(hum); // write hum to lcd
  }

  prevState = currentState;
  // check water level, if allowed to
  
  
  if(monitorWater){
    int waterLvl = adc_read(WATER_LEVEL); // calc water lvl
    if(waterLvl <= WATER_THRESHOLD){
      currentState = ERROR;
    }
  }
  
  delay(500);

 
}

int waterRead(int pin){
  return PINH & (0x01 << pin);
}

int pinRead(int pin) {
  // read the value of the pin using the PIN register
  return PINA & (0x01 << pin);
}


bool buttonOnState = true;

// if handle on off button is pressed this will automatically be called
void handleOnOff(){
  prevState = currentState;
  bool bPressed = pinRead(BUTTON_ON_OFF);
  if(buttonOnState && bPressed){
    currentState = IDLE;
    buttonOnState = false;
  }else if(bPressed) {
    currentState = DISABLED;
    buttonOnState = true;
  }
}

//write simple msgs over serial one char at a time
void writeStepperPos(DateTime now, States prevState, States currentState){
  U0putchar('S');
  U0putchar('T');
  U0putchar('E');
  U0putchar('P');

  
  U0putchar(' ');
  writeTimeStampTransition(now, prevState, currentState);
}

//write simple msgs over serial one char at a time
void writeTimeStampTransition(DateTime now, States prevState, States currentState){
  unsigned char pState = (prevState == DISABLED ? 'd' : (prevState == IDLE ? 'i' : (prevState == RUNNING ? 'r' : (prevState == ERROR ? 'e' : 'u'))));
  unsigned char cState = (currentState == DISABLED ? 'd' : (currentState == IDLE ? 'i' : (currentState == RUNNING ? 'r' : (currentState == ERROR ? 'e' : 'u')))); 
  
  U0putchar(pState);
  U0putchar(':');
  U0putchar(cState);

  U0putchar(' ');

  int year = now.year();
  int month = now.month();
  int day = now.day();
  int hour = now.hour();
  int minute = now.minute();
  int second = now.second();
  char numbers[10] = {'0','1','2','3','4','5','6','7','8','9'};
  int onesYear = year % 10;
  int tensYear = year / 10 % 10;
  int onesMonth = month % 10;
  int tensMonth = month / 10 % 10;
  int onesDay = day % 10;
  int tensDay = day / 10 % 10;
  int onesHour = hour % 10;
  int tensHour = hour / 10 % 10;
  int onesMinute = minute % 10;
  int tensMinute = minute / 10 % 10;
  int onesSecond = second % 10;
  int tensSecond = second / 10 % 10;
  
  U0putchar('M');
  U0putchar(':');
  U0putchar('D');
  U0putchar(':');
  U0putchar('Y');

  U0putchar(' ');
  
  U0putchar('H');
  U0putchar(':');
  U0putchar('M');
  U0putchar(':');
  U0putchar('S');

  U0putchar(' ');

  U0putchar(numbers[tensMonth]);
  U0putchar(numbers[onesMonth]);
  U0putchar(':');
  U0putchar(numbers[tensDay]);
  U0putchar(numbers[onesDay]);
  U0putchar(':');
  U0putchar(numbers[tensYear]);
  U0putchar(numbers[onesYear]);
  
  U0putchar(' ');

  U0putchar(numbers[tensHour]);
  U0putchar(numbers[onesHour]);
  U0putchar(':');
  U0putchar(numbers[tensMinute]);
  U0putchar(numbers[onesMinute]);
  U0putchar(':');
  U0putchar(numbers[tensSecond]);
  U0putchar(numbers[onesSecond]);

  U0putchar('\n');
}

States decideState(float temp, int waterLvl, States currentState){
  States state;
  if(temp <= TEMP_THRESHOLD && currentState == RUNNING){
    state = IDLE;
  }else if(temp > TEMP_THRESHOLD && currentState == IDLE){
    state = RUNNING;
  }else if(currentState == ERROR && pinRead(BUTTON_RESET) && waterLvl > WATER_THRESHOLD){
    state = IDLE;
  }else{
    state = currentState;
  }

  return state;
}

// method to turn on the specified LED and turn off the other LEDs
void turnLEDOn(int ledPin) {
  // turn off all of the LEDs using a bitwise AND operation
  PORTH &= ~(0x01 << LED_PINR);
  PORTG &= ~(0x01 << LED_PINB);
  PORTE &= ~(0x01 << LED_PING);
  PORTE &= ~(0x01 << LED_PINY);

  // turn on the specified LED using a bitwise OR operation
  switch (ledPin) {
    case 0:
      PORTH |= 0x01 << LED_PINR;
      break;
    case 1:
      PORTG |= 0x01 << LED_PINB;
      break;
    case 2:
      PORTE |= 0x01 << LED_PING;
      break;
    case 3:
      PORTE |= 0x01 << LED_PINY;
      break;
  }
}


// sets the stepper speed
void setStepperMotor(int dist){
  stepper.step(dist);
}

// turn fan motor on or off
void setFanMotor(bool on){
  if(on){
    PORTH |= (0x01 << MOTOR_PIN);
  }else {
    PORTH &= ~(0x01 << MOTOR_PIN);
  }
}


//---------------start analog read-----------------
void adc_init(){
     ADCSRA = 0x80;
     ADCSRB = 0x00;
     ADMUX = 0x40;
}

unsigned int adc_read(unsigned char adc_channel){
     ADCSRB &= 0xF7; // Reset MUX5.
     ADCSRB |= (adc_channel & 0x08); // Set MUX5.
     ADMUX &= 0xF8; // Reset MUX2:0.
     ADMUX |= (adc_channel & 0x07); // Set MUX2:0.

     ADCSRA |= 0x40; // Start the conversion.
     while (ADCSRA & 0x40) {} // Converting...
     return ADC; // Return the converted number.
}

//---------------end analog read-----------------

//---------------start serial print-----------------
// function to initialize USART0 to "int" Baud, 8 data bits,
// no parity, and one stop bit. Assume FCPU = 16MHz.
//
void U0init(unsigned long U0baud)
{
//  Students are responsible for understanding
//  this initialization code for the ATmega2560 USART0
//  and will be expected to be able to intialize
//  the USART in differrent modes.
//
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
//
// Read USART0 RDA status bit and return non-zero true if set
//
unsigned char U0kbhit()
{
  return (RDA & *myUCSR0A);
}
//
// Read input character from USART0 input buffer
//
unsigned char U0getchar()
{
  return *myUDR0;
}
//
// Wait for USART0 TBE to be set then write character to
// transmit buffer
//
void U0putchar(unsigned char U0pdata)
{
  while(!(TBE & *myUCSR0A));
  *myUDR0 = U0pdata;
}
//---------------end serial print-----------------
