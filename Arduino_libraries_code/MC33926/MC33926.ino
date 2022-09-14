// Pololu Dual MC33926 Motor Driver Shield
// Code by Paulo Costa, September 2013
// This is not the original driver, this it is based on TimerOne libray
// It has suport for two quadrature encoders
// and serial comunication for remote control

#include <digitalWriteFast.h> //http://code.google.com/p/digitalwritefast/
#include <TimerOne.h> //http://www.pjrc.com/teensy/td_libs_TimerOne.html
#include <Arduino.h>

// constants won't change. Used here to 
// set pin numbers:
const int ledPin =  13;      // the number of the LED pin
const int BUF_SIZE = 64;
const int debugPin = 11;

const int nD2_pin = 4;    // Tri-state disables both outputs of both motor channels when LOW;
                      // toggling resets latched driver fault condition
const int M1_DIR_pin = 7;  // Motor 1 direction input
const int M2_DIR_pin = 8;  // Motor 2 direction input
const int M1_PWM_pin = 9;  // Motor 1 speed input
const int M2_PWM_pin = 10; // Motor 2 speed input
const int nSF_pin = 12;   // Status flag indicator (LOW indicates fault)
const int M1_FB_pin = A0;  // Motor 1 current sense output (approx. 525 mV/A)
const int M2_FB_pin = A1;  // Motor 2 current sense output (approx. 525 mV/A)

const int M1_encoderA_pin = 2;  // Motor 1 encoder A input
const int M1_encoderB_pin = 3;  // Motor 1 encoder B input

const int M2_encoderA_pin = 5;  // Motor 2 encoder A input
const int M2_encoderB_pin = 6;  // Motor 2 encoder B input

const int Delta_VMAX = 500; 

// Variables
byte ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated

// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long interval = 50;           // interval at which to blink (milliseconds)
byte motors_timeout, motors_timeout_count;

int M1_PWM_value = 512;
int M2_PWM_value = 512;

int M1_ant_voltage = 0;
int M2_ant_voltage = 0;

int voltage_M1,voltage_M2, Volts;

uint16_t o1_a = 0;
uint16_t o2_a = 0;
int vel_odo1 = 0;
int vel_odo2 = 0;

int RefVel1 = 0;
float Ierro1 = 0;
float Erro1 = 0;
long RefVel2 = 0;
float Ierro2 = 0;
float Erro2 = 0;


byte encoder1_state, encoder2_state;
volatile uint16_t encoder1_pos, encoder2_pos;

uint16_t prepareOutFrame(char channel)
{
  if (channel == 'i') {         
    return (1);  
  } else if (channel == 'q')  {
    return 2;  
  }
}


void sendOut(char channel)
{
  if (channel == 'i') {         // i: sends A0 and A1 as j
    sendCurrent(); 
  } else if (channel == 'o')  { 
    sendOdometry(); 
  }
}

void sendCurrent(void)
{
  sendChannel('i', analogRead(A0)); 
  sendChannel('j', analogRead(A1)); 
}


void sendOdometry(void)
{
  uint16_t o1, o2;
  // Read odometry atomicaly
  cli();
  o1 = encoder1_pos;
  o2 = encoder2_pos;
  sei();
  vel_odo1 = o1 - o1_a;
  vel_odo2 = o2 - o2_a;
  o1_a = o1;
  o2_a = o2;

  // Send it
  sendChannel('l', o1); 
  sendChannel('r', o2); 
}


void processInFrame(unsigned char channel, uint16_t value)
{
  byte i;
  int voltage;
  if (channel == 'M') {
    voltage_M1 = (value & 0x0FFF);
    if ((value & 0xF000) > 0)
      voltage_M1 = - voltage_M1;
    motors_timeout_count = 0;
  } else if (channel == 'L')  {
    RefVel1 = (value & 0x0FFF);
    if ((value & 0xF000) > 0)
      RefVel1 = - RefVel1;
  } else if (channel == 'R')  {
    RefVel2 = (value & 0x0FFF);
    if ((value & 0xF000) > 0)
      RefVel2 = - RefVel2;
  }
}

static void set_M1_voltage(int new_voltage)
{
  if ((new_voltage-M1_ant_voltage) > Delta_VMAX ) {
    M1_ant_voltage = M1_ant_voltage + Delta_VMAX;
  } else {
    if ((M1_ant_voltage-new_voltage) > Delta_VMAX) {
      M1_ant_voltage = M1_ant_voltage - Delta_VMAX;
    } else {
      M1_ant_voltage = new_voltage;
    }
  }
  if (M1_ant_voltage >= 0) {
    M1_PWM_value = M1_ant_voltage;
    digitalWrite(M1_DIR_pin, 0);
  } else {
    M1_PWM_value = -M1_ant_voltage;
    digitalWrite(M1_DIR_pin, 1);
  }
  Timer1.setPwmDuty(TIMER1_A_PIN, M1_PWM_value);
}


static void set_M2_voltage(int new_voltage)
{
  if ((new_voltage-M2_ant_voltage) > Delta_VMAX ) {
    M2_ant_voltage = M2_ant_voltage + Delta_VMAX;
  } else {
    if ((M2_ant_voltage-new_voltage) > Delta_VMAX) {
      M2_ant_voltage = M2_ant_voltage - Delta_VMAX;
    } else {
      M2_ant_voltage = new_voltage;
    }
  }
  if (M2_ant_voltage >= 0) {
    M2_PWM_value = M2_ant_voltage;
    digitalWrite(M2_DIR_pin, 0);
  } else {
    M2_PWM_value = -M2_ant_voltage;
    digitalWrite(M2_DIR_pin, 1);
  }
  Timer1.setPwmDuty(TIMER1_B_PIN, M2_PWM_value);
}



void timer_interrupt(void)
{
  byte b, new_state;
  static int8_t encoder_table[16] = {0, 1, -1, 0,  -1, 0, 0, 1,  1, 0, 0, -1,  0, -1, 1, 0};
  
  digitalWriteFast(debugPin, 1);
  b = PIND;

  new_state = (b >> 2) & 0x03; // Put encoder channels in the lowest bits
  encoder1_pos += encoder_table[encoder1_state | new_state];
  encoder1_state = new_state << 2;

  new_state = (b >> 5) & 0x03; // Again, Put encoder channels in the lowest bits 
  encoder2_pos += encoder_table[encoder2_state | new_state];
  encoder2_state = new_state << 2;

  digitalWriteFast(debugPin, 0);
}




void setup()
{
  byte i;
  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);      
  pinMode(debugPin, OUTPUT);      

  pinMode(nD2_pin, OUTPUT);      

  pinMode(M1_DIR_pin, OUTPUT);      
  pinMode(M2_DIR_pin, OUTPUT);      

  pinMode(M1_PWM_pin, OUTPUT);      
  pinMode(M2_PWM_pin, OUTPUT);      

  pinMode(M1_encoderA_pin, INPUT_PULLUP);
  pinMode(M1_encoderB_pin, INPUT_PULLUP);

  pinMode(M2_encoderA_pin, INPUT_PULLUP);
  pinMode(M2_encoderB_pin, INPUT_PULLUP);
  
  motors_timeout = 0; 
  motors_timeout_count = 0;  
  
  Timer1.attachInterrupt(timer_interrupt); 
  Timer1.initialize(50); //uS
  Timer1.pwm(TIMER1_A_PIN, M1_PWM_value); 
  Timer1.pwm(TIMER1_B_PIN, M2_PWM_value); 

  initChannelsStateMachine();
  Serial.begin(115200);

  digitalWrite(nD2_pin, 1);
  
  RefVel1 = 0;
  RefVel2 = 0;
}


void loop()
{
  
  if (Serial.available() > 0) {
    byte serialByte = Serial.read();
    channelsStateMachine(serialByte);
    //Serial.write(serialByte);
  }  
  
  unsigned long currentMillis = millis();
 
  if(currentMillis - previousMillis > interval) {
    // save the last time you blinked the LED 
    previousMillis = currentMillis;   

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
    
    sendOdometry();
    sendCurrent();
    //Serial.println();

    Erro1 = RefVel1-vel_odo1;
    Ierro1 = Ierro1 + Erro1;
    Volts = (int)(2*(Erro1 + 0.4*Ierro1));
    if (Volts > 1023) {
      Ierro1 = Ierro1 - Erro1;
      Volts=1023;
    }  
    if (Volts < -1023) {
      Ierro1 = Ierro1 - Erro1;
      Volts = -1023;
    }  
    voltage_M1 = Volts; 
    set_M1_voltage(voltage_M1);
    //set_M1_voltage(500);


    Erro2 = RefVel2-vel_odo2;
    Ierro2 = Ierro2 + Erro2;
    Volts = (int)(2*(Erro2 + 0.4*Ierro2));
    if (Volts > 1023) {
      Ierro2 = Ierro2 - Erro2;
      Volts=1023;
    }  
    if (Volts < -1023) {
      Ierro2 = Ierro2 - Erro2;
      Volts = -1023;
    }  
    voltage_M2 = Volts; 
    set_M2_voltage(voltage_M2);

    /*
    if (motors_timeout > 0){
      motors_timeout_count++;
      if (motors_timeout_count >= motors_timeout){
        set_M1_voltage(0);
        set_M2_voltage(0);
      }
    }
    */
  }
}
