// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>

//#define SERIAL_ENABLE

/*
// Arduino Nano pins
#define ONE_WIRE_BUS 4
#define PWM_OUT 3
#define LED_RED 6
#define LED_GREEN 7
#define LED_BLUE 8
*/

// ATTiny85 pins
#define ONE_WIRE_BUS 4
#define PWM_OUT 0
#define LED_RED 3
#define LED_GREEN 2
#define LED_BLUE 1

// temp to turn off fan
#define TEMP_OFF (27<<2)

// temp to start the fan
#define TEMP_START (29<<2)

// bitshift amount to scale fan speed with temp (where temp is <<2)
#define FAN_SCALE 1

// minimum PWM value (note that this will get shifted right by PWM_SHIFT)
#define FAN_MIN 24

// resolution of temp sensor
#define TEMP_BITS 10

#define PWM_SHIFT 3
#define PWM_MASK 0x1f

#define SETPORT(x) PORTB = (PORTB & 0xF0) | ((x) & 0x0F)

byte portbstate[4]={
  0x0E, // Idle: blue led off
  0x0C, // Idle: blue led on
  0x0A, // Running: fan off, green led on    
  0x07  // Running: fan on, red led on  
};

#define IDLE0 portbstate[0]
#define IDLE1 portbstate[1]
#define FAN0 portbstate[2]
#define FAN1 portbstate[3]

// states, used as indexes into pairs of values in the portbstate array
#define STATE_IDLE 0
#define STATE_RUN 2
byte state = STATE_IDLE;

// how often to check temperature. When the task counter masked with this hits zero, the temperature will be checked. 0x0fff is about 1 sec period.
#define TASK_COUNTER_MASK 0x3fff


// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
DeviceAddress addr;

byte pwm_level = 0; // point at which pwm switched from on to off, within the range set by PWM_MASK.

int16_t millis_to_wait = 750;
unsigned long temperature_read_time_ms = 0;
byte pwm_counter = 0;
unsigned int task_counter = 0;

void led(byte r, byte g, byte b){
  digitalWrite(LED_RED,1-r);
  digitalWrite(LED_GREEN,1-g);
  digitalWrite(LED_BLUE,1-b);
}

void requestTemperature()
{
  sensors.requestTemperaturesByAddress(addr);
  temperature_read_time_ms = millis() + millis_to_wait;
}

bool readyToReadTemperature()
{
  return millis() > temperature_read_time_ms;
}

/*
 * The setup function. We only start the sensors here
 */
void setup(void)
{
  #ifdef SERIAL_ENABLE
  Serial.begin(57600);
  #endif

  // Start up the library
  sensors.begin();

  sensors.getAddress(addr,0);
  sensors.setResolution(addr,TEMP_BITS,true);
  sensors.setWaitForConversion(false);  // async mode, but we have to handle our own timing.
  millis_to_wait = sensors.millisToWaitForConversion(TEMP_BITS);

  pinMode(PWM_OUT,OUTPUT);
  pinMode(LED_RED,OUTPUT);
  pinMode(LED_GREEN,OUTPUT);
  pinMode(LED_BLUE,OUTPUT);

  SETPORT(FAN1);  // fan on
  led(1,0,0);  delay(100);
  led(1,1,0);  delay(100);
  led(0,1,0);  delay(100);
  led(0,1,1);  delay(100);
  led(0,0,1);  delay(100);
  led(1,0,1);  delay(100);
  led(0,0,0);
  SETPORT(FAN0); // fan off
  
  requestTemperature();
}


void processTemp(int16_t temp)
{
  byte newState = state;

  switch(state)
  {
    case STATE_IDLE:
      // if we've exceeded our turn-on temp, start the fan.
      if (temp >= TEMP_START){

        // kickstart the fan
        SETPORT(FAN1);  //red + fan
        delay(1000);
        SETPORT(0x08);  // blue + green while we wait for initial temp adjust
        
        // set the pwm to minimum
        pwm_level = FAN_MIN >> PWM_SHIFT;
        newState = STATE_RUN;
      }
    break;
    
    case STATE_RUN:

      // if we've dropped below our turn-off temp, revert to idle state
      if (temp <= TEMP_OFF){
        newState = STATE_IDLE;
        pwm_level = 1;
      }
      else{
        // calculate fan speed 
        int16_t s = (((temp - TEMP_OFF) << FAN_SCALE) + FAN_MIN);
        if (s<0) s = 0;
        if (s>255) s = 255;
        pwm_level = (s >> PWM_SHIFT) & PWM_MASK;
      }
    
    break;
  }

  state = newState;
}


/*
 * Main function, get and show the temperature
 */
void loop(void)
{   
  // process pwm
  byte pwm_counter_masked = pwm_counter & PWM_MASK;

  // if our pwm counter has reached our pwm level, then turn off...
  if (pwm_counter_masked == pwm_level)
  {
      SETPORT(portbstate[state]);
  }
  // otherwise if we've wrapped around to zero, turn on. Note that if pwm_level == 0, we wont reach this point.
  else if (pwm_counter_masked == 0)
  {
      SETPORT(portbstate[state+1]);
  }

  // increment task counter when pwm_counter hits zero
  if (!pwm_counter)
  {
    task_counter++;
    task_counter &= TASK_COUNTER_MASK;
    if (task_counter == 0)
    {
      if (readyToReadTemperature())
      {
          int16_t temp = sensors.getTemp(addr) >> 5;
          
          if(temp != DEVICE_DISCONNECTED_RAW) 
          {
              processTemp(temp);
              
              #ifdef SERIAL_ENABLE
              Serial.print("temp,");
              Serial.print(temp>>2);
              Serial.print(",fan,");
              Serial.println(pwm_level);
              #endif
          } 
          requestTemperature();
      }
    }
  }
    
  pwm_counter++;
}
