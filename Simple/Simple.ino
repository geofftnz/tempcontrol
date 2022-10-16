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
#define TEMP_OFF (25<<2)

// temp to start the fan
#define TEMP_START (28<<2)

// bitshift amount to scale fan speed with temp
#define FAN_SCALE 2

// minimum PWM value
#define FAN_MIN 64

// resolution of temp sensor
#define TEMP_BITS 10

#define PWM_SHIFT 3
#define PWM_MASK 0x1f

// these will need to be defined based on output pin. These are writing to port B output 0, which is pin 5 on the ATTiny85
#define FAN_ON PORTB |= 0x01
#define FAN_OFF PORTB &= ~0x01

// how often to check temperature. When the task counter masked with this hits zero, the temperature will be checked. 0x0fff is about 1 sec period.
#define TASK_COUNTER_MASK 0x3fff


// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
DeviceAddress addr;

byte current = 0;
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

  setFanSpeed(255);

  led(1,0,0);  delay(100);
  led(1,1,0);  delay(100);
  led(0,1,0);  delay(100);
  led(0,1,1);  delay(100);
  led(0,0,1);  delay(100);
  led(1,0,1);  delay(100);
  led(0,0,0);

  setFanSpeed(0);
  requestTemperature();
}

void setFanSpeed(byte newspeed)
{
  // set LEDs
  if (!newspeed) led(0,0,0);
  else if (newspeed<=64) led(0,1,0);
  else if (newspeed<=128) led(1,1,0);
  else led(1,0,0);

  newspeed >>= PWM_SHIFT;
  // give a stopped fan a kick-start
  if (newspeed && !current){
      FAN_ON;
      delay(500);
      FAN_OFF;
  }
  
  current = newspeed;
}


void processTemp(int16_t temp)
{
  if (temp < TEMP_OFF){
    setFanSpeed(0);    
  }
  if (temp >= TEMP_START){
    int16_t s = (temp - TEMP_START) << FAN_SCALE;
    if (s<0) s = 0;
    if (s>(255-FAN_MIN)) s = (255-FAN_MIN);
    if (s) s+= FAN_MIN;
    setFanSpeed((byte)s);
  }
}


/*
 * Main function, get and show the temperature
 */
byte state = 0;
void loop(void)
{   
  // process pwm
  byte pwm_counter_masked = pwm_counter & PWM_MASK;

  // if our pwm counter has reached our fan setting, then turn off
  if (pwm_counter_masked == current)
  {
      FAN_OFF;
  }

  // if our current fan speed value is not zero, and we're at the start of our pwm cycle, then turn on
  if (current && pwm_counter_masked == 0)
  {
      FAN_ON;
  }

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
              Serial.println(current);
              #endif
          } 
          requestTemperature();
      }
    }
  }
    
  pwm_counter++;
}
