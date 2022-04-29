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


#define TEMP_OFF (25<<2)
#define TEMP_START (28<<2)
#define FAN_SCALE 3
#define FAN_MIN 32


// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
DeviceAddress addr;
byte current = 0;

void led(byte r, byte g, byte b){
  digitalWrite(LED_RED,1-r);
  digitalWrite(LED_GREEN,1-g);
  digitalWrite(LED_BLUE,1-b);
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
  sensors.setResolution(addr,10,true);
  //sensors.setWaitForConversion(false);  // async mode, but we have to handle our own timing.

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
  
}

void setFanSpeed(byte newspeed)
{
  // set LEDs
  if (!newspeed) led(0,0,1);
  else if (newspeed<=64) led(0,1,0);
  else if (newspeed<=128) led(1,1,0);
  else led(1,0,0);

  
  // if we're starting the fan from a complete stop, drive it at full noise for a few cycles 
  if (!current && newspeed){
    analogWrite(PWM_OUT,255);
    delay(1000);
  }
  current = newspeed;
  analogWrite(PWM_OUT,current);
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
void loop(void)
{ 
  sensors.requestTemperaturesByAddress(addr);

  int16_t temp = sensors.getTemp(addr) >> 5;

  // Check if reading was successful
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
}
