/*
 * 
 * IOT123 ATTINY85 I2C SLAVE AUTO-JOIN LAYER FOR SENSOR: DHT11
 * 
 * Take readings on DHT11 and send across wire on request via I2C in 3 segment 16byte packets
 *  ID of PROPERTY (set in _properties)
 *  VALUE of PROPERTY (set in getProperties)
 *  MORE TO COME (0/1 0 = last property)
 * 
 * Pins on ATTINY85
 * SDA PB0
 * DHT11 PB1
 * SCL PB2
 */

#include <Wire.h> //SDA pin5/PB0, SCL pin7/PB2
#include "DHT.h"

#define arraySize(x)       (sizeof(x) / sizeof(x[0]))
#define NVC_NUM_STAGES 3
#define ADDRESS_SLAVE 10
#define PIN_SENSOR 1

#define TIME_RESPONSE_MS 0 
#if (TIME_RESPONSE_MS)
  unsigned long startMillis; 
#endif

struct nvc
{
    char Name[16];
    char Value[16];
    bool Continue;
};

#define META_COUNT 8
const static char m1[] PROGMEM = "ASSIM_NAME"; 
const static char m2[] PROGMEM = "DHT11"; 
const static char m3[] PROGMEM = "1"; 
const static char m4[] PROGMEM = "ASSIM_VERSION"; 
const static char m5[] PROGMEM = "1"; 
const static char m6[] PROGMEM = "1"; 
const static char m7[] PROGMEM = "ASSIM_ROLE"; 
const static char m8[] PROGMEM = "SENSOR"; 
const static char m9[] PROGMEM = "1"; 
const static char m10[] PROGMEM = "POWER_DOWN"; 
const static char m11[] PROGMEM = "1"; 
const static char m12[] PROGMEM = "1"; 
const static char m13[] PROGMEM = "PREPARE_MS"; 
const static char m14[] PROGMEM = "140000"; 
const static char m15[] PROGMEM = "1"; 
const static char m16[] PROGMEM = "RESPONSE_MS"; 
const static char m17[] PROGMEM = "50"; 
const static char m18[] PROGMEM = "1"; 
const static char m19[] PROGMEM = "MQTT_TOPIC"; 
const static char m20[] PROGMEM = "TEMP_HUMID"; 
const static char m21[] PROGMEM = "1"; 
const static char m22[] PROGMEM = "VCC_MV"; 
const static char m23[] PROGMEM = ""; 
const static char m24[] PROGMEM = "0"; 

const char* const _metas[] PROGMEM = { m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12, m13, m14, m15, m16, m17, m18, m19, m20, m21, m22, m23, m24 };

nvc _props[5] ={
    {"Humidity (%)", "", true},
    {"Temperature (C)", "", true},
    {"Temperature (F)", "", true},
    {"Temperature (K)", "", true},
    {"Dew Point (C)", "", false}
};

volatile int _packetStage = 0;
volatile int _propertyIndex = 0;
volatile bool _metasConfirmed = false;
volatile int _metaIndex = 0;
uint16_t _vcc; 

dht _dht;

void setup()
{
  _vcc = getVcc();
  Wire.begin(8);  
  Wire.onReceive(receiveEvent);                  
  Wire.onRequest(requestEvent);
}

void loop(){}

void receiveEvent (int howMany) 
{
  byte buf[10];
  int i;
  for (i=0; i<howMany; i++)
  {
    buf[i] = Wire.read();     // receive byte as a character
  }
  if((buf[0] == 1) && (howMany == 1)){
    _metasConfirmed = true;
    _packetStage = 0;
    _propertyIndex = 0;
  }
}

void requestEvent() {
  char currentPacket[16];
  int propCount = 0;
  if (_metasConfirmed){ 
    if (_propertyIndex == 0){
// get the sensor properties      
      getProperties();
    }
    strcpy(currentPacket, nvcAsCharArray(_props[_propertyIndex], _packetStage));
    propCount = arraySize(_props);
// METADATA    
  }else{ 
    propCount = META_COUNT;
    if (_metaIndex == 22){// if last metadata (VCC), only runtime entry
      itoa(_vcc, currentPacket, 10);
    }else{ // just a normal metadata item
      //itoa(_metaIndex, currentPacket, 10);
      strcpy_P(currentPacket, (char*)pgm_read_word(&(_metas[_metaIndex]))); 
    }
    _metaIndex++;
  }
  Wire.write(currentPacket); // send metadate or sensor property
  _packetStage = _packetStage + 1;    
  // go to next property if at last stage of current property
  if (_packetStage == NVC_NUM_STAGES){
    _packetStage = 0;
    _propertyIndex++;
  }
  // all properties processed?
  if (_propertyIndex == propCount){
    _propertyIndex = 0;
    // "0" should terminate requests to this slave
  }
}


void getProperties(){
  int chk =  _dht.read11(PIN_SENSOR);
  dtostrf(_dht.humidity,2,2,_props[0].Value);
  dtostrf(_dht.temperature,2,2,_props[1].Value);
  dtostrf(Fahrenheit(_dht.temperature),2,2,_props[2].Value);
  dtostrf(Kelvin(_dht.temperature),2,2,_props[3].Value);
  dtostrf(dewPoint(_dht.temperature, _dht.humidity),2,2,_props[4].Value);
}

char* nvcAsCharArray(nvc nvc, int packetStage){
  switch (packetStage){
    case 0:
      return nvc.Name;
      break;
    case 1: 
        #if (TIME_RESPONSE_MS)
          unsigned long currentMillis;
          currentMillis = millis();
          char millis[16];
          itoa(currentMillis - startMillis, millis, 10);
          return millis;
        #endif
      return nvc.Value;
      break;
    case 2:
      return nvc.Continue ? "1" : "0";
      break;
    default:
      char result[16];
      itoa(packetStage, result, 10);
      return result;
  }
}

// https://www.avrfreaks.net/forum/attiny85-vcc-measurement-skews-higher-vcc-voltages
//5v = 6393, 6504
//3.3V 3430
uint16_t getVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  ADMUX = _BV(MUX3) | _BV(MUX2);
  delay(2); // Wait for Vref to settle
  uint16_t result = 0;
  for (int x = 0; x < 32; x++){
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA,ADSC)); // measuring
    if (x > 15){
      result += (int16_t)((int16_t)(ADC - result) / 2);
    }
    else{
      result = ADC;
    }
  }
  uint16_t voltage = 1125300 / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return voltage;
}

//Celsius to Fahrenheit conversion
double Fahrenheit(double celsius)
{
	return 1.8 * celsius + 32;
}

//Celsius to Kelvin conversion
double Kelvin(double celsius)
{
	return celsius + 273.15;
}

// dewPoint function NOAA
// reference (1) : http://wahiduddin.net/calc/density_algorithms.htm
// reference (2) : http://www.colorado.edu/geography/weather_station/Geog_site/about.htm
//
double dewPoint(double celsius, double humidity)
{
	// (1) Saturation Vapor Pressure = ESGG(T)
	double RATIO = 373.15 / (273.15 + celsius);
	double RHS = -7.90298 * (RATIO - 1);
	RHS += 5.02808 * log10(RATIO);
	RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
	RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
	RHS += log10(1013.246);

        // factor -3 is to adjust units - Vapor Pressure SVP * humidity
	double VP = pow(10, RHS - 3) * humidity;

        // (2) DEWPOINT = F(Vapor Pressure)
	double T = log(VP/0.61078);   // temp var
	return (241.88 * T) / (17.558 - T);
}

// delta max = 0.6544 wrt dewPoint()
// 6.9 x faster than dewPoint()
// reference: http://en.wikipedia.org/wiki/Dew_point
double dewPointFast(double celsius, double humidity)
{
	double a = 17.271;
	double b = 237.7;
	double temp = (a * celsius) / (b + celsius) + log(humidity*0.01);
	double Td = (b * temp) / (a - temp);
	return Td;
}
