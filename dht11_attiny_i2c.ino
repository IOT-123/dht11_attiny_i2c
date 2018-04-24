/*
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
 * RX  PB3
 * TX PB4 
 */

#include <Wire.h> //SDA pin5/PB0, SCL pin7/PB2
#include "DHT.h"

#define USE_SOFTWARE_SERIAL 0
#define LIMIT_PROPERTIES 0  // if memory problems, set this to 1 and just get temp/humidity
#define PIN_DHT 1

#if (USE_SOFTWARE_SERIAL)
  #include <SoftwareSerial.h>
  #define PIN_RX    3   
  #define PIN_TX    4   
  SoftwareSerial mySerial(PIN_RX, PIN_TX);
#endif

dht _dht;
int _packetStage = 0;
int _propertyIndex = 0;

#define MAX_ENTRY_SIZE 16
#define NUM_COLUMNS 3

#if (!LIMIT_PROPERTIES) 
#define NUM_ROWS 5
char _properties[][NUM_COLUMNS][MAX_ENTRY_SIZE] = {
    {"Humidity (%)", "", "1"},
    {"Temperature (C)", "", "1"},
    {"Temperature (F)", "", "1"},
    {"Temperature (K)", "", "1"},
    {"Dew Point (C)", "", "0"},
};
#else
#define NUM_ROWS 2
char _properties[][NUM_COLUMNS][MAX_ENTRY_SIZE] = {
    {"Humidity (%)", "", "1"},
    {"Temperature (C)", "", "0"}
};
#endif

void setup()
{
  Wire.begin(8);                
  Wire.onRequest(requestEvent);
#if (USE_SOFTWARE_SERIAL)
  mySerial.begin(9600);
  mySerial.println("DHT11 TEST PROGRAM ");
  mySerial.print("LIBRARY VERSION: ");
  mySerial.println(DHT11LIB_VERSION);
  mySerial.println();
#endif
}

void loop()
{
  delay(100);
}

void requestEvent() {
  getProperties();
  Wire.write(_properties[_propertyIndex][_packetStage]);
  _packetStage++;
  if (_packetStage == NUM_COLUMNS){
    _packetStage = 0;
    _propertyIndex++;
  }
  if (_propertyIndex == NUM_ROWS){
    _propertyIndex = 0;
    // "0" in "Dew Point (°C)" should terminate requests to this slave
  }
}

void getProperties(){
  int chk =  _dht.read11(PIN_DHT);
#if (USE_SOFTWARE_SERIAL)
  mySerial.println("\n");
  mySerial.println("Read sensor: ");
  switch (chk)
  {
    case DHTLIB_OK: 
    mySerial.println("OK"); 
    break;
    case DHTLIB_ERROR_CHECKSUM: 
    mySerial.println("Checksum error"); 
    break;
    case DHTLIB_ERROR_TIMEOUT: 
    mySerial.println("Time out error"); 
    break;
    default: 
    mySerial.println("Unknown error"); 
    break;
  }
#endif  
  dtostrf(_dht.humidity,2,2,_properties[0][1]);
  dtostrf(_dht.temperature,2,2,_properties[1][1]);
#if (!LIMIT_PROPERTIES)  
  dtostrf(Fahrenheit(_dht.temperature),2,2,_properties[2][1]);
  dtostrf(Kelvin(_dht.temperature),2,2,_properties[3][1]);
  dtostrf(dewPoint(_dht.temperature, _dht.humidity),2,2,_properties[4][1]);
#endif 
#if (USE_SOFTWARE_SERIAL)
  for (int i = 0; i < NUM_ROWS; i++){
    mySerial.print( _properties[i][0]);
    mySerial.print(": ");
    mySerial.println(_properties[i][1]);
  }
#endif
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
