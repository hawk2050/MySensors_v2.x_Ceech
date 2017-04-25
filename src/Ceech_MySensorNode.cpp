/*
* Ceech_MySensorNode.cpp - Firmware for Ceech board temperature and humidity,
* pressure sensor Node with nRF24L01+ module
*
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, see <http://www.gnu.org/licenses/>.
*
* Authors:
* Tomas Hozza <thozza@gmail.com>
* Richard Clarke <richard.ns@clarke.biz>
*
* MySensors library - http://www.mysensors.org/
* Ceech board -
* nRF24L01+ spec - https://www.sparkfun.com/datasheets/Wireless/Nordic/nRF24L01P_Product_Specification_1_0.pdf
*
Hardware Connections (Breakoutboard to Arduino):
 -VCC = 3.3V
 -GND = GND
 -SDA = A4 (use inline 10k resistor if your board is 5V)
 -SCL = A5 (use inline 10k resistor if your board is 5V)

Ceech board compatible with Arduino PRO Mini 3.3V@8MHz

System Clock  = 8MHz

Information on porting a 1.5.x compatible sketch to 2.0.x

https://forum.mysensors.org/topic/4276/converting-a-sketch-from-1-5-x-to-2-0-x/2

 */

 // Enable debug prints to serial monitor
 #define MY_DEBUG
 #define DEBUG_RCC 1

 // Enable and select radio type attached
 #define MY_RADIO_NRF24
 //#define MY_RADIO_RFM69

 #define MY_NODE_ID 10
 /*Makes this static so won't try and find another parent if communication with
 gateway fails*/
 #define MY_PARENT_NODE_ID 0
 #define MY_PARENT_NODE_IS_STATIC

 /*These are actually the default pins expected by the MySensors framework.
  * This means we can use the default constructor without arguments when
  * creating an instance of the Mysensors class. Other defaults will include
  * transmitting on channel 76 with a data rate of 250kbps.
  */
// DHT Data wire is plugged into port 2 on the Arduino
#define HUMIDITY_SENSOR_DIGITAL_PIN 2
#define ONE_WIRE_BUS 5

#define MY_RF24_CE_PIN 7
#define MY_RF24_CS_PIN 8
#define MY_RF24_CHANNEL 76

#include <Arduino.h>
#include <MySensors.h>

// FORCE_TRANSMIT_INTERVAL, this number of times of wakeup, the sensor is forced to report all values to
// the controller
#define FORCE_TRANSMIT_INTERVAL 3
//#define SLEEP_TIME 300000
#define SLEEP_TIME 5000

#define CHILD_ID_DHT22_HUMIDITY 0
#define CHILD_ID_DHT22_TEMP 1
#define CHILD_ID_VOLTAGE 2

#define CHILD_ID_BMP180_PRESSURE 3
#define CHILD_ID_BMP180_TEMP 4

#define CHILD_ID_DALLAS_TEMP_BASE 5

#define MAX_ATTACHED_DS18B20 2
/**************************************************/
/********* Sensor Messages and FUNCTIONS *********/
/*************************************************/

#include <SFE_BMP180.h>
#include <Wire.h>
SFE_BMP180 pressure;
MyMessage msgBmp180Press(CHILD_ID_BMP180_PRESSURE, V_PRESSURE);
MyMessage msgBmp180Temp(CHILD_ID_BMP180_TEMP, V_TEMP);
void readBMP180TempAndPressure(bool force);

#include <OneWire.h>
#include <DallasTemperature.h>
float lastTemperature[2];
int numSensors=0;
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature dallas_sensor(&oneWire);
// Initialize temperature message
MyMessage msgDallas(CHILD_ID_DALLAS_TEMP_BASE, V_TEMP);
void readDS18B20(bool force);


#include "DHT22.h"
DHT22 dht(HUMIDITY_SENSOR_DIGITAL_PIN);
MyMessage msgHum(CHILD_ID_DHT22_HUMIDITY, V_HUM);
MyMessage msgTemp(CHILD_ID_DHT22_TEMP, V_TEMP);
void readDHTHumidityAndTemperature(bool force);


/**************************************************/
/************ Utility Functions *******************/
/**************************************************/


boolean receivedConfig = false;
boolean metric = true;
uint8_t loopCount = 0;


/**********************************/
/********* IMPLEMENTATION *********/
/**********************************/
/*If you need to do initialization before the MySensors library starts up,
define a before() function */
void before()
{

}

/*To handle received messages, define the following function in your sketch*/
void receive(const MyMessage &message)
{
  /*Handle incoming message*/
}

/* If your node requests time using requestTime(). The following function is
used to pick up the response*/
void receiveTime(unsigned long ts)
{
}

/*You can still use setup() which is executed AFTER mysensors has been
initialised.*/
void setup()
{
  analogReference(INTERNAL);
  dallas_sensor.begin();
  Serial.print("Locating devices...");
  Serial.print("Found ");
  numSensors = dallas_sensor.getDeviceCount();

  #ifdef DEBUG_RCC
  Serial.print(numSensors, DEC);
  Serial.println(" devices.");
  #endif

  if (pressure.begin())
  {
    Serial.println("BMP180 init success");
  }
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.

    Serial.println("BMP180 init fail\n\n");
    while(1); // Pause forever.
  }

}

void presentation()
{
   // Send the sketch version information to the gateway and Controller
   // Send the sketch version information to the gateway and Controller
  sendSketchInfo("ceech-temp-hum-pressure", "0.5");
   // Register all sensors to gateway (they will be created as child devices)
   // Present all sensors to controller
  present(CHILD_ID_DHT22_HUMIDITY, S_HUM);
  present(CHILD_ID_BMP180_PRESSURE, S_BARO);
  present(CHILD_ID_BMP180_TEMP, S_TEMP);
  present(CHILD_ID_DALLAS_TEMP_BASE, S_TEMP);


}


void loop()
{

  bool forceTransmit;

  loopCount++;
  forceTransmit = false;


  if (loopCount > FORCE_TRANSMIT_INTERVAL)
  { // force a transmission
    forceTransmit = true;
    loopCount = 0;
  }

  readDS18B20(forceTransmit);
  wait(1000);

  readDHTHumidityAndTemperature(forceTransmit);
  wait(1000);

  readBMP180TempAndPressure(forceTransmit);

  wait(SLEEP_TIME);


}//loop()


#define P_LIMIT_HI 1050.0
#define P_LIMIT_LO 900.0
void readBMP180TempAndPressure(bool force)
{
  char status;
  double T,P;
  static double lastP = 1000.0;

  // You must first get a temperature measurement to perform a pressure reading.

  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    wait(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3, higher numbers are slower, higher-res outputs..
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        wait(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);

        /*Filter value to get rid of the occasional erroneous large value*/
        if(P > P_LIMIT_HI || P < P_LIMIT_LO)
        {
          P = lastP;
        }

       lastP = P;

        if (status != 0)
        {
          send(msgBmp180Temp.set(T,1));
          send(msgBmp180Press.set(P,1));
          #if DEBUG_RCC
          Serial.print("BMP180 Temperature:");
          Serial.print(T, 1);
          Serial.print("C");
          Serial.println();
          Serial.print("BMP180 Pressure:");
          Serial.print(P, 1);
          Serial.print("mb");
          Serial.println();
          #endif

        }
      }
    }
  }
}

void readDHTHumidityAndTemperature(bool force)
{
  //static float lastTemp = 0;
  static float lastHumidity = 0;
  DHT22_ERROR_t errorCode;
  float humidity;

  if (force)
  {
    //lastTemp = -100.0;
    lastHumidity = -100.0;
  }

  errorCode = dht.readData();
  if (errorCode == DHT_ERROR_NONE)
  {
    humidity = dht.getHumidity();
    #ifdef DEBUG_RCC
    Serial.print("Got DHT22 Data ");
    Serial.print(dht.getTemperatureC());
    Serial.print("C ");
    Serial.print(humidity,1);
    Serial.println("%");
    #endif
  }
  else
  {
    humidity = lastHumidity;
    #ifdef DEBUG_RCC
    Serial.println("DHT read error ");
    #endif
  }

  //float temperature = dht.getTemperatureC();

  if(!isnan(humidity))
  {
    if(lastHumidity != humidity)
    {
      send(msgHum.set(humidity,1));
      lastHumidity = humidity;
    }
  }

}


void readDS18B20(bool force)
{
  if (force)
  {
    for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++)
    {
      lastTemperature[i] = -100.0;
    }
  }
  // Fetch temperatures from Dallas sensors
  dallas_sensor.requestTemperatures();

  // Read temperatures and send them to controller
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++)
  {

    // Fetch and round temperature to one decimal
    float temperature = static_cast<float>(static_cast<int>((getControllerConfig().isMetric ? dallas_sensor.getTempCByIndex(i) : dallas_sensor.getTempFByIndex(i)) * 10.)) / 10.;

    // Only send data if temperature has changed and no error
    if (lastTemperature[i] != temperature && temperature != -127.00)
    {

      // Send in the new temperature
      send(msgDallas.setSensor(i).set(temperature,1));
      lastTemperature[i]=temperature;
    }
  }
}
