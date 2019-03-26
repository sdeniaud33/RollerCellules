/**
   The MySensors Arduino library handles the wireless radio link and protocol
   between your home built sensors/actuators and HA controller of choice.
   The sensors forms a self healing radio network with optional repeaters. Each
   repeater and gateway builds a routing tables in EEPROM which keeps track of the
   network topology allowing messages to be routed to nodes.

   Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
   Copyright (C) 2013-2015 Sensnology AB
   Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors

   Documentation: http://www.mysensors.org
   Support Forum: http://forum.mysensors.org

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   version 2 as published by the Free Software Foundation.

 *******************************

   REVISION HISTORY
   Version 1.0: Henrik EKblad
   Version 1.1 - 2016-07-20: Converted to MySensors v2.0 and added various improvements - Torben Woltjen (mozzbozz)

   DESCRIPTION
   This sketch provides an example of how to implement a humidity/temperature
   sensor using a DHT11/DHT-22.

   For more information, please visit:
   http://www.mysensors.org/build/humidity

*/

// Enable debug prints
#define MY_DEBUG
#define MY_DEBUG_VERBOSE

// Enable and select radio type attached
#define MY_RADIO_NRF24
// #define MY_DEBUG_VERBOSE_RF24 
#define MY_RF24_PA_LEVEL RF24_PA_LOW

//#define MY_RADIO_RFM69
//#define MY_RS485

#include <SPI.h>
#include <MySensors.h>

#define CHILD_ID_TEMP 0

#define MY_NODE_ID 2
#define NODE_ID 2

MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);


void presentation()
{
  // Send the sketch version information to the gateway
  sendSketchInfo("TemperatureAndHumidity", "1.1");

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_TEMP, S_TEMP);

}


void setup()
{
}

int temp = 0;
void loop()
{
    /*
  send(msgTemp.set(temp++, 1));
  delay(1000);
  if (temp > 50)
    temp = 0;
*/
}
