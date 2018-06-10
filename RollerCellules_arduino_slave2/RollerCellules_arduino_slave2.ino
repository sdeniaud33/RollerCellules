/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0: Henrik EKblad
 * Version 1.1 - 2016-07-20: Converted to MySensors v2.0 and added various improvements - Torben Woltjen (mozzbozz)
 * 
 * DESCRIPTION
 * This sketch provides an example of how to implement a humidity/temperature
 * sensor using a DHT11/DHT-22.
 *  
 * For more information, please visit:
 * http://www.mysensors.org/build/humidity
 * 
 */

// Enable debug prints
#define MY_DEBUG
#define MY_RF24_PA_LEVEL RF24_PA_HIGH

// Enable and select radio type attached 
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69
//#define MY_RS485

#define SLAVEID 1
// *******************************************************
// **
// ** RADIO MESSAGES
// **
// *******************************************************
static const byte RADIO_MSG_MASTER_HEARTBEAT = 1;
static const byte RADIO_MSG_SLAVE_HEARTBEAT[2] = {2, 3};
static const byte RADIO_MSG_SLAVE_CELL_FREE[2] = {4, 5};
static const byte RADIO_MSG_SLAVE_CELL_OBSTACLE[2] = {6, 7};
static const byte RADIO_MSG_MASTER_MODE_DIAGNOSE = 10;
static const byte RADIO_MSG_MASTER_MODE_FREESTART = 11;
static const byte RADIO_MSG_MASTER_MODE_KO_SYSTEM_SINGLE = 12;
static const byte RADIO_MSG_MASTER_MODE_KO_SYSTEM_DUEL = 13;

#include <SPI.h>
#include <MySensors.h>  

MyMessage msgHeartbeat(RADIO_MSG_SLAVE_HEARTBEAT[SLAVEID], V_STATUS);


void presentation()  
{ 
  // Send the sketch version information to the gateway
  sendSketchInfo("Cellule slave #1", "1.1");

  // Register all sensors to gw (they will be created as child devices)
  present(RADIO_MSG_SLAVE_HEARTBEAT[SLAVEID], S_BINARY);
}


void setup()
{
}

unsigned int val = 0;
void loop()      
{  
    Serial.println(val);
    send(msgHeartbeat.set(val++));
    delay(500);
}
