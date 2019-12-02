/**
 * Copyright (c) 2017, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>
#include "MBED_Adafruit_GPS.h"
#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"

// Application helpers
#include "DummySensor.h"
#include "trace_helper.h"
#include "lora_radio_helper.h"
#include "mbed.h"

extern float valueSM;
extern float valueLight;
extern float temp,hum;
extern Adafruit_GPS myGPS;

using namespace events;


// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
uint8_t tx_buffer[30];
uint8_t rx_buffer[30];

Serial pc(USBTX, USBRX, 9600); //9600 baudios - used to print some values
InterruptIn pushbutton(USER_BUTTON);
DigitalOut green1(LED1); //LED of B-L072Z-LRWAN1 board
DigitalOut green2(LED2);
DigitalOut green3(LED3);

long int numberOfMeasures = 5;
long int time_interval = 2;
int mode = 0;

extern Thread thread_i2c;
extern Thread thread_serial;
extern Thread thread_analog;
extern Thread thread_alarms;

extern void read_analog(void);
extern void read_i2c(void);
extern void read_serial(void);
extern void trigger_alarm(void);

extern void printAll();
extern int saveData();
extern void printMean_Max_Min();
extern void advancedMode();
/*
 * Sets up an application dependent transmission timer in ms. Used only when Duty Cycling is off for testing
 */
#define TX_TIMER                        10000

/**
 * Maximum number of events for the event queue.
 * 10 is the safe number for the stack events, however, if application
 * also uses the queue for whatever purposes, this number should be increased.
 */
#define MAX_NUMBER_OF_EVENTS            10

/**
 * Maximum number of retries for CONFIRMED messages before giving up
 */
#define CONFIRMED_MSG_RETRY_COUNTER     3

/**
 * Dummy pin for dummy sensor
 */
#define PC_9                            0

/**
 * Dummy sensor class object
 */
DS1820  ds1820(PC_9);

/**
* This event queue is the global event queue for both the
* application and stack. To conserve memory, the stack is designed to run
* in the same thread as the application and the application is responsible for
* providing an event queue to the stack that will be used for ISR deferment as
* well as application information event queuing.
*/
static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS * EVENTS_EVENT_SIZE);

/**
 * Event handler.
 *
 * This will be passed to the LoRaWAN stack to queue events for the
 * application which in turn drive the application.
 */
static void lora_event_handler(lorawan_event_t event);

/**
 * Constructing Mbed LoRaWANInterface and passing it down the radio object.
 */
static LoRaWANInterface lorawan(radio);

/**
 * Application specific callbacks
 */
static lorawan_app_callbacks_t callbacks;


void changeMode(){
	mode += 1;
	mode %= 3;
}

/**
 * Entry point for application
 */
int main (void)
{
    // setup tracing
    setup_trace();

    // stores the status of a call to LoRaWAN protocol
    lorawan_status_t retcode;

    // Initialize LoRaWAN stack
    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
         pc.printf("\r\n LoRa initialization failed! \r\n");
        return -1;
    }

     pc.printf("\r\n Mbed LoRaWANStack initialized \r\n");

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Set number of retries in case of CONFIRMED messages
    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER)
                                          != LORAWAN_STATUS_OK) {
         pc.printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }

     pc.printf("\r\n CONFIRMED message retries : %d \r\n",
           CONFIRMED_MSG_RETRY_COUNTER);

    // Enable adaptive data rate
    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
         pc.printf("\r\n enable_adaptive_datarate failed! \r\n");
        return -1;
    }

     pc.printf("\r\n Adaptive data  rate (ADR) - Enabled \r\n");
		

    retcode = lorawan.connect();

    if (retcode == LORAWAN_STATUS_OK ||
        retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
         pc.printf("\r\n Connection error, code = %d \r\n", retcode);
        return -1;
    }

     pc.printf("\r\n Connection - In Progress ...\r\n");

    // make your event queue dispatching events forever
  pushbutton.rise(changeMode);
  thread_analog.start(read_analog);
  thread_i2c.start(read_i2c);
  thread_serial.start(read_serial);
    
    ev_queue.dispatch_forever();


		
	while (true) {
		
		switch (mode){
			case 0:
				if(thread_alarms.get_state()== Thread::Running)
					thread_alarms.join();
				green1 = 1;
				green2 = 0;
				green3 = 0;
				wait(2);
				pc.printf("Mode %d (0 = test, 1 = normal, 2 = advanced)\r\n", mode);

				printAll();
				break;
			case 1:
				if(thread_alarms.get_state()!= Thread::Running)
					thread_alarms.start(trigger_alarm);
				green1 = 0;
				green2 = 1;
				green3 = 0;
				wait(time_interval);
				pc.printf("Mode %d (0 = test, 1 = normal, 2 = advanced)\r\n", mode);
				printAll();
				if (saveData()==numberOfMeasures)
					printMean_Max_Min();
				break;
			case 2:
				green1 = 0;
				green2 = 0;
				green3 = 1;

				advancedMode();
				wait(1);
				break;
			
			default:
				wait(2);
				break;
		}
	}
    return 0;
}


/**
 * Sends a message to the Network Server
 */
static void send_message()
{
    uint16_t packet_len;
    int16_t retcode;

    /*if (ds1820.begin()) {
        ds1820.startConversion();
        sensor_value = ds1820.read();
        ds1820.startConversion();
    } else {
         pc.printf("\r\n No sensor found \r\n");
        return;
    }*/

    //packet_len = sprintf((char*) tx_buffer, "%03.1f%03.1f%03.1f", temp, hum, valueSM);
    memcpy((void*)tx_buffer, (void*)&temp, sizeof(float));
    memcpy((void*)(tx_buffer + 4), (void*)&hum, sizeof(float));
    memcpy((void*)(tx_buffer + 8), (void*)&valueSM, sizeof(float));
    float latitude = 40.423245; //myGPS.latitude/100;
    float longitude = -3.321572; //myGPS.longitude/100;
  
    memcpy((void*)(tx_buffer + 12), (void*)&latitude, sizeof(float));
    memcpy((void*)(tx_buffer + 16), (void*)&longitude, sizeof(float));
    for(int j = 0; j<20; j++)
     pc.printf("%x ", tx_buffer[j]);
     pc.printf("to send\n");
    packet_len = 20;
    retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, packet_len,
                           MSG_CONFIRMED_FLAG);

    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ?  pc.printf("send - WOULD BLOCK\r\n")
                :  pc.printf("\r\n send() - Error code %d \r\n", retcode);
        return;
    }

     pc.printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
    memset(tx_buffer, 0, sizeof(tx_buffer));
}

/**
 * Receive a message from the Network Server
 */
static void receive_message()
{
    int16_t retcode;
    retcode = lorawan.receive(MBED_CONF_LORA_APP_PORT, rx_buffer,
                              sizeof(rx_buffer),
                              MSG_CONFIRMED_FLAG|MSG_UNCONFIRMED_FLAG);

    if (retcode < 0) {
         pc.printf("\r\n receive() - Error code %d \r\n", retcode);
        return;
    }

     pc.printf(" Data:");

    for (uint8_t i = 0; i < retcode; i++) {
         pc.printf("%x", rx_buffer[i]);
    }

     pc.printf("\r\n Data Length: %d\r\n", retcode);

    memset(rx_buffer, 0, sizeof(rx_buffer));
}

/**
 * Event handler
 */
static void lora_event_handler(lorawan_event_t event)
{
    switch (event) {
        case CONNECTED:
             pc.printf("\r\n Connection - Successful \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            } else {
                ev_queue.call_every(TX_TIMER, send_message);
            }

            break;
        case DISCONNECTED:
            ev_queue.break_dispatch();
             pc.printf("\r\n Disconnected Successfully \r\n");
            break;
        case TX_DONE:
             pc.printf("\r\n Message Sent to Network Server \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
             pc.printf("\r\n Transmission Error - EventCode = %d \r\n", event);
            // try again
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case RX_DONE:
             pc.printf("\r\n Received message from Network Server \r\n");
            receive_message();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
             pc.printf("\r\n Error in reception - Code = %d \r\n", event);
            break;
        case JOIN_FAILURE:
             pc.printf("\r\n OTAA Failed - Check Keys \r\n");
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}

// EOF
