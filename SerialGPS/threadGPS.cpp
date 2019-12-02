#include "mbed.h"
#include "rtos.h"
#include "SerialGPS.h"
SerialGPS gpsline(PA_9, PA_10, 9600);
#include  "SerialGPS.h"
DigitalOut led2(LED2);
Thread threadGPS;
void GPS_thread(); 
void GPS_thread() {
    while (true) {
        led2 = !led2;
				gpsline.sample();
				Thread::wait(1000);
    }
}
