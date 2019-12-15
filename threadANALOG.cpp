#include "mbed.h"
AnalogIn soilmois(PA_0); 
AnalogIn light(PA_4);

extern int time_interval;
extern int mode;
extern float valueSM;
extern float valueLight;

Thread thread_analog(osPriorityNormal, 512); // 1K stack size

void read_analog(void) {
  while (true) {

    valueSM = soilmois*100;
	
    valueLight = light*100;
	
			Thread::wait(1000);
	}
}
