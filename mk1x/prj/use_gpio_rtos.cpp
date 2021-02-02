#include "device.h"
#include "pin.h"
//rtos

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

void vATaskFunction( void *pvParameters )
{
    Pin ledGreen(port::B, 23);
    while(1){
        ledGreen.set();
        vTaskDelay(5000);
        ledGreen.clear();
        vTaskDelay(1000);
    }
}
int main (){
    SysTick_Config(SystemCoreClock/1000);
    xTaskCreate( vATaskFunction, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL );
  //start scheduler
    vTaskStartScheduler();
    while(1){
    }
}