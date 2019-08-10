#include <stdint.h>
#include <stddef.h>

#include <xdc/std.h>
#include <xdc/runtime/Error.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/display/Display.h>
#include <ti/drivers/ADC.h>
//#include <ti/drivers/UART.h>
/* Example/Board Header files */
#include "Board.h"
#include "mybutton.h"
#include "simple_peripheral.h"
//#define SBP_BTN_EVT     0x0020
#define THREADSTACKSIZE    1024
Task_Struct myTask; // @suppress("Type cannot be resolved")
uint8_t myTaskStack[THREADSTACKSIZE];

uint16_t numBtn;

extern Display_Handle dispHandle;


Semaphore_Handle btnSem;

/***************** gpioButtonFxn0 ********************/
void gpioButtonFxn0(uint_least8_t index)
{
    /* Clear the GPIO interrupt and toggle an LED */
    GPIO_toggle(Board_GPIO_LED0);
    numBtn++;
//    SimplePeripheral_enqueueMsg(SBP_BTN_EVT, 0, NULL);
    Semaphore_post(btnSem);
}
void *myThread(void *arg0) {

/*    char input;
    const char  echoPrompt[] = "Number button click:";
    UART_Handle uart;
    UART_Params uartParams;
*/
    /* We will fill this in later */
 Semaphore_Params semParams;
 Semaphore_Struct structSem;

 Semaphore_Params_init(&semParams);
 //////////////// use create function
 /*btnSem = Semaphore_create(0, &semParams, Error_IGNORE);
 if(btnSem == NULL)
 {
       /* Semaphore_create() failed*/
/*     Display_print0(dispHandle, 0, 0,"btnSem semaphore creation failed\n");

     while(1);

 }

*/
 Semaphore_construct(&structSem,0,&semParams);
 btnSem = Semaphore_handle(&structSem);

 numBtn = 0;

 GPIO_init();
 //UART_init();

 GPIO_setConfig(Board_GPIO_BUTTON0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

 /* install Button callback */
 GPIO_setCallback(Board_GPIO_BUTTON0, gpioButtonFxn0);

 /* Enable interrupts */
 GPIO_enableInt(Board_GPIO_BUTTON0);

 /* Create a UART with data processing off. */
 /*    UART_Params_init(&uartParams);
     uartParams.writeDataMode = UART_DATA_BINARY;
     uartParams.readDataMode = UART_DATA_BINARY;
     uartParams.readReturnMode = UART_RETURN_FULL;
     uartParams.readEcho = UART_ECHO_OFF;
     uartParams.baudRate = 115200;

     uart = UART_open(Board_UART0, &uartParams);

     if (uart == NULL) {
             // UART_open() failed
             while (1);
         }
 */
    // UART_write(uart, echoPrompt, sizeof(echoPrompt));
 while (1) {
     /* Pend on semaphore, tmp116Sem */
     Semaphore_pend(btnSem, BIOS_WAIT_FOREVER);
     //MyButton_SetParameter(MYBUTTON_NUMBUTTON_ID, MYBUTTON_NUMBUTTON_LEN, &numBtn);
     Display_printf(dispHandle, 6, 0, "Number button click: %d\n", numBtn);
     //UART_write(uart, echoPrompt, sizeof(echoPrompt));
     //UART_write(uart, &numBtn, sizeof(numBtn));
     //UART_write(uart, "\r\n", 2);

 }
}
/********** myThread_create **********/
void myThread_create(void) {
  Task_Params taskParams;

  /* Configure task */
  Task_Params_init(&taskParams);
  taskParams.stack = myTaskStack;
  taskParams.stackSize = THREADSTACKSIZE;
  taskParams.priority = 1;

  Task_construct(&myTask, myThread, &taskParams, NULL);//Error_IGNORE
}
