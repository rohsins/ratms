/** 
		Hardware
*/

#include "cmsis_os.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"

USART_TypeDef *usartAcc = USART2;

uint8_t accRead(uint8_t regRead) {
	uint8_t tem = 0;
	USART_Tx(usartAcc, (regRead | 0x80));
	USART_Tx(usartAcc, 0x00);
	tem = USART_Rx(usartAcc);
	return tem;
}

void accWrite(uint8_t regWrite, uint8_t regValue) {
	uint8_t tem = 0;
	USART_Tx(usartAcc, (regWrite | 0x00));
	USART_Tx(usartAcc, regValue);
	tem = USART_Rx(usartAcc);
}

void spiInitialize(void) {
	USART_InitSync_TypeDef usartInitTypeDefAcc = USART_INITSYNC_DEFAULT;
	
	usartInitTypeDefAcc.clockMode = usartClockMode0;
	usartInitTypeDefAcc.msbf = true;
	
	usartInitTypeDefAcc.baudrate = 100000;
	usartInitTypeDefAcc.databits = usartDatabits8;
	
	USART_Enable_TypeDef usartEnableTypeDef = usartEnable;
		
	USART_InitSync(usartAcc, &usartInitTypeDefAcc);
	
	/* usartAcc */
	GPIO_PinModeSet(gpioPortB, 3, gpioModePushPull, 1); /*MOSI*/
  GPIO_PinModeSet(gpioPortB, 4, gpioModeInput, 0); /*MISO*/
	GPIO_PinModeSet(gpioPortB, 5, gpioModePushPull, 0); /*CLOCK*/
  GPIO_PinModeSet(gpioPortB, 6, gpioModePushPull, 1); /*CS*/
	
	/* usartAcc */
	usartAcc->ROUTE |= 1 << 0 | 1 << 1 | 1 << 2 | 1 << 3 | 1 << 8 | 0 << 9;
	usartAcc->CTRL |= USART_CTRL_AUTOCS;
	
	USART_Enable(usartAcc, usartEnableTypeDef);
}

void Initialize(void) {
	 CHIP_Init();
	 
	 CMU_OscillatorEnable (cmuOsc_HFRCO, true, true);
	 CMU_ClockSelectSet   (cmuClock_HF,    cmuSelect_HFRCO);
	 CMU_ClockDivSet      (cmuClock_HFPER, cmuClkDiv_2);
	 
	 CMU_ClockEnable(cmuClock_HFPER, true);
	 CMU_ClockEnable(cmuClock_GPIO, true);
	
	 GPIO_PinModeSet(gpioPortA, 8, gpioModePushPull, 0); 
	 GPIO_PinModeSet(gpioPortA, 9, gpioModePushPull, 0); 
	 GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 0); 
	
	 spiInitialize();
}

void blinky(void const* arg) {
	 while (1) {
		 GPIO_PinOutSet(gpioPortC, 0);
		 osDelay(1000);
		 GPIO_PinOutClear(gpioPortC, 0);
		 osDelay(100);
		 GPIO_PinOutSet(gpioPortC, 0);
		 osDelay(100);
		 GPIO_PinOutClear(gpioPortC, 0);
		 osDelay(100);
	 }
}
osThreadDef(blinky, osPriorityNormal, 1, 0);

void outtog(void const* arg) {
	 while (1) {
		 GPIO_PinOutSet(gpioPortA, 8);
		 GPIO_PinOutClear(gpioPortA, 9);
		 osDelay(3000);
		 GPIO_PinOutClear(gpioPortA, 8);
		 GPIO_PinOutSet(gpioPortA, 9);
		 osDelay(3000);
	 }
}
osThreadDef(outtog, osPriorityNormal, 1, 0);

void runner(void const * params) {
	uint8_t hewio;
	accWrite(0x20, 0x17);
	while (1) {
		hewio = accRead(0x2C);
		osDelay(1000);
	}
}
osThreadDef(runner, osPriorityNormal, 1, 0);
 
int main (void) {
  SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000); // 1 milisecond SysTick
	
	osKernelInitialize();
	Initialize();
	
	osThreadCreate(osThread(blinky), NULL);
	osThreadCreate(osThread(outtog), NULL);
	osThreadCreate(osThread(runner), NULL);
	
	return 0;
}
