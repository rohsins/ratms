/** 
		Hardware
*/

#include "cmsis_os.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "math.h"

USART_TypeDef *usartAcc = USART0;
int16_t axisZ;
int whoami;
float deg;
float zGravity;
const float toDeg = (180/3.1415);

void blinky(void const* arg) {
	 while (1) {
		 GPIO_PinOutSet(gpioPortC, 0);
		 osDelay(3000);
		 GPIO_PinOutClear(gpioPortC, 0);
		 osDelay(10);
		 GPIO_PinOutSet(gpioPortC, 0);
		 osDelay(130);
		 GPIO_PinOutClear(gpioPortC, 0);
		 osDelay(20);
	 }
}
osThreadDef(blinky, osPriorityNormal, 1, 0);

uint8_t accRead(uint8_t regRead) {
	uint8_t temp = 0;
	USART_Tx(usartAcc, (regRead | 0x80));
	USART_Tx(usartAcc, 0x00);
	temp = USART_Rx(usartAcc);
	osDelay(1);
	return temp;
}

uint8_t accWrite(uint8_t regWrite, uint8_t regValue) {
	uint8_t temp = 0;
	USART_Tx(usartAcc, regWrite);
	USART_Tx(usartAcc, regValue);
	temp = USART_Rx(usartAcc);
	osDelay(1);
	return temp;
}

void GPIO_ODD_IRQHandler(void) {
	GPIO_IntClear(GPIO_IntGet());
}

void GPIO_EVEN_IRQHandler(void) {
  GPIO_IntClear(GPIO_IntGet());
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
	GPIO_PinModeSet(gpioPortC, 11, gpioModePushPull, 1); /*MOSI*/
  GPIO_PinModeSet(gpioPortC, 10, gpioModeInput, 0); /*MISO*/
	GPIO_PinModeSet(gpioPortC, 9, gpioModePushPull, 0); /*CLOCK*/
  GPIO_PinModeSet(gpioPortC, 8, gpioModePushPull, 1); /*CS*/
	
	/* usartAcc */
	usartAcc->ROUTE |= 1 << 0 | 1 << 1 | 1 << 2 | 1 << 3 | 0 << 8 | 1 << 9; //location 02 bitposition 9 and 8.
	usartAcc->CTRL |= USART_CTRL_AUTOCS;
	
	USART_Enable(usartAcc, usartEnableTypeDef);
}

void Initialize(void) {
	 CHIP_Init();
	 
	 CMU_OscillatorEnable (cmuOsc_HFRCO, true, true);
	 CMU_ClockSelectSet   (cmuClock_HF,    cmuSelect_HFRCO);
	 CMU_ClockDivSet      (cmuClock_HFPER, cmuClkDiv_1);
	 
	 CMU_ClockEnable(cmuClock_HFPER, true);
	 CMU_ClockEnable(cmuClock_GPIO, true);
	 CMU_ClockEnable(cmuClock_USART0, true);
	
	 GPIO_PinModeSet(gpioPortA, 8, gpioModePushPull, 0); 
	 GPIO_PinModeSet(gpioPortA, 9, gpioModePushPull, 0); 
	 GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 0); 
	
	 spiInitialize();
	
	//gpio interrupts
	GPIO_PinModeSet(gpioPortD, 6, gpioModeInput, 0);
	GPIO_PinModeSet(gpioPortD, 7, gpioModeInput, 0);
	
	GPIO_IntConfig(gpioPortD, 6, false, true, true);
	GPIO_IntConfig(gpioPortD, 7, false, true, true);
	
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

//void outtog(void const* arg) {
//	 while (1) {
//		 GPIO_PinOutSet(gpioPortA, 8);
//		 GPIO_PinOutClear(gpioPortA, 9);
//		 osDelay(3000);
//		 GPIO_PinOutClear(gpioPortA, 8);
//		 GPIO_PinOutSet(gpioPortA, 9);
//		 osDelay(3000);
//	 }
//}
//osThreadDef(outtog, osPriorityNormal, 1, 0);

void runner(void const * params) {	
	accWrite(0x20, 0x7C); //enable bdu, zen, data rate 400Hz
	accWrite(0x24, 0x80); //anti-aliasing filter 400Hz
	accWrite(0x23, 0x88); //enable dataready interrupt
	
	int8_t axisZH;
	uint8_t axisZL;
	
	while (1) {
		whoami = accRead(0x0F);
		axisZH = accRead(0x2D);
		axisZL = accRead(0x2C);
		axisZ = axisZH  << 8 | axisZL;
		zGravity = ((axisZ*2.0)/32768.0);
		osDelay(100);
	}
}
osThreadDef(runner, osPriorityNormal, 1, 0);

 int main (void) {
  SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000); // 1 milisecond SysTick
	
	osKernelInitialize();
	Initialize();
		
	osThreadCreate(osThread(blinky), NULL);
//	osThreadCreate(osThread(outtog), NULL);
	osThreadCreate(osThread(runner), NULL);
	
	return 0;
}
