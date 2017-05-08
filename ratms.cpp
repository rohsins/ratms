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

uint8_t whoAmI;
uint8_t axisXL;
int8_t axisXH;
int16_t axisX;
uint8_t axisYL;
int8_t axisYH;
int16_t axisY;
uint8_t axisZL;
int8_t axisZH;
int16_t axisZ;

float xGravity;
float yGravity;
float zGravity;

enum movement {
	none = 0,
	left,
	backward,
	right,
	forward
};

movement direction;

uint8_t accRead(uint8_t regRead) {
	uint8_t temp = 0;
	USART_Tx(usartAcc, (regRead | 0x80));
	USART_Rx(usartAcc);
	USART_Tx(usartAcc, 0x00);
	USART_Tx(usartAcc, (regRead | 0x80));
	USART_Rx(usartAcc);
	USART_Tx(usartAcc, 0x00);
	USART_Tx(usartAcc, (regRead | 0x80));
	USART_Rx(usartAcc);
	USART_Tx(usartAcc, 0x00);
	USART_Tx(usartAcc, (regRead | 0x80));
	temp = USART_Rx(usartAcc);
	USART_Tx(usartAcc, 0x00);
	osDelay(1);
	return temp;
}

void accWrite(uint8_t regWrite, uint8_t regValue) {
	USART_Tx(usartAcc, regWrite);
	USART_Tx(usartAcc, regValue);
	osDelay(1);
}

void readAxisData() {
	axisXL = accRead(0x28);
	axisXH = accRead(0x29);
	axisYL = accRead(0x2A);
	axisYH = accRead(0x2B);
	axisZL = accRead(0x2C);
	axisZH = accRead(0x2D);
	
	axisX = axisXH << 8 | axisXL;
	axisY = axisYH << 8 | axisYL;
	axisZ = axisZH << 8 | axisZL;
}

void accConfig() {
	accWrite(0x20, 0x7F); //enable bdu, zen, data rate 400Hz
	accWrite(0x24, 0x80); //anti-aliasing filter 400Hz
	//accWrite(0x23, 0x88); //enable dataready interrupt
	accWrite(0x23, 0x00);
	accWrite(0x25, 0x00);
}

void GPIO_ODD_IRQHandler(void) {
	GPIO_IntClear(GPIO_IntGet());
}

void GPIO_EVEN_IRQHandler(void) {
  GPIO_IntClear(GPIO_IntGet());
}

void spiInitialize(void) {
	USART_InitSync_TypeDef usartInitTypeDefAcc = USART_INITSYNC_DEFAULT;
	
	usartInitTypeDefAcc.clockMode = usartClockMode3;
	usartInitTypeDefAcc.msbf = true;
	
	usartInitTypeDefAcc.baudrate = 800000;
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
//	GPIO_PinModeSet(gpioPortD, 6, gpioModeInput, 0);
//	GPIO_PinModeSet(gpioPortD, 7, gpioModeInput, 0);
//	
//	GPIO_IntConfig(gpioPortD, 6, false, true, true);
//	GPIO_IntConfig(gpioPortD, 7, false, true, true);
//	
//	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
//	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
//	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
//	NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

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
osThreadDef(blinky, osPriorityBelowNormal, 1, 0);

void runner(void const * params) {	
	while (1) {
		whoAmI = accRead(0x0F);
		if (whoAmI == 0x3F) readAxisData();
		osDelay(100);
	}
}
osThreadDef(runner, osPriorityNormal, 1, 0);

void computeEngine(void const * params) {
	while (1) {
		if (axisX > 4000) direction = forward;
		else if (axisX < -6000) direction = backward;
		else if (axisY > 6000) direction = left;
		else if (axisY < -6000) direction = right;
		else direction = none;
		
		xGravity = (axisX * 9.8)/16384;
		yGravity = (axisY * 9.8)/16384;
		zGravity = (axisZ * 9.8)/16384;
		
		osDelay(200);
	}
}
osThreadDef(computeEngine, osPriorityNormal, 1, 0);

 int main (void) {
	 
	SysTick_Config(SystemCoreClock/1000); // 1 milisecond SysTick
	
	osKernelInitialize();
	osKernelStart();
	Initialize();
	accConfig();
		
	osThreadCreate(osThread(blinky), NULL);
//	osThreadCreate(osThread(outtog), NULL);
	osThreadCreate(osThread(runner), NULL);
	osThreadCreate(osThread(computeEngine), NULL);
	
	return 0;
}
