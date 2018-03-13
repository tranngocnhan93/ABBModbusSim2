/*
===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
 */

#include "chip.h"
#include "board.h"
#include <cr_section_macros.h>
#include <cstring>
#include <cstdio>
#include "ModbusMaster.h"
#include "itm_class.h"
#include "I2C.h"
#include "DigitalIoPin.h"
#include <string>
#include "LiquidCrystal.h"
#include <vector>
#include <algorithm>
#define	BUTTON_STEP 5
#define FILTER_LEN	11
static volatile int counter;
static volatile uint32_t systicks;

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
void SysTick_Handler(void)
{
	systicks++;
	if(counter > 0) counter--;
}
#ifdef __cplusplus
}
#endif

void Sleep(int ms)
{
	counter = ms;
	while(counter > 0) {
		__WFI();
	}
}

/* this function is required by the modbus library */
uint32_t millis() {
	return systicks;
}

void printRegister(ModbusMaster& node, uint16_t reg) {
	uint8_t result;
	// slave: read 16-bit registers starting at reg to RX buffer
	result = node.readHoldingRegisters(reg, 1);

	// do something with data if read is successful
	if (result == node.ku8MBSuccess)
	{
		printf("R%d=%04X\n", reg, node.getResponseBuffer(0));
	}
	else {
		printf("R%d=???\n", reg);
	}
}

bool setFrequency(ModbusMaster& node, uint16_t freq) {
	uint8_t result;
	int ctr;
	bool atSetpoint;
	const int delay = 500;

	node.writeSingleRegister(1, freq); // set motor frequency

	//	printf("Set freq = %d\n", freq/40); // for debugging

	// wait until we reach set point or timeout occurs
	ctr = 0;
	atSetpoint = false;
	do {
		Sleep(delay);
		// read status word
		result = node.readHoldingRegisters(3, 1);
		// check if we are at setpoint
		if (result == node.ku8MBSuccess) {
			if(node.getResponseBuffer(0) & 0x0100) atSetpoint = true;
		}
		ctr++;
	} while(ctr < 20 && !atSetpoint);

	//	printf("Elapsed: %d\n", ctr * delay); // for debugging

	return atSetpoint;
}

long i2cTest(I2C &i2c) {

	uint8_t pressureData[3];
	uint8_t readPressureCmd = 0xF1;
	int16_t pressure = 0;
	if (i2c.transaction(0x40, &readPressureCmd, 1, pressureData, 3)) {
		pressure = (pressureData[0] << 8) | pressureData[1];
		pressure = pressure/240*0.95;
	}
	else {
		pressure = -1;	//in this case negative pressure means error
	}
	Sleep(5);
	return pressure;
}

void setFanSpeed(ModbusMaster &node, uint8_t speed){
	uint16_t freq = speed*200;
	setFrequency(node, freq);
}

long kalman(long noisy) {
	float x_est_last = 0;
	float P_last = 0;
	//the noise in the system
	float Q = 9.0;//0.022;
	float R = 9.0;//0.617;

	float K;
	float P;
	float P_temp;
	float x_temp_est;
	float x_est;
	float z_measured; //the 'noisy' value we measured

	//initialize with a measurement
	x_est_last = 0;

	//do a prediction
	x_temp_est = x_est_last;
	P_temp = P_last + Q;
	//calculate the Kalman gain
	K = P_temp * (1.0/(P_temp + R));
	//measure
	z_measured = (float)noisy;
	//correct
	x_est = x_temp_est + K * (z_measured - x_temp_est); //kalman
	P = (1- K) * P_temp;

	//update our last's
	P_last = P;
	x_est_last = x_est;
	return (long)x_est;
}

uint8_t filter(uint8_t noisy) {
	static uint8_t arr[FILTER_LEN];
	static uint8_t i = 0;
	static bool initialised = false;
//	uint8_t mean = 0;
//	uint16_t sum = 0;
	std::vector<uint8_t> filter_vec(arr, arr+FILTER_LEN-1);
	std::vector<uint8_t>::iterator it;
	if(initialised == false) {
		arr[i] = noisy;
		i++;
		if(i == FILTER_LEN) {
			initialised = true;
			i = 0;
		}
	}
	else {
		arr[i] = noisy;
		i++;
		if(i == FILTER_LEN) {
			i = 0;
		}
//		for(int j = 0; j < FILTER_LEN; j++) {
//			sum += arr[j];
//		}
//		mean = sum/FILTER_LEN;
		std::sort(filter_vec.begin(), filter_vec.end());
		it = filter_vec.begin()+5;
	}

	return *it;
}

uint8_t pid(uint8_t desired_pressure, uint8_t actual_pressure, uint8_t delta_time) {
	signed char error;
	static signed char last_error;
	float pTerm, dTerm, speed, bias_speed;
	float kp = 0.425;
	float ki = 0.013;
	float kd = 50.0;
	static float iTerm = 0;

	bias_speed = ((float)desired_pressure)/127*100;
	error = desired_pressure - actual_pressure;
	pTerm = kp*(float)error;
	iTerm += ki*error*delta_time;
	dTerm = kd*(error - last_error)/delta_time;
	speed = pTerm + iTerm + dTerm + bias_speed;

//	itm.print(std::to_string(k));
//	itm.print("  ");
//	itm.print(std::to_string(delta_time));
//	itm.print("\n");

	if(speed > 100.0)
		speed = 100.0;
	else if(speed < 0.0)
		speed = 0.0;
	last_error = error;
	return speed;
}

/**
 * @brief	Main UART program body
 * @return	Always returns 1
 */
int main(void)
{
	SystemCoreClockUpdate();
	Chip_SWM_MovablePortPinAssign(SWM_SWO_O, 1, 2);// Set up SWO to PIO1_2  Needed for SWO printf
	SysTick_Config(SystemCoreClock / 1000);/* Enable and setup SysTick Timer at a periodic rate */
	Chip_RIT_Init(LPC_RITIMER);
	Board_Init();

	ModbusMaster node(2); // Create modbus object that connects to slave id 2
	node.begin(9600); // set transmission rate - other parameters are set inside the object and can't be changed here
	node.writeSingleRegister(0, 0x0406); // prepare for starting
	Sleep(1000); // give converter some time to set up
	node.writeSingleRegister(0, 0x047F); // set drive to start mode

	I2C i2c(0, 100000);
	SWOITMclass itm;
	DigitalIoPin button1(0, 16, true, true, true);
	DigitalIoPin button2(0, 0, true, true, true);
	DigitalIoPin button3(1, 3, true, true, true);
	DigitalIoPin button4(0, 10, true, true, true);
	DigitalIoPin RS(0, 8, false, false, false);
	DigitalIoPin EN(1, 6, false, false, false);
	DigitalIoPin D4(1, 8, false, false, false);
	DigitalIoPin D5(0, 5, false, false, false);
	DigitalIoPin D6(0, 6, false, false, false);
	DigitalIoPin D7(0, 7, false, false, false);
	LiquidCrystal lcd(&RS, &EN, &D4, &D5, &D6, &D7);

	uint8_t speed = 0, desired_pressure = 0, actual_pressure = 0, delta_time = 0, filtered_press = 0;
	uint32_t time1 = 0, time2 = 0;
	std::string str;
	bool mode = true;		//false: manual true: automatic
//	float k = 0.013;
	while(1) {
		while(!mode) {
			if(button1.Read()) {
				if(speed <= 100 - BUTTON_STEP)
					speed += BUTTON_STEP;
			}
			if(button3.Read()) {
				if(speed >= BUTTON_STEP)
					speed -= BUTTON_STEP;
			}
			if(button2.Read()) {
				mode = true;
			}
			setFanSpeed(node, speed);

			/*	Print LCD	*/
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print("Fan Speed: ");
			lcd.print(std::to_string(speed));
			lcd.setCursor(0,  1);
			lcd.print("Pressure:  ");
			actual_pressure = i2cTest(i2c);
			if(actual_pressure >= 0)
				lcd.print(std::to_string(actual_pressure));
			else lcd.print("Error");
			Sleep(300);
		}
		while(mode) {
			time1 = millis();
//			if(button4.Read()) {
//				k += 0.001;
//			}
//			if(button2.Read()) {
//				k -= 0.001;
//			}
			if(button1.Read()) {
				if(desired_pressure <= 120 - BUTTON_STEP)
					desired_pressure += BUTTON_STEP;
			}
			if(button3.Read()) {
				if(desired_pressure >= BUTTON_STEP)
					desired_pressure -= BUTTON_STEP;
			}
			actual_pressure = i2cTest(i2c);
			if(actual_pressure >= 0) {
				time2 = millis();
				if(time2 < time1) 			//the systick has been reset
					delta_time = 4294967295 - time1 + time2;
				else delta_time = time2 - time1;
//				itm.print(delta_time);
				filtered_press = filter(actual_pressure);
				speed = pid(desired_pressure, filtered_press, 10);
				setFanSpeed(node, speed);

				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print("Desire: ");
				lcd.print(std::to_string(desired_pressure));
				lcd.setCursor(0, 1);
				lcd.print("Actual: ");
				str = std::to_string(filtered_press);
				lcd.print(str);
			}
			else lcd.print("Error");
			Board_UARTPutSTR(str.c_str());
			Board_UARTPutChar(',');
			str = std::to_string(desired_pressure);
			Board_UARTPutSTR(str.c_str());
			Board_UARTPutChar('\n');
//			Sleep(200);
		}
	}

	return 0;
}





























/*
 * if(button2.Read()) {
				mode = false;
				Sleep(200);
			}

void abbModbusTest() {
	ModbusMaster node(2); // Create modbus object that connects to slave id 2

	node.begin(9600); // set transmission rate - other parameters are set inside the object and can't be changed here

	printRegister(node, 3); // for debugging

	node.writeSingleRegister(0, 0x0406); // prepare for starting

	printRegister(node, 3); // for debugging

	Sleep(1000); // give converter some time to set up
	// note: we should have a startup state machine that check converter status and acts per current status
	//       but we take the easy way out and just wait a while and hope that everything goes well

	printRegister(node, 3); // for debugging

	node.writeSingleRegister(0, 0x047F); // set drive to start mode

	printRegister(node, 3); // for debugging

	Sleep(1000); // give converter some time to set up
	// note: we should have a startup state machine that check converter status and acts per current status
	//       but we take the easy way out and just wait a while and hope that everything goes well

	printRegister(node, 3); // for debugging

	int i = 0;
	int j = 0;
	const uint16_t fa[20] = { 1000, 2000, 3000, 3500, 4000, 5000, 7000, 8000, 8300, 10000, 10000, 9000, 8000, 7000, 6000, 5000, 4000, 3000, 2000, 1000 };

	while (1) {
		uint8_t result;

		// slave: read (2) 16-bit registers starting at register 102 to RX buffer
		j = 0;
		do {
			result = node.readHoldingRegisters(102, 2);
			j++;
		} while(j < 3 && result != node.ku8MBSuccess);
		// note: sometimes we don't succeed on first read so we try up to three times
		// if read is successful print frequency and current (scaled values)
		if (result == node.ku8MBSuccess) {
			printf("F=%4d, I=%4d  (ctr=%d)\n", node.getResponseBuffer(0), node.getResponseBuffer(1),j);
		}
		else {
			printf("ctr=%d\n",j);
		}

		Sleep(3000);
		i++;
		if(i >= 20) {
			i=0;
		}
		// frequency is scaled:
		// 20000 = 50 Hz, 0 = 0 Hz, linear scale 400 units/Hz
		setFrequency(node, fa[i]);
	}
}

void displayConfig(ModbusMaster &node) {
	// slave: read (2) 16-bit registers starting at register 102 to RX buffer
	int j = 0;
	uint8_t result;
	do {
		result = node.readHoldingRegisters(102, 2);
		j++;
	} while(j < 3 && result != node.ku8MBSuccess);
	// note: sometimes we don't succeed on first read so we try up to three times
	// if read is successful print frequency and current (scaled values)
	if (result == node.ku8MBSuccess) {
		printf("F=%4d, I=%4d  (ctr=%d)\n", node.getResponseBuffer(0), node.getResponseBuffer(1),j);
	}
	else {
		printf("ctr=%d\n",j);
	}
	Sleep(100);
}




 */
