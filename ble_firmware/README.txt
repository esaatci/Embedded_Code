this is the readme for ble firmware for camera revision F


There are two folders in this directory:
	- nrf_ble_peripheral
	- stm32_ble_controller


stm32_ble_controller
	This is the STM32 CUBEIDE project folder that implements syncing current time by using current time service from NRF BLE and sending heartbeat to the the nrf BLE module.

	The two chips are connected via I2C and an interrupt pin. One thing to note I was using external Pullup resistors for the I2C line. Make sure schematic has pullup resistors. Else enable pullup resistors for the lines on STM32.

	In this communication scheme STM32 which is the camera MCU is the Master and the nrf BLE chip is the Slave. STM32 has a async state machine that controls the data flow between the two chips.

	If you look at loop() inside the main function, you can see there are three essential functions.
		
			- async_ble_conn_handler()
			This is the function that implements the state machine for I2C communication. Taking a step back, The way that this logic works is simple. When the NRF BLE chip connects and bonds to the phone, it'll discover the Current Time Service. It'll then raise the INT pin which will cause an interrupt in STM32. I used the default interrupt handlers for GPIO external event. Once the interrupt is serviced, it call a Callback function called HAL_GPIO_EXTI_Callback that will set the comm flag. This callback is defined in main.c.
			The states of the state machine.
			CUR_TIME_POL_TX,
				Asks the BLE CHIP whether the current time is ready. This state is here for John's version which lacks an interrupt pin. 
			CUR_TIME_POL_RX,
				If the current time is not ready, The BLE chip will send a NOT_OK response and the state machine will come back to this state for 
				checking it again.
			REC_CUR_TIME_TX,
				The Master Sends, "send me the Current time command".
			REC_CUR_TIME_RX
							The master will recieve the current time from the NRF BLE chip in the format "day/month/year hours:minutes:seconds:fractions"
			HB_REQ_TX,
				The Master Sends, "I will send you the HeartBeat" 
			HB_REQ_RX,
				NRF will respond
			SEND_HB_TX,
				Master will transmit the Heartbeat that is prepared in the system_init function.
			POW_OFF_TX,
				Master will send power_off command which will put the NRF chip into sleep.
			POW_OFF_RX,
				NRF will respond
			MY_RESTART,
				This is the state that will hopefully resolve the errors if there are any. When this command is recieved by the NRF chip, it'll cause it to restart. It'll also reset the Master state machine. I'll talk more about what happens for error handling.
			DONE,
				This state is the last stage of the comm. When this stage is reached, the flag for camera operation will be set and the camera operation will start. 

			The state transitions are handled by the I2C tx and rx complete callbacks. If you look at the RX callback, you can see I have a special case for John's new camera which you can think of an arrow that goes back into CUR_TIME_POL_TX state if Master Recieved NOT_OK(which stands for current time not ready).

			my_cts_handler()
				This function is responsible for parsing and checking the current time recieved from the NRF BLE CHIP. It'll check the len of the string recieved from the NRF, parse the string into its constiutent parts and set the RTC with the time recieved.

			ble_i2c_error_handler()
				This function is responsible for resolving errors in the i2c communcation. It'll keep track of the number of errors. Here MAX_ERR_COUNT is rather an arbitrary number that I set. It can be adjusted. The function will basically reset the physical I2C hardware on the STM32 and force the state machine to renter the MY_RESTART state. 

			camera_operation()
				I have implemented the ble functionality and I2C using the devboards and the exact pin configuration that is on the rev f schematic. Double check the pins just in case though. This function is here so that It'll give an idea where the camera functionality will reside. Since the Thermal and RGB files are named as timestamps, it didn't make sense to me to start the camera operation before the time is synced.

nrf_ble_peripheral
	
	This is the project folder that implements the NRF side of the communication. NRF doesn't have "I2C" per se, but it has TWI. Which is the same thing but with a different name. I placed this folder under examples/ble_peripheral in NRF SDK 12.3.0 to make it easier to resolve the dependencies. We could have used a later SDK version but I used this so that the code is identical with the necklace. The code is based on the cts example and the twi_master_with_twis_slaves example projects.
		on_cts_c_evt()
			This is the event handler for the current time service.
			
			BLE_CTS_C_EVT_DISCOVERY_COMPLETE
				This is the state where the connected_to_cts flag will be set
			When the flag is set, we will read the current time from the Phone.
			
			BLE_CTS_C_EVT_CURRENT_TIME:
				This is state will be entered when the current time is recieved from the phone. It'll call a function called current_time_print that'll parse the current time and then pull the pin low to signal the interrupt.

			twis_event_handler()
				This is the function acts as an state machine for the i2c comm. Its similar to the STM32 state machine and the example twi code from the examples.

			bsp_event_handler()
				This is there to simulate BLE connection with the buttons on the nrf dev board.

What is Done?
	The dataflow STM -> NRF -> PHONE is working (HBT)
	The dataflow PHONE -> NRF -> STM is working (CTS)
	I2C state machines

What is left:
	- Integrating the ble and i2c code with the actual camera firmware. 
	- Watchdog Timer?
		There is no timeout in the HAL I2C functions of the STM32. If there is something wrong with the communication. There should be timeout.
	- Fatal Errors:
		I tried to tackle as many error cases I can, however in case of a Fatal Error where the I2C is not working at all, there should be a mechanism to let the participant know that the camera is not working. This could be achieved by flashing the LEDs on the board in a particular sequence and Writing a LOG file in to the SD card.
	- Putting the NRF chip to SLEEP:
		Once the Master device sends POW_OFF command, the BLE chip can put itself to sleep. There is a function in the code that does this, however it uses the buttons on the devboard to wake the chip up. The question is how would you wake up the BLE chip when it has to?
		Since the RTC on the BLE chip is not powered, I think STM32 should be responsible for waking up the NRF.
		Steps to Wake UP:
			1. Once POW_OFF command is recieved, disable/deinit the twis.
			2. Set one of the now unused twis pins to GPIO rising edge external interrupt
			3. Set this up as the wakeup source inside the sleep function.
		Steps on the STM32 side:
			Once the state machine is in DONE state:
			1. Disable the I2C module
			2. set one of the now unused pins as GPIO pin
			3. set the pin to be low.
			4. setup RTC alarm when to sync the device during the day.
			5. Once the alarm triggers an interrupt, raise the pin.
			6. wait a bit
			7. Renable the pins as I2C and init I2C module again.
			8. Poll NRF by using I2C

		This scheme will work if there is no phsyical pullup resistors on the PCB. Based on sleep_mode_enter() function in NRF, it wakes up, it'll cause a reset, so the pins will be automatically reconfigured as twi pins. instead of STM32 polling the NRF to check if it woke up, NRF can raise the interrupt pin to signal the wakeup.

	- Implement prepare_hbt function
		This function will problably be the same with the necklace. I wasn't able to test it since I don't have the SD card module for the dev board.

	- Resolving the Connection Glitch
		Right now BLE chip needs to erase the bonds to connect with the phone. This issue needs to be resolved.

Suggestions for integration
	First thing would be to flash the code to the PCB and test if the BLE is working. 
	
	Next, I would just take a look at the gtcam_stm32l4_mlx folder, particularly the SRC directory. Look at all the .c files there. Cross reference with the same Src directory in stm32_ble_controller and copy all the lines/functions that is missing in the gtcam_stm32l4_mlx to gtcam_stm32l4_mlx. Encapuslate the camera logic and place it under camera_operation() function.


