// This where the code to handle motor controller errors and such will go


#include "motor_controller.h"

// We need to include can.h because we will send CAN messages through the functions in that file
// When a CAN message comes it will throw an interrupt can.c deals with the incoming message
// the function in can.c gets the ID and sends the data to the functions here
#include "can.h"
#include "main.h"
#include "constants.h"

// We need to include pdu.h for the shutdown circuit
#include "pdu.h"


// the calculations for speed/torque control happen in driving loop

const uint32_t MC_Expected_Serial_Number = 0x627E7A01;
const uint16_t MC_Expected_FW_Version = 0xDC01;


void MC_Parse_Message(int DLC, uint8_t Data[]){
	// The first step is to look at the first byte to figure out what we're looking at

	// Then, we pass the rest of the message to the appropriate handlers
	// The appropriate handlers need to ignore the first byte of Data[]
	switch (Data[0]){
		// important checks
		case motor_controller_errors_warnings:
			MC_Check_Error_Warning(Data);
		break;

		case N_actual:
			MC_Parse_Actual_Speed(Data);
		break;

		case serial_number:
			MC_Check_Serial_Number(Data);
		break;

		case firmware_version:
			MC_Check_Firmware(Data);
		break;

		default: // This is a code that is not recognized (bad)
			Error_Handler();
		break;
	}

}

// this function is used to get data from the motor controller
void MC_Request_Data(uint8_t RegID){

	TxHeader.StdId = MC_CAN_ID_Tx;
	TxHeader.DLC = 2;
	TxData[0] = 0x3D; // this is the code to request data from MC
	TxData[1] = RegID;

	if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK){
		/* Transmission request Error */
		Error_Handler();
	}

}



// We can either send 2 or 4 bytes of data
// This will correspond to a DLC of 3 or 5, respectively
// The first byte needs to be the register ID, then either 2 or 4 bytes of data
// The number size is either 2 or 4 and is very important
// If the motor controller wants 0x1234, the input to this function should be 0x1234
void MC_Send_Data(uint8_t RegID, uint8_t data_to_send[], uint8_t size){

	TxHeader.StdId = MC_CAN_ID_Tx;
	TxHeader.DLC = size;
	TxData[0] = RegID;

	switch (size){
	case 2:
		TxData[1] = data_to_send[2];
		TxData[2] = data_to_send[1];
	break;

	case 4:
		TxData[1] = data_to_send[4];
		TxData[2] = data_to_send[3];
		TxData[3] = data_to_send[2];
		TxData[4] = data_to_send[1];
	break;

	default:
		// bad
	break;
	}

	if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK){
		/* Transmission request Error */
		Error_Handler();
	}

}

void MC_Parse_Actual_Speed(uint8_t Data[]){
	// TODO
}


void MC_Check_Error_Warning(uint8_t Data[]){

	// The motor controller will send a message with 6 bytes of data
	// The first byte is the register ID, which in this case is 0x8F
	// The second and third byte are low bytes which correspond to errors
	// The order of bits in the low bytes is
	// 7 6 5 4 3 2 1 0  15 14 13 12 11 10 9 8

	// The fourth and fifth bytes are high bytes and correspond to warnings
	// The order of bits in the high bytes is
	// 23 22 21 20 19 18 17 16   31 30 29 28 27 26 25 24

	// We need to and the bytes with the right bitmasks and check for errors


	// Split four bytes into 2 high and 2 low bytes

	uint16_t MC_errors = (Data[1] << 8) | Data[2];
	uint16_t MC_warnings = (Data[3] << 8) | Data[4];

	// All the error flags should be zero
	if (MC_errors){
		// shutdown here

		// Compare errors to errors bitmask to determine error
		if (MC_errors & eprom_read_error){
			// bad
		}
		if (MC_errors & hardware_fault){
			// bad
		}
		if (MC_errors & rotate_field_enable_not_present_run){
			// bad
		}
		if (MC_errors & CAN_timeout_error){
			// bad
		}
		if (MC_errors & feedback_signal_error){
			// bad
		}
		if (MC_errors & mains_voltage_min_limit){
			// bad
		}
		if (MC_errors & motor_temp_max_limit){
			// bad
		}
		if (MC_errors & IGBT_temp_max_limit){
			// bad
		}
		if (MC_errors & mains_voltage_max_limit){
			// bad
		}
		if (MC_errors & critical_AC_current){
			// bad
		}
		if (MC_errors & race_away_detected){
			// bad
		}
		if (MC_errors & ecode_timeout_error){
			// bad
		}
		if (MC_errors & watchdog_reset){
			// bad
		}
		if (MC_errors & AC_current_offset_fault){
			// bad
		}
		if (MC_errors & internal_hardware_voltage_problem){
			// bad
		}
		if (MC_errors & bleed_resistor_overload){
			// bad
		}
	}

	if (MC_warnings){
		// suspend or shutdown here

		// compare warnings to warnings bitmask
		if (MC_warnings & parameter_conflict_detected){
			// not great
		}
		if (MC_warnings & special_CPU_fault){
			// not great
		}
		if (MC_warnings & rotate_field_enable_not_present_norun){
			// not great
		}
		if (MC_warnings & auxiliary_voltage_min_limit){
			// not great
		}
		if (MC_warnings & feedback_signal_problem){
			// not great
		}
		if (MC_warnings & warning_5){
			// not great
		}
		if (MC_warnings & motor_temperature_warning){
			// not great
		}
		if (MC_warnings & IGBT_temperature_warning){
			// not great
		}
		if (MC_warnings & Vout_saturation_max_limit){
			// not great
		}
		if (MC_warnings & warning_9){
			// not great
		}
		if (MC_warnings & speed_actual_resolution_limit){
			// not great
		}
		if (MC_warnings & check_ecode_ID ){
			// not great
		}
		if (MC_warnings & tripzone_glitch_detected){
			// not great
		}
		if (MC_warnings & ADC_sequencer_problem){
			// not great
		}
		if (MC_warnings & ADC_measurement_problem){
			// not great
		}
		if (MC_warnings & bleeder_resistor_warning){
			// not great
		}
	}
}




// -----------------------------------------------------------------------------------
// Startup checks

void MC_Check_Serial_Number(uint8_t Data[]){
	// the serial number is 4 bytes of data
	uint8_t MC_Read_Serial_Numbers[4] = {0};
	uint32_t MC_Read_Serial_Number = 0;

	for (int i = 0; i < 4; ++i){
		MC_Read_Serial_Numbers[i] = Data[i+1];
	}

	MC_Read_Serial_Number = (MC_Read_Serial_Numbers[0] << 24) | (MC_Read_Serial_Numbers[1] << 16)
							| (MC_Read_Serial_Numbers[2] << 8) | MC_Read_Serial_Numbers[3];

	if (MC_Read_Serial_Number != MC_Expected_Serial_Number){
		// bad
		// add what we want to do

		// return;
	}
	// if this code runs, serial number good
}

void MC_Check_Firmware(uint8_t Data[]){
	uint8_t MC_Read_Firmwares[2] = {0};
	uint32_t MC_Read_Firmware = 0;


	MC_Read_Firmwares[0] = Data[1];
	MC_Read_Firmwares[1] = Data[2];

	MC_Read_Firmware = (MC_Read_Firmwares[0] << 8) | MC_Read_Firmwares[1];

	if (MC_Read_Firmware != MC_Expected_FW_Version){
		// bad
		// add what we want to do

		// return;
	}
	// if this code runs, firmware good
}

void MC_Startup(){
	MC_Request_Data(serial_number);
	MC_Request_Data(firmware_version);


}



