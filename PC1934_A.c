/* Source File */



#include "PAC1934_A.h"
/*
PAC1934 is a current and voltage sensor that is also capable of calculating power and energy values. 
This file is created for two identical sensors which are embedded on a development board. Therefore 
it also determines eight ports connected to the sensor outputs.

*/



/* --------------- DEFINES --------------------- */
uint8_t REFRESH 				= 0x00; // Assigned the address values to the corresponding registers.
uint8_t PAC_CTRL				= 0x01; 
uint8_t ACC_COUNT				= 0x02;
uint8_t VPOWER1_ACC				= 0x03;
uint8_t VPOWER2_ACC   				= 0x04;
uint8_t VPOWER3_ACC   				= 0x05;
uint8_t VPOWER4_ACC  				= 0x06;
uint8_t VBUS1 					= 0x07;
uint8_t VBUS2					= 0x08;
uint8_t VBUS3					= 0x09;
uint8_t VBUS4					= 0x0A;
uint8_t VSENSE1					= 0x0B;
uint8_t VSENSE2					= 0x0C;
uint8_t VSENSE3					= 0x0D;
uint8_t VSENSE4					= 0x0E;
uint8_t VBUS1_AVG				= 0x0F;
uint8_t VBUS2_AVG				= 0x10;
uint8_t VBUS3_AVG				= 0x11;
uint8_t VBUS4_AVG				= 0x12;
uint8_t VSENSE1_AVG				= 0x13;
uint8_t VSENSE2_AVG				= 0x14;
uint8_t VSENSE3_AVG				= 0x15;
uint8_t VSENSE4_AVG				= 0x16;
uint8_t VPOWER1					= 0x17;
uint8_t VPOWER2					= 0x18;
uint8_t VPOWER3					= 0x19;
uint8_t VPOWER4					= 0x1A;
uint8_t CHANNEL_DIS				= 0x1C;
uint8_t NEG_PWR					= 0x1D;
uint8_t REFRESH_G				= 0X1E;
uint8_t REFRESH_V				= 0X1F;
uint8_t SLOW					= 0X20;
uint8_t CTRL_ACT				= 0X21;
uint8_t CHANNEL_DIS_ACT 			= 0X22;
uint8_t NEG_PWR_ACT				= 0X23;
uint8_t CTRL_LAT				= 0X24;
uint8_t CHANNEL_DIS_LAT 			= 0X25;
uint8_t NEG_PWR_LAT 				= 0X26;
uint8_t PID					= 0XFD;	
uint8_t MID					= 0XFE;
uint8_t REV					= 0XFF;

uint8_t  pac_ctrl[1] 				= {0x00};   // Assigned the contents of registers.
uint8_t  channel[1]				= {0x00};   
uint8_t  neg[1]     				= {0xFF};   //Vsense and Vbus are determined as bidirectional measurements.
uint8_t  neg_r[1] 				= {0x00};		
uint8_t  slow[1] 				= {0x00};
   



double		d_temp;    					//Temporary variable
double		d_tempCurrent;					//Temporary variable
double		d_tempVoltage;					//Temporary variable
double		d_iSense;					//Sense current value
double		d_imax;						//Maximum current to measure
double		d_rSense;					//External rSense resistor value
double		d_powerFullScaleRange;				//Full scale range of power
double		d_fullScaleCurrent;				//Full scale current value
double		d_propPower;					//Propotional power value
double		d_powerReturn;					//Return value of power measurement
double		d_energyReturn = 0;				//Return value of energy measurement
int 			channel_number;				//Channel numbers corresponding to related port numbers
uint8_t		Vbus[2];					//Memory array for Vbus voltage value read from VBUS registers
uint8_t		Vsense[2];					//Memory array for Vsense voltage value read from VSENSE registers
uint8_t 	Vpower[4];					//Memory array for power value read from VPOWER registers
uint8_t 	Vpower_acc[6];  				//Memory array for accumulated power value read from VPOWER_ACC registers
uint8_t		Count[3]={0};					//Memory array for counter value read from ACC_COUNT register(initially assigned 0x00 to prevent conflict if any)
uint16_t	register_addr[1];				//Changable register address value according to the channel number inserted in main function
uint16_t	data_bus;					//Temporary variable used for voltage calculation
uint16_t 	data_current;					//Temporary variable used for current calculation
uint32_t 	data_power;					//Temporary variable used for power calculation
uint64_t 	data_energy;					//Temporary variable used for energy calculation
uint32_t i;




void PAC1934_init(void){
		// Checked whether the devices are ready for communication or not with the respond of the function below.
		if(HAL_I2C_IsDeviceReady(&hi2c1, DEV_ADDR_W1, 2, 200) == HAL_OK){
																																		 
			// Refresh command having an address of 0x00 is transmitted.																														 
	  	HAL_I2C_Master_Transmit(&hi2c1, DEV_ADDR_W1, &REFRESH, 1, 1000);
			HAL_Delay(200);
			// Writing to the address in the memory is done with using the necessary info.
			HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR_W1, PAC_CTRL, 1, &pac_ctrl[0], 1, 500);
			HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR_W1, NEG_PWR, 1, &neg[0], 1, 500 ); 
			HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR_W1, CHANNEL_DIS,1, &channel[0], 1, 500);
			HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR_W1, SLOW, 1, &slow[0], 1, 500 );
			
			//In order to changings to be updated refresh command is again transmitted.
			HAL_I2C_Master_Transmit(&hi2c1, DEV_ADDR_W1, &REFRESH ,1, 500);
			HAL_Delay(200);
			
		}
		// Initializing the second sensor.
		if (HAL_I2C_IsDeviceReady(&hi2c1, DEV_ADDR_W2, 2, 200) == HAL_OK){               
		
			HAL_I2C_Master_Transmit(&hi2c1, DEV_ADDR_W2, &REFRESH , 1, 1000);
			HAL_Delay(500);
		
			HAL_I2C_Mem_Write(&hi2c1,DEV_ADDR_W2, PAC_CTRL, 1, &pac_ctrl[0], 1, 500);		
			HAL_I2C_Master_Transmit(&hi2c1, DEV_ADDR_W2, &REFRESH ,1, 500);
			HAL_Delay(200);
		
			HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR_W2, CHANNEL_DIS,1, &channel[0], 1, 500);
			HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR_W2, NEG_PWR, 1, &neg[0], 1, 500 );
			HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR_W2, SLOW, 1, &slow[0], 1, 500 );
			HAL_I2C_Master_Transmit(&hi2c1, DEV_ADDR_W2, &REFRESH , 1, 500);
	
		}

}

void sendComment(void){


	HAL_I2C_Master_Transmit(&hi2c1, DEV_ADDR_W1, &REFRESH , 1, 500);  
	
	
	
	HAL_I2C_Master_Transmit(&hi2c1, DEV_ADDR_W2, &REFRESH , 1, 500);  
	HAL_Delay(200);
	
	
	
}


double VoltageMeasurement(int port_number){
	
		//A user could know the port numbers but the corresponding channel numbers.
		if(port_number==2 || port_number==5)   channel_number=1;
		if(port_number==3 || port_number==6)   channel_number=2;
		if(port_number==4 || port_number==7)   channel_number=3;
		if(port_number==9 || port_number==1)   channel_number=4;
		
		//Vbus addresses are 0x07, 0x08, 0x09 and 0x0A so they could be obtained by adding 0x06.With this way each channel is associated with the Vbus registers. 
		register_addr[0] = channel_number + 0x06;
			
		//Port number of 2,3,4 and 9 belong to a first sensor and so that sensor is initialized.
		if(port_number ==2 || port_number==3 || port_number==4 ||port_number ==9){
			//When integer variable used instead of an array, an error occured. Therefore an array must be used irrespective of the size of an data.
			HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR_R1, register_addr[0], 1, &Vbus[0], 2, 500); 
		}
		//Port number of 1,5,6, and 7 belong to a second sensor and so that sensor is initialized.
		if(port_number ==1 || port_number==5 || port_number==6 ||port_number ==7){  

			//Vbus is an 16 bit register and so 2 byte must be read.
			HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR_R2, register_addr[0], 1, &Vbus[0], 2, 500);
		}
		
		//Vbus value is finally stored in data_bus variable.
		data_bus= Vbus[0];
		data_bus<<=8;
		data_bus |= Vbus[1];
		
		if(data_bus >= 0x8000){
			data_bus = ( (0xFFFF)-(data_bus) )+1;            	//Two's complement 
			d_tempVoltage = (double) data_bus;                //Type casting must be done to convert the value into double initially.
			d_tempVoltage= (double) ( (d_tempVoltage * 32)/ (0xFFFF) );   // 0xFFFF is the biggest 28 bit-number to be written.
		}
		
		else{ 
			d_tempVoltage = (double) data_bus; 										
			d_tempVoltage= (double) ( (d_tempVoltage * 32)/ (0x7FFF) ); // 0x7FFF is the biggest 27 bit-number to be written.
		}
			
		return d_tempVoltage;
}
			


double CurrentMeasurement(int port_number){

		if(port_number==2 || port_number==5)   channel_number=1;
		if(port_number==3 || port_number==6)   channel_number=2;
		if(port_number==4 || port_number==7)   channel_number=3;
		if(port_number==9 || port_number==1)   channel_number=4;
	
		register_addr[0] = channel_number + 0x0A;
	
		if(port_number ==2 || port_number==3 || port_number==4 ||port_number ==9){

			HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR_R1, register_addr[0], 1, &Vsense[0], 2, 500);
		}
		if(port_number ==1 || port_number==5 || port_number==6 ||port_number ==7){

			HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR_R2, register_addr[0], 1, &Vsense[0], 2, 500);
		}
			
		data_current = Vsense[0];
		data_current <<= 8; 
		data_current |= Vsense[1]; 
	
		if(data_current >= 0x8000){										//0x8000 is the smallest negative number that could be written.
			data_current=((0xFFFF)-(data_current))+1; 	//Two's complement
			d_tempCurrent = (double) data_current; 
			d_imax = (double)(0.1f)/(0.004);    				// 0.004 :Rsense internal resistor value and 0.1 = 100mV full scale current value.
			d_rSense = (0.1f)/(d_imax);          				// I max : maximum current to be measured.
			d_fullScaleCurrent=(0.1f)/ d_rSense;				
			d_iSense = (d_fullScaleCurrent * d_tempCurrent ) / (0xFFFF); // iSense: Actual current value.
		}
		else{
			d_tempCurrent = (double) data_current; 
			d_imax = (double)(0.1f)/(0.004);    
			d_rSense = (0.1f)/(d_imax);
			d_fullScaleCurrent=(0.1f)/ d_rSense;
			d_iSense = (d_fullScaleCurrent * d_tempCurrent ) / (0x7FFF);  
		}
		return d_iSense;
			
			

}


double PowerMeasurement(int port_number){

		d_imax = (double)(0.1f)/(0.004);
		d_rSense = (0.1f)/(d_imax);
		d_powerFullScaleRange = (double)(( (0.1f)/(d_rSense) ) * 32);  // 32: Full Scale voltage by default.
	
	
		if(port_number==2 || port_number==5)   channel_number=1;
		if(port_number==3 || port_number==6)   channel_number=2;
		if(port_number==4 || port_number==7)   channel_number=3;
		if(port_number==9 || port_number==1)   channel_number=4;
	
		//Vpower registers' addresses begin with 0x17.
		register_addr[0] = channel_number + 0x16;
	
		if(port_number ==2 || port_number==3 || port_number==4 ||port_number ==9){

		// Because Vpower is 32 bit register, 4 byte must be read.
		HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR_R1, register_addr[0], 1, &Vpower[0], 4, 500);
		}
		if(port_number ==1 || port_number==5 || port_number==6 ||port_number ==7){

			HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR_R2, register_addr[0], 1, &Vpower[0], 4, 500);
		}
		// Vpower is a 32 bit array and it is stored in the data_power.
		data_power = Vpower[0];
		data_power<<=8;
		data_power |= Vpower[1];
		data_power<<=8;
		data_power |= Vpower[2];
		data_power<<=8;
		data_power |= Vpower[3];	

	
		if(data_power >= 0x80000000){																	//0x80000000 is the smallest negative number that could be written.
			data_power=((0xFFFF)-(data_power))+1;												//Two's complement
			d_propPower = ( (double) data_power / (0xFFFFFFF0) );				// The most right 4 bits are unused.*******
			d_powerReturn = ( d_powerFullScaleRange * d_propPower );
		}

		else{
			d_propPower = ( (double) data_power * d_powerFullScaleRange ); 
			d_powerReturn = (d_propPower / 0x7FFFFFF0);     					  // The most right 4 bits are unused.*******
		}
		
		return d_powerReturn;
		

	}

double EnergyMeasurement(int port_number){
	
		float calc;
		d_imax = (double)(0.1f) /(0.004);
		d_rSense = (0.1f) /(d_imax);
		d_powerFullScaleRange = (double)(( (0.1f) /(d_rSense) ) * 32);
	
		if(port_number==2 || port_number==5)   channel_number=1;
		if(port_number==3 || port_number==6)   channel_number=2;
		if(port_number==4 || port_number==7)   channel_number=3;
		if(port_number==9 || port_number==1)   channel_number=4;
		
		// Counter register address is 0x02.
		register_addr[0] = 0x02;
		
		if(port_number ==2 || port_number==3 || port_number==4 ||port_number ==9){
				
			calc = (2.5f);
			
			//Counter register must be read even it is not used in the equation.
			HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR_R1, register_addr[0], 1, Count, 3, 200);
			
			//Vpower_acc registers' addresses begin with 0x03.
			register_addr[0] = channel_number + 0x02;
				
			//Vpower_acc is a 48 bit array. 6 byte must be read.
			HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR_R1, register_addr[0], 1, &Vpower_acc[0], 6, 500);
		
		}
			
		if(port_number ==1 || port_number==5 || port_number==6 ||port_number ==7){
				
			calc = 250;

			HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR_R1, register_addr[0], 1, Count, 3, 200);
			register_addr[0] = channel_number + 0x02;
			HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR_R2, register_addr[0], 1, &Vpower_acc[0], 6, 500);
		}
		
		//48 bit value is finally stored in data_energy.
		data_energy = Vpower_acc[0];
		data_energy<<=8;
		data_energy |=Vpower_acc[1];
		data_energy<<=8;
		data_energy |= Vpower_acc[2];	
		data_energy<<=8;
		data_energy |= Vpower_acc[3];
		data_energy<<=8;
		data_energy |= Vpower_acc[4];
		data_energy<<=8;
		data_energy |= Vpower_acc[5];
		data_energy = (double)data_energy;
	
		if (data_energy >= 0x800000000000){             			 
			data_energy=( (0xFFFFFFFFFFFF)-(data_energy) )+1;		// Two's complement.
			//     1024: sampling rate     and    0xFFFFFFFF: 28 bit range of number.
			d_energyReturn	= ( (double)data_energy / (0xFFFFFFFF)  ) * ( d_powerFullScaleRange / 1024 );   
		}
		else{
			d_temp	= ((double)data_energy * d_powerFullScaleRange); 
			d_temp  = d_temp / (0x7FFFFFF);  
			d_temp  = d_temp / (1024);
			d_temp	=	d_temp * calc; // new line

			
			d_energyReturn = d_temp;
		}

			
			
			
		return d_energyReturn;
		
}
 
double TimeMeasurement(int port_number){

		for(i=1 ; i<10 ; i++){
	
		Time_J[i] =	 ( Energy_J[i] / Power_J[i] ) * 1000;

		}


}

