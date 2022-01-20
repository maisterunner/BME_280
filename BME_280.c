#include "BME_280.h"
#include "main.h"

/* --------
	
	LUT definitions
	
-------- */

const float BME_altitude_LUT[] = { 1948.989172, 1926.107253, 1903.277761, 1880.500430, 1857.774994, 1835.101189, 1812.478754, 1789.907429, 1767.386958, 1744.917084, 1722.497555, 1700.128119, 1677.808526, 1655.538529, 1633.317883, 1611.146343, 1589.023668, 1566.949617, 1544.923953, 1522.946439, 1501.016841, 1479.134925, 1457.300460, 1435.513218, 1413.772970, 1392.079491, 1370.432556, 1348.831942, 1327.277429, 1305.768798, 1284.305830, 1262.888310, 1241.516023, 1220.188756, 1198.906298, 1177.668439, 1156.474970, 1135.325685, 1114.220378, 1093.158846, 1072.140885, 1051.166296, 1030.234878, 1009.346434, 988.500767, 967.697681, 946.936983, 926.218480, 905.541980, 884.907295, 864.314236, 843.762614, 823.252245, 802.782944, 782.354527, 761.966813, 741.619620, 721.312770, 701.046083, 680.819382, 660.632493, 640.485239, 620.377448, 600.308947, 580.279565, 560.289132, 540.337479, 520.424439, 500.549844, 480.713529, 460.915331, 441.155085, 421.432629, 401.747802, 382.100445, 362.490397, 342.917502, 323.381602, 303.882541, 284.420165, 264.994319, 245.604850, 226.251608, 206.934440, 187.653196, 168.407729, 149.197889, 130.023531, 110.884506, 91.780671, 72.711882, 53.677994, 34.678865, 15.714354, -3.215679, -22.111376, -40.972874, -59.800311, -78.593825, -97.353553 };

const float BME_alt_grad_LUT[] = { -22.881920, -22.829491, -22.777331, -22.725436, -22.673805, -22.622435, -22.571325, -22.520471, -22.469874, -22.419529, -22.369436, -22.319593, -22.269997, -22.220646, -22.171540, -22.122675, -22.074050, -22.025664, -21.977514, -21.929599, -21.881916, -21.834464, -21.787242, -21.740248, -21.693479, -21.646935, -21.600613, -21.554513, -21.508631, -21.462968, -21.417520, -21.372287, -21.327267, -21.282458, -21.237859, -21.193469, -21.149285, -21.105307, -21.061532, -21.017960, -20.974589, -20.931418, -20.888444, -20.845667, -20.803086, -20.760698, -20.718503, -20.676499, -20.634685, -20.593060, -20.551621, -20.510369, -20.469301, -20.428417, -20.387714, -20.347193, -20.306851, -20.266687, -20.226700, -20.186890, -20.147254, -20.107791, -20.068501, -20.029382, -19.990433, -19.951653, -19.913041, -19.874595, -19.836314, -19.798199, -19.760246, -19.722456, -19.684827, -19.647357, -19.610047, -19.572895, -19.535900, -19.499061, -19.462376, -19.425846, -19.389468, -19.353243, -19.317168, -19.281243, -19.245467, -19.209840, -19.174359, -19.139024, -19.103835, -19.068790, -19.033888, -18.999129, -18.964511, -18.930034, -18.895696, -18.861498, -18.827437, -18.793514, -18.759727 };

// const float BME_P_LUT[] = { 80000.000000, 80227.272727, 80454.545455, 80681.818182, 80909.090909, 81136.363636, 81363.636364, 81590.909091, 81818.181818, 82045.454545, 82272.727273, 82500.000000, 82727.272727, 82954.545455, 83181.818182, 83409.090909, 83636.363636, 83863.636364, 84090.909091, 84318.181818, 84545.454545, 84772.727273, 85000.000000, 85227.272727, 85454.545455, 85681.818182, 85909.090909, 86136.363636, 86363.636364, 86590.909091, 86818.181818, 87045.454545, 87272.727273, 87500.000000, 87727.272727, 87954.545455, 88181.818182, 88409.090909, 88636.363636, 88863.636364, 89090.909091, 89318.181818, 89545.454545, 89772.727273, 90000.000000, 90227.272727, 90454.545455, 90681.818182, 90909.090909, 91136.363636, 91363.636364, 91590.909091, 91818.181818, 92045.454545, 92272.727273, 92500.000000, 92727.272727, 92954.545455, 93181.818182, 93409.090909, 93636.363636, 93863.636364, 94090.909091, 94318.181818, 94545.454545, 94772.727273, 95000.000000, 95227.272727, 95454.545455, 95681.818182, 95909.090909, 96136.363636, 96363.636364, 96590.909091, 96818.181818, 97045.454545, 97272.727273, 97500.000000, 97727.272727, 97954.545455, 98181.818182, 98409.090909, 98636.363636, 98863.636364, 99090.909091, 99318.181818, 99545.454545, 99772.727273, 100000.000000, 100227.272727, 100454.545455, 100681.818182, 100909.090909, 101136.363636, 101363.636364, 101590.909091, 101818.181818, 102045.454545, 102272.727273, 102500.000000 };


/* --------
	
	Functions
	
-------- */


/* --------
	BME setup
-------- */

void BME_280_mem_map_init(BME_data *input, I2C_HandleTypeDef *hi2c){
	// struct BME_mem_map BME_struct;
	// Remember to set SDO low to read and high to write

	// T
	input->dig_T[0] = BME_T1_1;
	input->dig_T[1] = BME_T1_2;
	input->dig_T[2] = BME_T2_1;
	input->dig_T[3] = BME_T2_2;
	input->dig_T[4] = BME_T3_1;
	input->dig_T[5] = BME_T3_2;
	
	// P
	input->dig_P[0] = BME_P1_1;
	input->dig_P[1] = BME_P1_2;
	input->dig_P[2] = BME_P2_1;
	input->dig_P[3] = BME_P2_2;
	input->dig_P[4] = BME_P3_1;
	input->dig_P[5] = BME_P3_2;
	input->dig_P[6] = BME_P4_1;
	input->dig_P[7] = BME_P4_2;
	input->dig_P[8] = BME_P5_1;
	input->dig_P[9] = BME_P5_2;
	input->dig_P[10] = BME_P6_1;
	input->dig_P[11] = BME_P6_2;
	input->dig_P[12] = BME_P7_1;
	input->dig_P[13] = BME_P7_2;
	input->dig_P[14] = BME_P8_1;
	input->dig_P[15] = BME_P8_2;
	input->dig_P[16] = BME_P9_1;
	input->dig_P[17] = BME_P9_2;
	
	// H
	input->dig_H[0] = BME_H1;
	input->dig_H[1] = BME_H2_1;
	input->dig_H[2] = BME_H2_2;
	input->dig_H[3] = BME_H3;
	input->dig_H[4] = BME_H4_1;
	input->dig_H[5] = BME_H4_2;
	input->dig_H[6] = BME_H5_1;
	input->dig_H[7] = BME_H5_2;
	input->dig_H[8] = BME_H6;
	
	// Interface
	input->i2c_Handle = hi2c;
	input->trig[0] = BME_ctrl_meas;
	input->trig[1] = BME_ctrl_trig;
	
	// Vars
	input->index_T = 0;
	input->buf_T[0] = 0;
	input->buf_T[1] = 0;
	input->buf_T[2] = 0;
	input->buf_T[3] = 0;
	input->buf_T[4] = 0;
	input->buf_T[5] = 0;
	input->buf_T[6] = 0;
	input->buf_T[7] = 0;

}

uint8_t BME_280_Config_ReadOut(BME_data *input){
	
	uint8_t buf[24];
	HAL_StatusTypeDef BME_stat;

	buf[0] = BME_T1_1;

	// Read out calibration data (T and P)
	BME_stat = HAL_I2C_Master_Transmit(input->i2c_Handle, (uint16_t)BME_ADDR_W, buf, 1, HAL_MAX_DELAY);
	if( BME_stat != HAL_OK ){
		return 0;
	}
	else{
		// BME_stat = HAL_I2C_Master_Receive(input->i2c_Handle, (uint16_t)BME_ADDR_R, &buf, (uint8_t*)24, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(input->i2c_Handle, (uint16_t)BME_ADDR_R, buf, 24, HAL_MAX_DELAY);
		if( BME_stat != HAL_OK ){
			return 0;
		}
		else{
			input->dig_T[0] = (uint16_t)((buf[0] << 8) | buf[1]);
			input->dig_T[1] = (int16_t)((buf[2] << 8) | buf[3]);
			input->dig_T[2] = (int16_t)((buf[4] << 8) | buf[5]);
			input->dig_P[0] = (uint16_t)((buf[6] << 8) | buf[7]);
			input->dig_P[1] = (int16_t)((buf[8] << 8) | buf[9]);
			input->dig_P[2] = (int16_t)((buf[10] << 8) | buf[11]);
			input->dig_P[3] = (int16_t)((buf[12] << 8) | buf[13]);
			input->dig_P[4] = (int16_t)((buf[14] << 8) | buf[15]);
			input->dig_P[5] = (int16_t)((buf[16] << 8) | buf[17]);
			input->dig_P[6] = (int16_t)((buf[18] << 8) | buf[19]);
			input->dig_P[7] = (int16_t)((buf[20] << 8) | buf[21]);
			input->dig_P[8] = (int16_t)((buf[22] << 8) | buf[23]);
		}
	}

	// Read out Read out calibration data (H and id)
	buf[0] = BME_H1;
	buf[1] = BME_H2_1;
	buf[2] = BME_H2_2;
	buf[3] = BME_H3;
	buf[4] = BME_H4_1;
	buf[5] = BME_H4_2;
	buf[6] = BME_H5_2;
	buf[7] = BME_H6;
	buf[8] = BME_chip_id;
	
	for(uint8_t i = 0; i < 9; i++){
		BME_stat = HAL_I2C_Master_Transmit(input->i2c_Handle, (uint16_t)BME_ADDR_W, &buf[i], 1, HAL_MAX_DELAY);
		if( BME_stat != HAL_OK ){
			return 0;
		}
		else{
			BME_stat = HAL_I2C_Master_Receive(input->i2c_Handle, (uint16_t)BME_ADDR_R, &buf[i], 1, HAL_MAX_DELAY);
			if( BME_stat != HAL_OK ){
				return 0;
			}
		}
	}
	
	input->dig_H[0] = (uint8_t)buf[0];
	input->dig_H[1] = (int16_t)((buf[1] << 8) | buf[2]);
	input->dig_H[2] = (uint8_t)buf[3];
	input->dig_H[3] = (int16_t)((buf[4] << 4) | (buf[5] & 0x0F));
	input->dig_H[4] = (int16_t)(((buf[5] & 0xF0) << 4) | buf[6]);
	input->dig_H[5] = (int8_t)buf[7];
	input->id = (uint8_t)buf[8];

	if( input->id != 0x60 ){
		return 0;
	}
	else{
		return 1;
	}
}

uint8_t BME_280_InitWrite(BME_data *input){
	// Initializes settings for the measurement
	// Registers written
	// 0xE0 "reset" --> 0xB6 
	// 		- reset procedure performed
	// 0xF2 "ctrl_hum" --> 0x00
	// 		- humidity over-sampling x0 -> skipped
	// 0xF4 "ctrl_meas"  --> 0x28
	// 		- sleep mode
	//		- pressure over-sampling x2
	//		- temperature over-sampling x1
	// 0xF5 "config" --> 0x04
	// 		- IIR filter constant 2
	
	uint8_t buf[8];
	HAL_StatusTypeDef BME_stat;

	buf[0] = BME_reset;
	buf[1] = 0xB6;
	buf[2] = BME_ctrl_hum;
	buf[3] = 0x00;
	buf[4] = BME_ctrl_meas;
	buf[5] = 0x28;
	buf[6] = BME_config;
	buf[7] = 0x04;


	for( int8_t i=0; i <= 3; i++ ){				// Write configuration
		BME_stat = HAL_I2C_Master_Transmit(input->i2c_Handle, (uint16_t)BME_ADDR_W, &buf[2*i], 2, HAL_MAX_DELAY);
			if( BME_stat != HAL_OK ){
				return 0;
			}
	}

	for( int8_t i=0; i <= 3; i++ ){				// Check configuration
		BME_stat = HAL_I2C_Master_Transmit(input->i2c_Handle, (uint16_t)BME_ADDR_W, &buf[2*i], 1, HAL_MAX_DELAY);
		if( BME_stat != HAL_OK ){
			return 0;
		}
		else{
			BME_stat = HAL_I2C_Master_Receive(input->i2c_Handle, (uint16_t)BME_ADDR_R, &buf[2*i], 1, HAL_MAX_DELAY);
			if( BME_stat != HAL_OK ){
				return 0;
			}
			else if( buf[2*i] != buf[2*i + 1] ){
				if( (i == 0 && buf[0] != 0) | (i != 0)  ){
					return 0;
				}
			}
		}
	}

	return 1;
}


/* --------
	Taking measurements
-------- */


uint8_t BME_TriggerConversion_DMA(BME_data *input){
	// Trigger new measurement
	
	if( HAL_I2C_Master_Transmit_DMA(input->i2c_Handle, (uint16_t)BME_ADDR_W, input->trig, 2) == HAL_OK ){
		// Set trig flag
  		input->trig_flag = 1;
		return 1;
  	}
	else{
		return 0;
	}
}


// Selective read
/*
uint8_t BME_SendAddr_DMA(BME_data *input){
	// Send addr to read from
	
	uint8_t addr;
	
	// Only read if previously triggered
	if(input->trig_flag == 0){
		BME_TriggerConversion_DMA(input);
		return 0;
	}
	else if(input->read_cntr_T < BME_T_cntr_max && input->read_cntr_P < BME_P_cntr_max){
		input->read_cntr_T++;
		input->read_cntr_P++;
		return 0;
	}
	else if(input->read_cntr_T == BME_T_cntr_max){
		addr = BME_T_addr;
	}
	else if(input->read_cntr_P == BME_P_cntr_max){
		addr = BME_P_addr;
	}
	else{
		input->trig_flag = 0;
		return 0;
	}

  	if( HAL_I2C_Master_Transmit_DMA(input->i2c_Handle, (uint16_t)BME_ADDR_W, &addr, 1) == HAL_OK ){
		// Set flag for reading is busy
  		input->reading_data = 1;
		input->trig_flag = 0;
		return 1;
  	}
	else{
		return 0;
	}
}	
*/


uint8_t BME_SendAddr_DMA(BME_data *input){
	if( HAL_I2C_Master_Transmit_DMA(input->i2c_Handle, (uint16_t)BME_ADDR_W, (uint8_t*)BME_P_addr, 1) == HAL_OK ){
		// Set flag for reading is busy
	  	input->reading_data = 1;
		input->trig_flag = 0;
		return 1;
	  }
	else{
		return 0;
	}
}


uint8_t BME_ReadData_DMA(BME_data *input){
	// Read the data
	if(input->read_cntr_P < 500){
		if( HAL_I2C_Master_Receive_DMA(input->i2c_Handle, (uint16_t)BME_ADDR_R, input->raw, 6) == HAL_OK ){
			input->read_cntr_P++;
			input->read_P = 1;
			return 1;
		}
		else{
			return 0;
		}
	}
	else if(input->read_cntr_P == 500){
		if( HAL_I2C_Master_Receive_DMA(input->i2c_Handle, (uint16_t)BME_ADDR_R, input->raw, 6) == HAL_OK ){
			input->read_cntr_P = 1;
			input->read_P = 1;
			return 1;
		}
		else{
			return 0;
		}
	}
	else{
		input->read_cntr_P = 0;
		return 0;
	}
}

uint8_t BME_ProcMeas(BME_data *input){
	// Collect up data processing of read raw data
	if(input->read_P == 1){		// if p is read calculate pressure then altitude
		BME_AssembleP(input);
		BME_AssembleT(input);
		BME_compensate_P_fast(input);
		Calc_Barometric_LUT(input, BME_altitude_LUT, BME_alt_grad_LUT);

		input->read_P = 0;
		return 1;
	}
	else if(input->read_T == 1){		// if t is read calculate temp and temp compensation of
		BME_AssembleT(input);
		BME_compensate_T(input);
		BME_compensate_P_precalc(input);

		input->read_T = 0;
		return 1;
	}
	else{
		return 0;
	}
}


/* --------
	Calc float vals
-------- */
void BME_AssembleT(BME_data *input){
	// Assemble measurement data /temp/

	int32_t buf;
	
	// Clear flag
  	input->reading_data = 0;
	
	input->buf_T[input->index_T] = (input->raw[3] << 12) | (input->raw[4] << 4) | (input->raw[5] >> 4);

	buf = input->buf_T[0] + input->buf_T[1];
	buf += input->buf_T[2];
	buf += input->buf_T[3];
	buf += input->buf_T[4];
	buf += input->buf_T[5];
	buf += input->buf_T[6];
	buf += input->buf_T[7];

	input->adc_T = buf >> 3;

	input->index_T++;
	if(input->index_T > 7){
		input->index_T = 0;
	}

}


void BME_AssembleP(BME_data *input){
	// Assemble measurement data /pressure/
	
	int32_t buf;

	// Clear flag
  	input->reading_data = 0;
	
	input->adc_P = (input->raw[0] << 12) | (input->raw[1] << 4) | (input->raw[2] >> 4);

	buf = input->buf_T[0] + input->buf_T[1];

	input->adc_T = buf >> 1;

	if(input->index_P == 1){
		input->index_P = 0;
	}
	else{
		input->index_P = 0;
	}
}


void BME_AssembleH(BME_data *input){
	// Assemble measurement data /pressure/

	// Clear flag
  	input->reading_data = 0;

  	input->adc_H = (input->raw[6] << 8) | input->raw[7];
}


/* --------
	LUT
-------- */

/*
// OLD SLOW MATH, REPLACED WITH CONST FLOAT ARRAYS
void Init_Barometric_LUT(void){
	// Initialize the barometric LUT --> fill it up with values
	// Range is from 80 000 Pa to 102 500 Pa
	// Corresponding altitudes are 2036.7 m and -97.049 m

	for(int8_t i = 0; i < BME_table_length; i++)
	{
		BME_P_LUT[i] = BME_p_sat_low + i*((float)(BME_p_sat_high - BME_p_sat_low)/(BME_table_length-1));
		BME_h_LUT[i] = BME_Tb/BME_Lb * (pow((BME_P_LUT[i]/(float)BME_Pb),((-BME_R*BME_Lb)/(BME_g*BME_M))) - 1 );
	}
}
*/

void Calc_Barometric_LUT( BME_data *input, const float *BME_altitude_LUT, const float *BME_alt_grad_LUT ){
	// Calculate the lookup table altitude value from pressure measurement

	uint8_t done = 0;

	if ( input->Pf > BME_PSAT_H ) // counteract high saturation
	{
		input->Pf = BME_PSAT_H;
		input->altitude = BME_altitude_LUT[0];
		done = 1;
	}
	else if( input->Pf < BME_PSAT_L ) // counteract low saturation
	{
		input->Pf = BME_PSAT_L;
		input->altitude = BME_altitude_LUT[BME_table_length-1];
		done = 1;
	}

	// Define output
	if(done == 0){
		float t, P_relative;
		uint8_t index;
	
		P_relative = input->Pf - BME_PSAT_L;
	
		index = (uint8_t)(P_relative / BME_LUT_Inc);
	
		// Interpolation y = y0 + (x-x0)*((y1-y0)/(x1-x0))
		t = (P_relative - index * BME_LUT_Inc);								// (x-x0)
		t *= *( BME_alt_grad_LUT + index );									// Gradient from lookup table  (x-x0)*((y1-y0)/(x1-x0))

		input->altitude = *(BME_altitude_LUT + index) + t;
	}
}

	/*
	// OLD SLOW STUFF
	// Find the index of P_meas
	for( int8_t i = 0; i <= BME_table_length; i++ )
	{
		if ( ( BME_P_LUT[i] <= P_meas ) && ( BME_P_LUT[i+1] > P_meas ) )
		{
			t = (P_meas - BME_P_LUT[i]); 	// Interpolate
			t /= (BME_P_LUT[i+1] - BME_P_LUT[i]);

			BME_calc_alt = BME_h_LUT[i] + t * ( BME_h_LUT[i+1] - BME_h_LUT[i] );
		}
	}
	return BME_calc_alt;
	*/


/* --------
	Compenstaions
-------- */


void BME_compensate_T(BME_data *input){
	// Gives temp. in degC, resolution is 0.01 degC
	// Output of '1234' equals to 12.34 degC
	// t_fine carries raw temp. values as global variable

	int32_t var1, var2;

	var1 = ((((input->adc_T >> 3) - ((int32_t)input->dig_T[0] << 1))) * ((int32_t)input->dig_T[1])) >> 11;
	var2 = (((((input->adc_T >> 4) - ((int32_t)input->dig_T[0])) * ((input->adc_T >> 4) - ((int32_t)input->dig_T[0]))) >> 12) * ((int32_t)input->dig_T[2])) >> 14;

	input->t_fine = var1 + var2;
	input->T = (input->t_fine * 5 + 128) >> 8;
}

void BME_float_T(BME_data *input){
	// Calculates the float value of the compensated temperature
	
	input->Tf = (float)input->T / 100.0f;
}

void BME_compensate_P_precalc(BME_data *input){
	// Calculate var1 and var2 assuming T measurement has to be taken at lower frequency
	
	input->Pvar1 = ((int64_t)input->t_fine) - 128000;

	input->Pvar2 = input->Pvar1 * input->Pvar1 * (int64_t)input->dig_P[5];
	input->Pvar2 = input->Pvar2 + ((input->Pvar1 * (int64_t)input->dig_P[4]) << 17);
	input->Pvar2 = input->Pvar2 + (((int64_t)input->dig_P[3]) << 35);

	input->Pvar1 = ((input->Pvar1 * input->Pvar1 * (int64_t)input->dig_P[2]) >> 8) + ((input->Pvar1 * (int64_t)input->dig_P[1]) << 12);
	input->Pvar1 = ((((int64_t)1) << 47) + input->Pvar1) * ((int64_t)input->dig_P[0]) >> 33;

	if (input->Pvar1 == 0)
	{
		input->Pvar1 = 1; // avoid dividing by 0 (original instruction return 0)
	}
}


void BME_compensate_P_fast(BME_data *input){
	// takes the precalculated values to accelerate compensation
	
	// new var1 and var2 is defined so that only the precalc fuction edits the precalculated temp vals.
	int64_t var1, var2, p;
	
	p = 1048576 - input->adc_P;
	p = (((p << 31) - input->Pvar2) * 3125) / input->Pvar1;

	var1 = (((int64_t)input->dig_P[8]) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)input->dig_P[7]) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((int64_t)input->dig_P[6]) << 4);

	input->P = (uint32_t)p;
}


void BME_compensate_P(BME_data *input){
	// Returns 32bit unsigned integer
	// Output value of '24674867' represents 24674867 / 256 = 96386.2 Pa

	int64_t var1, var2, p;

	var1 = ((int64_t)input->t_fine) - 128000;

	var2 = var1 * var1 * (int64_t)input->dig_P[5];
	var2 = var2 + ((var1 * (int64_t)input->dig_P[4]) << 17);
	var2 = var2 + (((int64_t)input->dig_P[3]) << 35);

	var1 = ((var1 * var1 * (int64_t)input->dig_P[2]) >> 8) + ((var1 * (int64_t)input->dig_P[1]) << 12);
	var1 = ((((int64_t)1) << 47) + var1) * ((int64_t)input->dig_P[0]) >> 33;

	if (var1 == 0)
	{
		var1 = 1; // avoid dividing by 0 (original instruction return 0)
	}

	p = 1048576 - input->adc_P;
	p = (((p << 31) - var2) * 3125) / var1;

	var1 = (((int64_t)input->dig_P[8]) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)input->dig_P[7]) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((int64_t)input->dig_P[6]) << 4);

	input->P = (uint32_t)p;
}

void BME_float_P(BME_data *input){
	// Calculates the float value of the compensated pressure
	
	input->Pf = (float)input->P / 256.0f;
}

void BME_compensate_H(BME_data *input){
	// Returns humidity as unsigned 32 bit integer
	// Output value of '47445' represents 47445 / 1024 = 46.33 %RH

	int32_t h;

	h = (input->t_fine - ((int32_t)76800));
	h = (((((input->adc_H << 14) - (((int32_t)input->dig_H[3]) << 20) - (((int32_t)input->dig_H[4]) * h)) + ((int32_t)16384)) >> 15) * (((((((h * ((int32_t)input->dig_H[5])) >> 10) * (((h * ((int32_t)input->dig_H[2])) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)input->dig_H[1]) + 8192) >> 14));
	h = (h - (((((h >> 15) * (h >> 15)) >> 7) * ((int32_t)input->dig_H[0])) >> 4));
	h = (h < 0 ? 0 : h);
	h = (h > 419430400 ? 419430400 : h); // (test) ? (if true) : (otherwise)    ---> make sure result will be between 0 - 100 %RH

	input->H = (uint32_t)(h >> 12);
}

void BME_float_H(BME_data *input){
	// Calculates the float value of the compensated humidity
	
	input->Hf = (float)input->H / 1024.0f;
}
