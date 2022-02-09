#ifndef BME_280_H
#define BME_280_H

#include "main.h"

/* --------
	
	Defines
	
-------- */

// Temperature defines
#define BME_T1_1						0x88
#define	BME_T1_2  						0x89
#define	BME_T2_1  						0x8A
#define	BME_T2_2  						0x8B
#define	BME_T3_1  						0x8C
#define	BME_T3_2  						0x8D

// Air pressure defines
#define	BME_P1_1  						0x8E
#define	BME_P1_2  						0x8F
#define	BME_P2_1  						0x90
#define	BME_P2_2  						0x91
#define	BME_P3_1  						0x92
#define	BME_P3_2  						0x93
#define	BME_P4_1  						0x94
#define	BME_P4_2  						0x95
#define	BME_P5_1  						0x96
#define	BME_P5_2  						0x97
#define	BME_P6_1  						0x98
#define	BME_P6_2  						0x99
#define	BME_P7_1  						0x9A
#define	BME_P7_2  						0x9B
#define	BME_P8_1  						0x9C
#define	BME_P8_2  						0x9D
#define	BME_P9_1  						0x9E
#define	BME_P9_2  						0x9F

// Humidity defines
#define	BME_H1  						0xA1
#define	BME_H2_1  						0xE1
#define	BME_H2_2  						0xE2
#define	BME_H3  						0xE3
#define	BME_H4_1  						0xE4
#define	BME_H4_2  						0xE5
#define	BME_H5_1  						0xE5
#define	BME_H5_2  						0xE6
#define	BME_H6  						0xE7

// Data regs
#define BME_P_addr						0xF7
#define BME_P_len						3
#define BME_T_addr						0xFA
#define BME_T_len						3
#define BME_H_addr						0xFD
#define BME_H_len						2

// Misc defines
#define	BME_chip_id  					0xD0
#define	BME_ctrl_hum  					0xF2
#define	BME_ctrl_meas  					0xF4
#define BME_ctrl_trig					0x6D
#define	BME_config  					0xF5
#define	BME_reset  						0xE0
#define BME_ADDR_W 						0x76 << 1
#define BME_ADDR_R						(0x76 << 1) | 1
// #define BME_table_length				100
// #define BME_PSAT_H					102500
// #define BME_PSAT_L					80000
// #define BME_LUT_Inc					227.27f		// (BME_PSAT_H-BME_PSAT_L)/(BME_table_length)
// #define BME_P_cntr_max				50
// #define BME_T_cntr_max				2500

// Physics defines
// #define BME_PB						101325
// #define BME_TB						288.15f
// #define BME_LB						0.0065f
// #define BME_R						8.31432f
// #define BME_G						9.81584f
// #define BME_M						0.0289644f



/* ----------------
*
*	Typedefs
*
 ----------------- */
 
typedef struct BME_data{
		// Conditioning
	uint16_t dig_T[3];
	uint16_t dig_P[9];
	uint16_t dig_H[6];
	uint8_t id;
		// raw measurements
	int32_t t_fine;
	int32_t adc_T;
	int32_t buf_T[8];
	uint8_t index_T;
	int32_t adc_P;
	int32_t buf_P[2];
	uint8_t index_P;
	int32_t adc_H;
		// conditioned measurements
	int32_t T;
	uint32_t P;
	uint32_t H;
		// calculated measurements
	float Tf;
	float Pf;
	float Hf;
	float altitude;
		// Interface
	I2C_HandleTypeDef *i2c_Handle;
		// Control
	uint16_t read_cntr_P;
	uint8_t read_P;
	uint16_t read_cntr_T;
	uint8_t read_T;
	int64_t Pvar1;
	int64_t Pvar2;
	uint8_t reading_data;
	uint8_t trig_flag;
	uint8_t trig[2];
		// raw data
	uint8_t raw[8];
} BME_data;
	

/* ----------------
*
*	Function prototypes
*
 ----------------- */
 
void BME_280_mem_map_init(BME_data *input, I2C_HandleTypeDef *hi2c);
uint8_t BME_280_Config_ReadOut(BME_data *input);
uint8_t BME_280_InitWrite(BME_data *input);
uint8_t BME_TriggerConversion_DMA(BME_data *input);
uint8_t BME_SendAddr_DMA(BME_data *input);
uint8_t BME_ReadData_DMA(BME_data *input);
uint8_t BME_ProcMeas(BME_data *input);
void BME_AssembleT(BME_data *input);
void BME_AssembleP(BME_data *input);
void Calc_Barometric_LUT( BME_data *input, const float *BME_altitude_LUT, const float *BME_alt_grad_LUT );
void BME_compensate_T(BME_data *input);
void BME_float_T(BME_data *input);
void BME_compensate_P_precalc(BME_data *input);
void BME_compensate_P_fast(BME_data *input);
void BME_compensate_P(BME_data *input);
void BME_float_P(BME_data *input);
void BME_compensate_H(BME_data *input);
void BME_float_H(BME_data *input);


#endif
