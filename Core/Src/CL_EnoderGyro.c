/*
 * CL_EnoderGyro.c
 *
 *  Created on: Oct 15, 2023
 *      Author: sf199
 */


#include "CL_EnoderGyro.h"
#include "PL_gyro.h"
#include "PL_encoder.h"
#include "PL_timer.h"
#include "math.h"
#include "matrix_calculation.h"


float yaw_angle,angle_speed;
float anglex,angle_speedx,angle_speedx_set;
float gf_speed,gf_distance,gf_accel;
float omegaX_offset,omegaZ_offset,accelY_offset;


float encoder_R0,encoder_L0;

float E_distanceL,E_distanceR;
float E_speedL,E_speedR;

float E_lpf_distanceL,E_lpf_distanceR;
float E_lpf_speedL,E_lpf_speedR;

float G_hpf_distanceL,G_hpf_distanceR;
float G_hpf_speedL,G_hpf_speedR;

// 相補
float fusion_distanceL,fusion_distanceR;
float fusion_speedL,fusion_speedR;
float straight_alpha;

float theta_comp_gain;//角度伝達誤差の補償

// カルマンフィルタ系の変数
float kal_A_1[2][2] = {
{1,INTERRUPT_TIME},
{0,1}
};
//"B" of the state equation (update freq = 100 Hz)
float kal_B_1[2][1] = {
{1/2*INTERRUPT_TIME*INTERRUPT_TIME},
{INTERRUPT_TIME}
};
//"C" of the state equation (update freq = 100 Hz)
float kal_C_1[2][2] = {
{0, 0},
{0, 1}
};
float kal_I2[2][2] = {
{1,0},
{0,1}
};

float kal_W_1[2][2];
float kal_U_1;

float kal_x_1[2][1];
float kal_x_1_predict[2][1];
//Covariance matrix
float kal_P_1_predict[2][2];
float kal_P_1[2][2];
float K_G_1[2][2];

float kalman_speed,kalman_distance,kalman_distance2;

float g_omegaZ_mean,g_accelY_mean;
float g_encY_variance,g_accelY_variance;


void init_EncoderGyro(void){
	straight_alpha=0.65;
	theta_comp_gain=1;
	reset_Kalman();
	reset_gyro();
	reset_distance();
	reset_speed();
}



void reset_Kalman(void){

	//reset_EncoderGyro_MeanVariance();

    //initial value of x_data_predict
    for(int i=0; i<2; i++)
    {
    	kal_x_1_predict[i][0] = 0;
    }
    //initial value of P_x_predict
    for(int i=0; i<2; i++)
    {
        for(int j=0; j<2; j++)
        {
        	kal_P_1_predict[i][j] = 0;
        }
    }
    for(int i=0; i<2; i++)
    {
    	kal_P_1_predict[i][i] = 1e-4;
    }
}

void reset_gyro(void){
	reset_EncoderGyro_MeanVariance();
	reset_gyro_integral();
}




void reset_EncoderGyro_MeanVariance(void){
	int sample_num=1000;
    //get data
    float omegaZ_array[sample_num];
    float omegaZ_mean;
    //float omegaZ_variance;
    float omegaX_array[sample_num];
    float omegaX_mean;
    //float omegaX_variance;
    float accelY_array[sample_num];
    float accelY_mean;
    float accelY_variance;
    float encY_array[sample_num];
    float encY_mean;
    float encY_variance;
    for(int i=0;i<sample_num;i++)
    {
    	omegaZ_array[i] = gyro.omega_z*GYRO_COEFFICIENT;
    	omegaX_array[i] = gyro.omega_x;
    	accelY_array[i] = gyro.accel_y*ACCEL_COEFFICIENT;
    	encY_array[i] = (E_speedL+E_speedR)/2;
    	wait_ms_NoReset(1);
    }

    //calculate mean
    omegaZ_mean = 0;
    omegaX_mean = 0;
    accelY_mean = 0;
    encY_mean = 0;
    for(int i=0;i<sample_num;i++)
    {
    	omegaZ_mean += omegaZ_array[i];
    	omegaX_mean += omegaX_array[i];
    	accelY_mean += accelY_array[i];
    	encY_mean += encY_array[i];
    }
    omegaZ_mean /= sample_num;
    omegaX_mean /= sample_num;
    accelY_mean /= sample_num;
    encY_mean /= sample_num;

    //calculate variance
    float temp_a,temp_e;
    accelY_variance = 0;
    encY_variance = 0;
    for(int i=0; i<sample_num; i++)
    {
        temp_a = accelY_array[i] - accelY_mean;
        accelY_variance += temp_a*temp_a;

        temp_e = encY_array[i] - encY_mean;
        encY_variance += temp_e*temp_e;
    }
    accelY_variance /= sample_num;
    encY_variance /= sample_num;
	
	g_omegaZ_mean = omegaZ_mean;
	g_accelY_mean = accelY_mean;
	g_encY_variance=encY_variance;
	g_accelY_variance=accelY_variance;

	encY_variance = ENCY_VAR;
	//accelY_mean = ACCY_MEAN;
	accelY_variance = ACCY_VAR;


    //measurement noise matrix
    for(int i=0; i<2; i++)
    {
        for(int j=0; j<2; j++)
        {
        	kal_W_1[i][j] = 0;
        }
    }
    kal_W_1[0][0] = encY_variance*INTERRUPT_TIME;
    kal_W_1[1][1] = encY_variance;

	kal_U_1=accelY_variance;

    // offset代入
	omegaZ_offset = omegaZ_mean;
	omegaX_offset = omegaX_mean;
	accelY_offset = accelY_mean;
}



void reset_gyro_integral(void){

	yaw_angle = 0;
	anglex = 0;
	gf_speed = 0;
	gf_distance = 0;

}


void reset_distance(void){
	E_distanceL = 0;
	E_distanceR = 0;
	E_lpf_distanceL = 0;
	E_lpf_distanceR = 0;
	G_hpf_distanceL = 0;
	G_hpf_distanceR = 0;
	fusion_distanceL=0;
	fusion_distanceR=0;
	kalman_distance=0;
	kalman_distance2=0;

    	kal_x_1_predict[0][0] = 0;


}

void reset_speed(void){
	G_hpf_speedL=0;
	G_hpf_speedR=0;
	E_lpf_speedL=0;
	E_lpf_speedR=0;
	fusion_speedL=0;
	fusion_speedR=0;

}








void interupt_calEncoder(void) {
	float angle_R,angle_L;
	angle_R=encoder_R-encoder_R0;
	if(angle_R>180){angle_R=angle_R-360;}
	if(angle_R<-180){angle_R=angle_R+360;}
	angle_L=-(encoder_L-encoder_L0);
	if(angle_L>180){angle_L=angle_L-360;}
	if(angle_L<-180){angle_L=angle_L+360;}
/*
	E_speedL = (angle_L) * pi / 180 * TIRE_DIAMETER /2 * 1000  / INTERRUPT_TIME;
	E_speedR = (angle_R) * pi / 180 * TIRE_DIAMETER /2 * 1000 / INTERRUPT_TIME;
*/
	E_speedL = (angle_L) * pi / 180 * TIRE_DIAMETER /2 * 1000  / INTERRUPT_TIME*THETA_COMP_L0
			/(THETA_COMP_L0 + theta_comp_gain*(THETA_COMP_L1*sinf(encoder_L*pi/180+THETA_COMP_L2)
	+THETA_COMP_L3*sinf(2*encoder_L*pi/180+THETA_COMP_L4)+THETA_COMP_L5*sinf(3*encoder_L*pi/180+THETA_COMP_L6)));
	E_speedR = (angle_R) * pi / 180 * TIRE_DIAMETER /2 * 1000 / INTERRUPT_TIME*THETA_COMP_R0
			/ (THETA_COMP_R0 + theta_comp_gain*(THETA_COMP_R1*sinf(encoder_R*pi/180+THETA_COMP_R2)
	+THETA_COMP_R3*sinf(2*encoder_R*pi/180+THETA_COMP_R4)+THETA_COMP_R5*sinf(3*encoder_R*pi/180+THETA_COMP_R6)));



	E_distanceL += E_speedL * INTERRUPT_TIME;
	E_distanceR += E_speedR * INTERRUPT_TIME;

	encoder_L0=encoder_L;
	encoder_R0=encoder_R;

}


void interupt_calFusion(void) {


	E_lpf_speedL = straight_alpha * E_lpf_speedL + (1 - straight_alpha) * E_speedL;
	E_lpf_speedR = straight_alpha * E_lpf_speedR + (1 - straight_alpha) * E_speedR;
	E_lpf_distanceL += E_lpf_speedL * INTERRUPT_TIME;
	E_lpf_distanceR += E_lpf_speedR * INTERRUPT_TIME;


	//G_hpf_speedL = straight_alpha * (G_hpf_speedL + INTERRUPT_TIME * gf_accel);
	//G_hpf_speedR = straight_alpha * (G_hpf_speedL + INTERRUPT_TIME * gf_accel);
	//G_hpf_distanceL += G_hpf_speedL * INTERRUPT_TIME;
	//G_hpf_distanceR += G_hpf_speedR * INTERRUPT_TIME;


	fusion_speedL = straight_alpha * (fusion_speedL + INTERRUPT_TIME * gf_accel) + (1 - straight_alpha) * E_speedL;
	fusion_speedR = straight_alpha * (fusion_speedR + INTERRUPT_TIME * gf_accel) + (1 - straight_alpha) * E_speedR;
	fusion_distanceL += fusion_speedL * INTERRUPT_TIME;
	fusion_distanceR += fusion_speedR * INTERRUPT_TIME;

}





void interupt_calKalman(void) {
	float kal_data_in[2][1];
	float kal_data_out[2][1];
	float delta_kal_data[2][1];
	float kal_C_1_x[2][1];
	float tran_kal_C_1[2][2];
	float tran_kal_B_1[1][2];
	float kal_B_1_Vin[2][1];
	float kal_A_1_x[2][1];
	float tran_kal_A_1[2][2];
	float P_CT[2][2];
    float G_temp1[2][2];
    float G_temp2[2][2];
    float G_temp2_inv[2][2];
    float delta_x[2][1];
    float GC[2][2];
    float I2_GC[2][2];
    float AP[2][2];
    float APAT[2][2];
    float BBT[2][2];
    float BUBT[2][2];
	//---------------------------------------
	//Kalman Filter (all system)
	//---------------------------------------
	//measurement data
	        kal_data_in[0][0] = (E_distanceL+E_distanceR)/2;
	        kal_data_in[1][0] = (E_speedL+E_speedR)/2;


	        //calculate Kalman gain: G = P'C^T(W+CP'C^T)^-1
	        mat_tran(kal_C_1[0], tran_kal_C_1[0], 2, 2);//C^T
	        mat_mul(kal_P_1_predict[0], tran_kal_C_1[0], P_CT[0], 2, 2, 2, 2);//P'C^T
	        mat_mul(kal_C_1[0], P_CT[0], G_temp1[0], 2, 2, 2, 2);//CPC^T
	        mat_add(G_temp1[0], kal_W_1[0], G_temp2[0], 2, 2);//W+CP'C^T
	        mat_inv(G_temp2[0], G_temp2_inv[0], 2, 2);//(W+CP'C^T)^-1
	        mat_mul(P_CT[0], G_temp2_inv[0], K_G_1[0], 2, 2, 2, 2); //P'C^T(W+CP'C^T)^-1

	        //x_data estimation: x = x'+G(y-Cx')
	        mat_mul(kal_C_1[0], kal_x_1_predict[0], kal_C_1_x[0], 2, 2, 2, 1);//Cx'
	        mat_sub(kal_data_in[0], kal_C_1_x[0], delta_kal_data[0], 2, 1);//y-Cx'
	        mat_mul(K_G_1[0], delta_kal_data[0], delta_x[0], 2, 2, 2, 1);//G(y-Cx')
	        mat_add(kal_x_1_predict[0], delta_x[0], kal_data_out[0], 2, 1);//x'+G(y-Cx')

	        //calculate covariance matrix: P=(I-GC)P'
	        mat_mul(K_G_1[0], kal_C_1[0], GC[0], 2, 2, 2, 2);//GC
	        mat_sub(kal_I2[0], GC[0], I2_GC[0], 2, 2);//I-GC
	        mat_mul(I2_GC[0], kal_P_1_predict[0], kal_P_1[0], 2, 2, 2, 2);//(I-GC)P'

	        //predict the next step data: x'=Ax+Bu
	        mat_mul(kal_A_1[0], kal_data_out[0], kal_A_1_x[0], 2, 2, 2, 1);//Ax_hat
	        mat_mul_const(kal_B_1[0], gf_accel , kal_B_1_Vin[0], 2, 1);//Bu
	        mat_add(kal_A_1_x[0], kal_B_1_Vin[0], kal_x_1_predict[0], 2, 1);//Ax+Bu

	        //predict covariance matrix: P'=APA^T + BUB^T
	        mat_tran(kal_A_1[0], tran_kal_A_1[0], 2, 2);//A^T
	        mat_mul(kal_A_1[0], kal_P_1[0], AP[0], 2, 2, 2, 2);//AP
	        mat_mul(AP[0], tran_kal_A_1[0], APAT[0], 2, 2, 2, 2);//APA^T
	        mat_tran(kal_B_1[0], tran_kal_B_1[0], 2, 1);//B^T
	        mat_mul(kal_B_1[0], tran_kal_B_1[0], BBT[0], 2, 1, 1, 2);//BB^T
	        mat_mul_const(BBT[0], kal_U_1, BUBT[0], 2, 2);//BUB^T
	        mat_add(APAT[0], BUBT[0], kal_P_1_predict[0], 2, 2);//APA^T+BUB^T

	        kalman_speed= *kal_data_out[1];
	        kalman_distance2 += kalman_speed * INTERRUPT_TIME;//いつかカルマンに組み込む
	        kalman_distance = *kal_data_out[0];
}






void interrupt_calGyro(void) {
// 生値の使用
	angle_speedx=-gyro.omega_x;

// オフセット差分
	angle_speed = gyro.omega_z*GYRO_COEFFICIENT - omegaZ_offset;
	angle_speedx_set=-(gyro.omega_x - omegaX_offset);
	gf_accel = gyro.accel_y*ACCEL_COEFFICIENT - accelY_offset;

// 積分値
	yaw_angle += angle_speed * INTERRUPT_TIME; //deg
	anglex += angle_speedx_set * INTERRUPT_TIME; //deg
	gf_speed += gf_accel * INTERRUPT_TIME;
	gf_distance += gf_speed * INTERRUPT_TIME;




}
