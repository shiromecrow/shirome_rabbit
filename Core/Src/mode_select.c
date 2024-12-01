/*
 * mode_select.c
 *
 *  Created on: Jan 11, 2023
 *      Author: sf199
 */



#include "mode_select.h"

#include "stdio.h"

#include "PL_timer.h"
#include "PL_LED.h"
#include "PL_sensor.h"
#include "PL_encoder.h"
#include "PL_gyro.h"
#include "PL_motor.h"
#include "PL_flash.h"

#include "CL_EnoderGyro.h"
#include "CL_sensor.h"
#include "Control_motor.h"
#include "PID_wall.h"
#include "PID_EncoderGyro.h"

#include "maze_Turning.h"
#include "turning_parameter.h"
#include "record.h"

//#include "wait_ms.h"
#include "define.h"
#include "math.h"
#include "fail_safe.h"
#include "maze_strategy.h"
#include "maze_wall.h"

uint16_t main_mode;



unsigned char mode_decision(unsigned char now_mode){

unsigned char main_modeR=0;
unsigned char main_modeL=0;
unsigned char main_mode=0;
float dis8 =10;
main_modeR=now_mode & 0x0F;
main_modeL=now_mode >> 4;

	pl_yellow_LED_count(main_modeL * 16 + main_modeR);
			//pl_play_oneSound(main_modeR);
			//モード選択*************************************************************************
			while (angle_speedx <= 400) {
				HAL_Delay(1);
				if (E_distanceR >= dis8) {
					if (main_modeR == 15) {
						main_modeR = 0;
					} else {
						main_modeR++;
					}
					reset_distance();

					pl_yellow_LED_count(main_modeL * 16 + main_modeR);
				}
				if (E_distanceR <= -dis8) {
					if (main_modeR == 0) {
						main_modeR = 15;
					} else {
						main_modeR = main_modeR - 1;
					}
					reset_distance();
					pl_yellow_LED_count(main_modeL * 16 + main_modeR);
				}
				if (E_distanceL >= dis8) {
					if (main_modeL == 15) {
						main_modeL = 0;
					} else {
						main_modeL = main_modeL + 1;
					}
					reset_distance();
					pl_yellow_LED_count(main_modeL * 16 + main_modeR);
				}
				if (E_distanceL <= -dis8) {
					if (main_modeL == 0) {
						main_modeL = 15;
					} else {
						main_modeL = main_modeL - 1;
					}
					reset_distance();
					pl_yellow_LED_count(main_modeL * 16 + main_modeR);
				}
				//main_mode=main_modeL << 4 | main_modeR;
	//printf("main_mode=%x L=%x R=%x\n",main_mode,main_modeL,main_modeR);

			}
			pl_yellow_LED_off();
			pl_r_blue_LED(ON);
			pl_l_blue_LED(ON);
			HAL_Delay(1000);
			while (g_sensor[0][0] <= SENSOR_FINGER_0 || g_sensor[2][0] <= SENSOR_FINGER_2 || g_sensor[3][0] <= SENSOR_FINGER_3 || g_sensor[5][0] <= SENSOR_FINGER_5) {	//

				if (angle_speedx >= 400) {
					main_modeL = 0;
					main_modeR = 0;
					break;
				}
				HAL_Delay(1);
			}
			pl_r_blue_LED(OFF);
			pl_l_blue_LED(OFF);
			pl_yellow_LED_off();

			main_mode=main_modeL << 4 | main_modeR;

return main_mode;
}


void mode_execution(unsigned char now_mode){

unsigned char main_modeR=0;
unsigned char main_modeL=0;

main_modeR=now_mode & 0x0F;
main_modeL=now_mode >> 4;


reset_Kalman();
reset_gyro_integral();
reset_distance();
reset_speed();
clear_Ierror();

switch (main_modeL) {
	case 0b0000://PLテストモード1
		mode_PLtest(main_modeR);
	break;
	case 0b0001://実行モード1
		mode_Running(main_modeR);
	break;
	case 0b0010://実行モード2
		mode_Running2(main_modeR);
	break;
	case 0b0011:
		mode_Tuning0(main_modeR);
	break;
	case 0b0100:
		mode_WallSensorTuning(main_modeR);
	break;
	case 0b0101://試験モード
		testturning(speed600_shortest_mollifier,main_modeR,1,OFF,3.3,1);
	break;
	case 0b0110:
		mode_WallSensorTuning_fast(main_modeR);
	break;
	case 0b0111:
		testturning(speed1000_shortest_mollifier,main_modeR,1,ON,4.0,1);
	break;
	case 0b1000:
		testturning(speed1200_shortest_mollifier,main_modeR,1,ON,5.86,1);
	break;
	case 0b1001:
		testturning(speed1400_shortest_mollifier,main_modeR,1,ON,5.86,1);
	break;
	case 0b1010:
	break;
	case 0b1011:
	break;
	case 0b1100:
	break;
	case 0b1101:
	break;
	case 0b1110:
		mode_Tuning2(main_modeR);
	break;
	case 0b1111:
		mode_Tuning1(main_modeR);
	break;
}
}



void mode_PLtest(unsigned char main_modeR) {
	wait_ms(2000);
	int duty_L=0, duty_R=0;
	switch (main_modeR) {
	case 0b0000:
		//飛ばすerror
		break;
	case 0b0001:	//sensor
		while (1) {
			printf("SEN1=%d,SEN2=%d,SEN3=%d,SEN4=%d,SEN5=%d,SEN6=%d\n", g_sensor[0][0],
					g_sensor[1][0], g_sensor[2][0], g_sensor[3][0],
					g_sensor[4][0],g_sensor[5][0]);
			wait_ms(500);
		}
		break;
	case 0b0010:	//encoder
		pl_r_blue_LED(ON);
		pl_l_blue_LED(ON);
		record_mode = 1;
		pl_DriveMotor_duty(900, 900);
		pl_R_DriveMotor_mode(MOTOR_FRONT);
		pl_L_DriveMotor_mode(MOTOR_FRONT);
		pl_DriveMotor_start();
		wait_ms(2500);
		pl_DriveMotor_stop();
		pl_r_blue_LED(OFF);
		pl_l_blue_LED(OFF);
		record_mode = 0;
		while (g_sensor[0][0] <= SENSOR_FINGER_0
				|| g_sensor[2][0] <= SENSOR_FINGER_2
				|| g_sensor[4][0] <= SENSOR_FINGER_4) {
			wait_ms(1);
		}
		record_print();
		break;
	case 0b0011:	//gyro
		reset_gyro();
		reset_speed();
		pl_r_blue_LED(ON);
		pl_l_blue_LED(ON);
		record_mode = 2;
		wait_ms(2500);
		pl_r_blue_LED(OFF);
		pl_l_blue_LED(OFF);
		record_mode = 0;
		while (g_sensor[0][0] <= SENSOR_FINGER_0
				|| g_sensor[2][0] <= SENSOR_FINGER_2
				|| g_sensor[4][0] <= SENSOR_FINGER_4) {
			wait_ms(1);
		}
		record_print();
		break;
	case 0b0100://speaker
		while (1) {
		  	printf("gyro x : %5.5f, y : %5.5f,z : %5.5f, accel x : %5.5f, y :%5.5f, z : %5.5f\r",
				  			gyro.omega_x, gyro.omega_y, gyro.omega_z, gyro.accel_x,gyro.accel_y, gyro.accel_z);
			wait_ms(500);
		}
	break;
	case 0b0101:
		test_flash();
		break;
	case 0b0110:
		record_mode=6;
		pl_r_blue_LED(OFF);
		pl_l_blue_LED(OFF);
		//get_duty(1.0, -1.0,&duty_L,&duty_R);
		//pl_DriveMotor_duty(duty_L,duty_R);
		pl_DriveMotor_duty(400,400);
		pl_DriveMotor_start();
		while (g_sensor[0][0] <= SENSOR_FINGER_0 || g_sensor[2][0] <= SENSOR_FINGER_2 || g_sensor[4][0] <= SENSOR_FINGER_4) {
			wait_ms(1);
			if(record_rupe_flag==1){
				record_mode=0;
				break;
			}
		}
		pl_DriveMotor_stop();
		pl_r_blue_LED(OFF);
		pl_l_blue_LED(OFF);

		break;
	case 0b0111:
			control_fun(5.86);
			pl_FunMotor_start();
			HAL_Delay(600);
			reset_gyro();
			reset_speed();
			clear_Ierror();
			pl_FunMotor_stop();
			pl_r_blue_LED(ON);
			pl_l_blue_LED(ON);
			while (g_sensor[0][0] <= SENSOR_FINGER_0 || g_sensor[2][0] <= SENSOR_FINGER_2 || g_sensor[4][0] <= SENSOR_FINGER_4) {
				HAL_Delay(1);
			}
			pl_r_blue_LED(OFF);
			pl_l_blue_LED(OFF);
			printf("encoder  var : %f\n",g_encY_variance);
			printf("ACC   mean : %f, var : %f\n",g_accelY_mean, g_accelY_variance);
			printf("GYRO   mean : %f\n",g_omegaZ_mean);
	break;
	case 0b1001://tim6
		g_wallCut_mode = 1;
		g_WallControl_mode = 1;
		highspeed_mode = 1;
		modeacc = 6;
		no_safty = 1;
		record_mode=1;
		while (1) {
		  	printf("count=%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
		  			tim6_log1,tim6_log2,tim6_log3,tim6_log4,tim6_log5,tim6_log6,
					tim6_log7,tim6_log8,tim6_log9,tim6_log10,tim6_log11,tim6_log12);
			wait_ms(500);
		}

		break;
	case 0b1010://fun
		    control_fun(5.86);
			pl_FunMotor_start();
			HAL_Delay(2000);
			reset_gyro();
			reset_speed();
			clear_Ierror();
			pl_r_blue_LED(ON);
			pl_l_blue_LED(ON);
			while (g_sensor[0][0] <= SENSOR_FINGER_0 || g_sensor[2][0] <= SENSOR_FINGER_2 || g_sensor[4][0] <= SENSOR_FINGER_4) {
				HAL_Delay(1);
			}
			pl_r_blue_LED(OFF);
			pl_l_blue_LED(OFF);
			pl_FunMotor_stop();
		break;
	case 0b1011://fun
			reset_gyro();
		    control_fun(5.86);
			pl_FunMotor_start();
			HAL_Delay(2000);
			reset_speed();
			clear_Ierror();
			record_mode=2;
			pl_r_blue_LED(ON);
			pl_l_blue_LED(ON);
			no_angle();
			pl_r_blue_LED(OFF);
			pl_l_blue_LED(OFF);
			pl_FunMotor_stop();
		break;
	case 0b1100:
			pl_r_blue_LED(ON);
			pl_l_blue_LED(ON);
			get_duty(1, 1,&duty_L,&duty_R);
			pl_DriveMotor_duty(duty_L,duty_R);
			pl_DriveMotor_start();
			record_mode=11;
			wait_ms(2000);
			record_mode=0;
			wait_ms(1);
			pl_DriveMotor_stop();
			pl_r_blue_LED(OFF);
			pl_l_blue_LED(OFF);
		break;
	case 0b1111:	//record_out
		record_print();
		break;

//		case 0b0100://speaker
//			speaker_mode = 1;
//			while (g_sensor[0][0] <= SENSOR_FINGER_0 || g_sensor[2][0] <= SENSOR_FINGER_2 || g_sensor[4][0] <= SENSOR_FINGER_4) {
//				wait_ms(1);
//							if (g_sensor[2][0] > SENSOR_FINGER_2){
//								speaker_mode++;
//								wait_ms(1000);
//							}
//
//						}
//			speaker_mode = 101;
//		break;
//		case 0b0101://gyro
//			reset_gyro();
//			reset_speed();
//			pl_r_blue_LED(ON);
//			pl_l_blue_LED(ON);
//			record_mode = 2;
//			wait_ms(2000);
//			pl_r_blue_LED(OFF);
//			pl_l_blue_LED(OFF);
//			record_mode = 0;
//			while (g_sensor[0][0] <= SENSOR_FINGER_0 || g_sensor[2][0] <= SENSOR_FINGER_2 || g_sensor[4][0] <= SENSOR_FINGER_4) {
//				wait_ms(1);
//			}
//				record_print();
//		break;
//		case 0b0110://gyro
//			pl_FunMotor_duty(0.35);
//			pl_FunMotor_start();
//			wait_ms(1000);
//			reset_gyro();
//			reset_speed();
//			record_mode = 1;
//			wait_ms(2000);
//			record_mode = 0;
//			pl_FunMotor_stop();
//			while (g_sensor[0][0] <= SENSOR_FINGER_0 || g_sensor[2][0] <= SENSOR_FINGER_2 || g_sensor[4][0] <= SENSOR_FINGER_4) {
//				wait_ms(1);
//			}
//				record_print();
//
//		break;
//		case 0b0111://宴会芸(別のほうがいいかも)
//			//pl_FunMotor_duty(150);
//			//pl_FunMotor_start();
//			HAL_Delay(1000);
//			reset_gyro();
//			reset_speed();
//			pl_r_blue_LED(ON);
//			pl_l_blue_LED(ON);
//			pl_DriveMotor_standby(ON);
//			clear_Ierror();
//			no_angle();
//			pl_R_DriveMotor_mode(MOTOR_STOP);
//			pl_L_DriveMotor_mode(MOTOR_STOP);
//			pl_DriveMotor_standby(OFF);
//			pl_r_blue_LED(OFF);
//			pl_l_blue_LED(OFF);
//			//pl_FunMotor_stop();
//		break;
//		case 0b1000://printfalldata
//			while (1) {
//				//	printf("SENSOR_ON  SEN1=%d,SEN2=%d,SEN3=%d,SEN4=%d,SEN5=%d\n",
//				//	g_sensor_on[0], g_sensor_on[1], g_sensor_on[2], g_sensor_on[3], g_sensor_on[4]);
////				printf("SEN1=%d,SEN2=%d,SEN3=%d,SEN4=%d,SEN5=%d\n", g_sensor[0][0],
////						g_sensor[1][0], g_sensor[2][0], g_sensor[3][0], g_sensor[4][0]);
//				HAL_Delay(500);
//				//	printf("SENSOR_OFF  SEN1=%d,SEN2=%d,SEN3=%d,SEN4=%d,SEN5=%d\n",
//				//					g_sensor_off[0], g_sensor_off[1], g_sensor_off[2], g_sensor_off[3],g_sensor_off[4]);
////				printf("R  dis=%f,speed=%f,   L  dis=%f,speed=%f\n",
////						E_distanceR, E_speedR, E_distanceL, E_speedL);
//
//				printf(
//						"gyro x : %5.5f, y : %5.5f,z : %5.5f, accel x : %5.5f, y :%5.5f, z : %5.5f\r",
//						gyro.omega_x, gyro.omega_y, gyro.omega_z, gyro.accel_x,
//						gyro.accel_y, gyro.accel_z);
//				//pl_yellow_LED_on();
//				//printf("R  dis=%f,speed=%f,   L  dis=%f,speed=%f\n",E_distanceR,E_speedR,E_distanceL,E_speedL);
//				//printf("angle=%f,angle_speed=%f\n", angle, angle_speed);
//			}
//		break;

//		case 0b1010://センサーの線形化
//			sensor_line();
//		break;
//		case 0b1011://バッテリー
//			while (1) {
//				printf("BATT=%f,BATT_Mena=%f,%f\n", g_V_battery[0],g_V_battery_mean,g_V_batt);
//				wait_ms(500);
//			}
//		break;
//		case 0b1100:
//			sensor_line();
//		break;
//		case 0b1101:
//			sensor_line45();
//		break;
//		case 0b1110:
//			sensor_line45_slant();
//		break;
//		case 0b1111:
//			record_print();
//		break;
	}
//

}


void mode_Running(unsigned char main_modeR){

	pl_L_DriveMotor_mode(MOTOR_BREAK);
	pl_R_DriveMotor_mode(MOTOR_BREAK);
	reset_gyro();
	reset_speed();
	reset_distance();
	clear_Ierror();

	switch (main_modeR) {
		case 0b0000://迷路表示
			//maze_maker2(1, 0, 0, 0, 7, 7);
			record_out();
			//wall.row[8]=(1<<8);
		//	maze_clear();
			maze_out_matlab();

			create_StepCountMap_queue();
			maze_display();

			create_DijkstraMap();
			maze_display_Dijkstra();

			route_Dijkstra();
			create_StepCountMap_unknown();
			maze_display();

			tic_timer();
			for(int j=0;j<1000;j++){
			create_StepCountMap_queue();
			}
			float tim2 = toc_timer();
			maze_display();
			printf("tim2=%f\n", tim2);
			tic_timer();
			create_DijkstraMap();
			for(int j=0;j<100;j++){
			//create_StepCountMap();
			//maze_clear();
			//create_DijkstraMap();
			route_Dijkstra();
			create_StepCountMap_unknown();

			}
			float tim1 = toc_timer();
			maze_display();
			maze_display_Dijkstra();
			printf("tim1=%f,tim2=%f\n", tim1, tim2);
		break;
		case 0b0001://足立法(遅い)
			record_out();
			create_StepCountMap_queue();
			maze_display();
		break;
		case 0b0010://Flashから
			//maze_clear();
			//record_in();
			record_out();
			//create_StepCountMap_queue();
			//maze_display();
		break;
		case 0b0011://ゴミ
			tic_timer();
			AdatiWayReturn(300,700,4000,8000,speed300_exploration,1,1);
		break;
		case 0b0100://ゴミ
			record_out();
			run_shortest(1000,3000,3000,TURN_OFF,FUN_OFF,SLANT_OFF,speed300_shortest,2.4,0,0);
		break;
		case 0b0101://
			record_out();
			run_shortest(1000,3000,3000,TURN_ON,FUN_OFF,SLANT_OFF,speed600_shortest_mollifier,5.86,1,0);
		break;
		case 0b0110://吸引なしで斜め走行
			record_out();
			run_shortest(1000,3000,3000,TURN_ON,FUN_OFF,SLANT_ON,speed600_shortest_mollifier,5.86,1,0);
		break;
		case 0b0111://吸引ありで斜め走行
			record_out();
			run_shortest(4000,10000,10000,TURN_ON,FUN_ON,SLANT_ON,speed1000_shortest_mollifier,4.0,1,0);
		break;
		case 0b1000://
			record_out();
			run_shortest(4000,18000,18000,TURN_ON,FUN_ON,SLANT_ON,speed1000_shortest_mollifier,4.0,1,0);
		break;
		case 0b1001:
			record_out();
			run_shortest(4000,18000,18000,TURN_ON,FUN_ON,SLANT_ON,speed1400_shortest_mollifier,5.86,1,0);
		break;
		case 0b1010:
			record_out();
			run_shortest(4000,22000,22000,TURN_ON,FUN_ON,SLANT_ON,speed1400_shortest_mollifier,5.86,1,0);
		break;
		case 0b1011:
			record_out();
			run_shortest(5000,25000,25000,TURN_ON,FUN_ON,SLANT_ON,speed1400_shortest_mollifier,5.86,1,0);
		break;
		case 0b1100:
			record_out();
			run_shortest(3000,12000,12000,TURN_ON,FUN_ON,SLANT_ON,speed1400_shortest_mollifier,5.86,1,0);
		break;
		case 0b1101:
			record_out();
			run_shortest(4000,13000,14000,TURN_ON,FUN_ON,SLANT_ON,speed1200_shortest_mollifier,5.86,1,2);
		break;
		case 0b1110:
			record_out();
			run_shortest(4000,17000,20000,TURN_ON,FUN_ON,SLANT_ON,speed1200_shortest_mollifier,5.86,1,2);
//			run_shortest(3100,11000,11000,TURN_ON,FUN_ON,SLANT_ON,speed1600_shortest_mollifier,0.99,1,0);
		break;
		case 0b1111:
			AdatiWayReturn(250,400,2000,3000,speed250_exploration,1,0);
		break;
	}


}


void mode_Running2(unsigned char main_modeR){

	pl_L_DriveMotor_mode(MOTOR_BREAK);
	pl_R_DriveMotor_mode(MOTOR_BREAK);
	reset_gyro();
	reset_speed();
	reset_distance();
	clear_Ierror();

	switch (main_modeR) {
		case 0b0000:
			tic_timer();
			AdatiWayReturn(300,400,3000,3000,speed300_exploration,0,1);
		break;
		case 0b0001:
			record_out();
			tic_timer();
			AdatiWayReturn(300,400,3000,3000,speed300_exploration,1,1);
		break;
		case 0b0010:
			record_out();
			run_shortest(4000,13000,14000,TURN_ON,FUN_ON,SLANT_ON,speed1600_shortest_mollifier,5.86,1,2);
		break;
		case 0b0011:
			record_out();
			run_shortest(4000,17000,20000,TURN_ON,FUN_ON,SLANT_ON,speed1600_shortest_mollifier,5.86,1,2);
		break;
		case 0b0100:

		break;
		case 0b0101:

		break;
		case 0b0110:

		break;
		case 0b0111:

		break;
		case 0b1000:

		break;
		case 0b1001:

		break;
		case 0b1010:

		break;
		case 0b1011:

		break;
		case 0b1100:

		break;
		case 0b1101:

		break;
		case 0b1110:

		break;
		case 0b1111:
			create_StepCountMap_queue();
			maze_display();
		break;
	}


}

void mode_Tuning2(unsigned char main_modeR){
	MOTOR_MODE mode;
	//int duty_L=0, duty_R=0;
	mode.WallControlMode=0;
	mode.WallControlStatus=0;
	mode.WallCutMode=0;
	mode.calMazeMode=0;
	pl_L_DriveMotor_mode(MOTOR_BREAK);
	pl_R_DriveMotor_mode(MOTOR_BREAK);

	pl_DriveMotor_standby(ON);
	reset_gyro();
	reset_speed();
	reset_distance();
	clear_Ierror();
	yaw_angle=0;
	switch (main_modeR) {
		case 0:
			sensor_line();
		break;
		case 1:
			while (1) {
				printf("dis90L=%f,dis90R=%f\n",g_sensor_distance[SENSOR_LEFT],g_sensor_distance[SENSOR_RIGHT] );
				wait_ms(500);
			}
		break;
		case 2:
			//sensor_line_slant();
			mode.WallControlMode=0;
			mode.WallControlStatus=0;mode.WallCutMode=0;mode.calMazeMode=0;
			reset_distance();
			record_mode=22;
			straight_table2(-150*sqrt(2), 0, 0, -300, 3000,mode);
			record_mode=100;
			while (MODE_SENSER_DEC) {HAL_Delay(1);}
			reset_distance();
			record_mode=23;
			straight_table2(-150*sqrt(2), 0, 0, -300, 3000,mode);
			record_mode=0;
		break;
		case 3:
			while (1) {
//				printf("SEN45R=%d,Sdis45R=%f\n",g_sensor[SENSOR_FRONT_RIGHT][0],g_sensor_distance_slant[SENSOR_FRONT_RIGHT]);

				printf("Sdis90L=%f,Sdis90R=%f,Sdis45L=%f,Sdis45R=%f\n"
						,g_sensor_distance_slant[SENSOR_LEFT][0],g_sensor_distance_slant[SENSOR_RIGHT][0]
						,g_sensor_distance_slant[SENSOR_FRONT_LEFT][0],g_sensor_distance_slant[SENSOR_FRONT_RIGHT][0]);
				wait_ms(500);
			}
		break;
		case 4://斜め直進(90)
			mode.WallControlMode=0;
			mode.WallControlStatus=0;mode.WallCutMode=0;mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER2+MAZE_SECTION/2,0,0,200*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			turning_table2(-45, 0, 0, -300*MAZE_SECTION/90, 3000*MAZE_SECTION/90);
			straight_table2(-BACK_TO_CENTER_FRONT_SLANT,0,0,-100*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			record_mode=18;
			straight_table2(90*3*sqrt(2), 0, 0, 300, 3000,mode);
		break;
		case 5://斜め直進(45)
			mode.WallControlMode=0;
			mode.WallControlStatus=0;mode.WallCutMode=0;mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER2+MAZE_SECTION/2,0,0,200*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			turning_table2(-45, 0, 0, -300*MAZE_SECTION/90, 3000*MAZE_SECTION/90);
			straight_table2(-BACK_TO_CENTER_FRONT_SLANT,0,0,-100*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			record_mode=19;
			straight_table2(90*3*sqrt(2), 0, 0, 300, 3000,mode);
		break;
		break;
		case 6://斜め直進(90)
			mode.WallControlMode=0;
			mode.WallControlStatus=0;mode.WallCutMode=0;mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER2+MAZE_SECTION/2,0,0,200*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			turning_table2(-45, 0, 0, -300*MAZE_SECTION/90, 3000*MAZE_SECTION/90);
			straight_table2(-BACK_TO_CENTER_FRONT_SLANT,0,0,-100*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			mode.WallControlMode=3;
			record_mode=18;
			straight_table2(90*5*sqrt(2), 0, 0, 1000, 9000,mode);
		break;
		case 7://斜め直進(45)
			mode.WallControlMode=0;
			mode.WallControlStatus=0;mode.WallCutMode=0;mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER2+MAZE_SECTION/2,0,0,200*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			turning_table2(-45, 0, 0, -300*MAZE_SECTION/90, 3000*MAZE_SECTION/90);
			straight_table2(-BACK_TO_CENTER_FRONT_SLANT,0,0,-100*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			mode.WallControlMode=3;
			record_mode=19;
			straight_table2(90*5*sqrt(2), 0, 0, 1000, 9000,mode);
		break;
		case 8://斜め直進(90)
			mode.WallControlMode=0;
			mode.WallControlStatus=0;mode.WallCutMode=0;mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER2+MAZE_SECTION/2,0,0,200*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			turning_table2(45, 0, 0, 300*MAZE_SECTION/90, 3000*MAZE_SECTION/90);
			straight_table2(-BACK_TO_CENTER_FRONT_SLANT,0,0,-100*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			mode.WallControlMode=3;
			highspeed_mode = 1;
			control_fun(5.86);
			reset_gyro();
			reset_speed();
			reset_distance();
			clear_Ierror();
			pl_FunMotor_start();
			HAL_Delay(600);
			record_mode=20;
			straight_table2(90*7*sqrt(2), 0, 0, 4000, 22000,mode);
		break;
		case 9://斜め直進(45)
			mode.WallControlMode=0;
			mode.WallControlStatus=0;mode.WallCutMode=0;mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER2+MAZE_SECTION/2,0,0,200*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			turning_table2(-45, 0, 0, -300*MAZE_SECTION/90, 3000*MAZE_SECTION/90);
			straight_table2(-BACK_TO_CENTER_FRONT_SLANT,0,0,-100*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			mode.WallControlMode=3;
			highspeed_mode = 1;
			control_fun(5.86);
			reset_gyro();
			reset_speed();
			reset_distance();
			clear_Ierror();
			pl_FunMotor_start();
			HAL_Delay(600);
			record_mode=21;
			straight_table2(90*7*sqrt(2), 0, 0, 4000, 22000,mode);
		break;
		case 10://前壁制御調整
			mode.WallControlMode=0;
			mode.WallControlStatus=0;mode.WallCutMode=0;mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER2,0,0,200*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			record_mode=27;
			HAL_Delay(300);
			record_mode=0;
			for(int i=0;i<8;i++){
			turning_table2(90, 0, 0, 400, 3000);
			}


		break;
		case 11://斜め直進(90)

		break;
		case 12://斜め直進(45)

		break;
		case 13://斜め直進(90)

		break;
		case 14://斜め直進(45)

		break;
		case 15://直線
			highspeed_mode = 1;
			control_fun(5.86);
			pl_FunMotor_start();
			HAL_Delay(600);
			//reset_gyro();
			reset_gyro_integral();
			reset_speed();
			reset_distance();
			clear_Ierror();
			record_mode=13;
//			mode.WallControlMode=0;
//			straight_table2(90*32, 0, 0, 4000, 17000,mode);
			mode.WallControlMode=1;
//			straight_table2(BACK_TO_CENTER_FRONT,0,1000,1000,1000*1000/ BACK_TO_CENTER_FRONT/2, mode);
			straight_table2(90*8, 0, 0, 4500, 30000,mode);
		break;
	}
	record_mode=0;
	pl_R_DriveMotor_mode(MOTOR_BREAK);
	pl_L_DriveMotor_mode(MOTOR_BREAK);
	wait_ms_NoReset(500);
	pl_FunMotor_stop();
	wait_ms_NoReset(500);
	pl_DriveMotor_standby(OFF);
	record_mode=0;
	highspeed_mode = 0;
	//pl_FunMotor_stop();
	while (g_sensor[0][0] <= SENSOR_FINGER_0 || g_sensor[2][0] <= SENSOR_FINGER_2 || g_sensor[4][0] <= SENSOR_FINGER_4) {
		HAL_Delay(1);
	}
	pl_r_blue_LED(ON);
	pl_l_blue_LED(ON);
	record_print();



}



void mode_Tuning1(unsigned char main_modeR){
	MOTOR_MODE mode;
	int duty_L=0, duty_R=0;
	mode.WallControlMode=0;
	mode.WallControlStatus=0;
	mode.WallCutMode=0;
	mode.calMazeMode=0;
	pl_L_DriveMotor_mode(MOTOR_BREAK);
	pl_R_DriveMotor_mode(MOTOR_BREAK);

	pl_DriveMotor_standby(ON);
	reset_gyro();
	reset_speed();
	reset_distance();
	clear_Ierror();
	yaw_angle=0;
	switch (main_modeR) {
		case 0:// 横壁制御の調整
			record_mode=7;
			mode.WallControlMode=0;
			wait_ms(150);
			record_mode=100;
			pl_r_blue_LED(ON);pl_l_blue_LED(ON);
			while (MODE_SENSER_DEC) {HAL_Delay(1);}
			pl_r_blue_LED(OFF);pl_l_blue_LED(OFF);
			wait_ms(600);
			record_mode=7;
			wait_ms(150);
			record_mode=100;
			pl_r_blue_LED(ON);pl_l_blue_LED(ON);
			while (MODE_SENSER_DEC) {HAL_Delay(1);}
			pl_r_blue_LED(OFF);pl_l_blue_LED(OFF);
			wait_ms(600);
			record_mode=7;
			wait_ms(150);
			straight_table2(90*4, 0, 0, 300, 6000,mode);
			record_mode=100;
			pl_r_blue_LED(ON);pl_l_blue_LED(ON);
			while (MODE_SENSER_DEC) {HAL_Delay(1);}
			pl_r_blue_LED(OFF);pl_l_blue_LED(OFF);
			wait_ms(600);
			reset_gyro();
			reset_speed();
			reset_distance();
			clear_Ierror();
			highspeed_mode = 1;
			control_fun(5.86);
			pl_FunMotor_start();
			HAL_Delay(600);
			//reset_gyro();
			reset_gyro_integral();
			reset_speed();
			clear_Ierror();
			record_mode=7;
			straight_table2(90*8, 0, 0, 4000, 22000,mode);
			record_mode=100;
		break;
		case 1://タイヤ径調整+壁制御ゲイン確認＋探索用のゲイン調整(直線)
			record_mode = 5;
			mode.WallControlMode=1;
			straight_table2(90*8, 0, 0, 300, 6000,mode);
		break;
		case 2://ジャイロ係数調整＋探索用のゲイン調整(旋回)
			highspeed_mode = 0;
			reset_gyro();
			control_fun(5.86);
			pl_FunMotor_start();
			HAL_Delay(600);
			reset_gyro_integral();
			reset_speed();
			reset_distance();
			clear_Ierror();
			record_mode=2;
			for(int i=0;i<8;i++){
			turning_table2(90, 0, 0, 400, 3000);
			}
			record_mode=100;
			wait_ms_NoReset(500);
			pl_FunMotor_stop();
			wait_ms_NoReset(500);
			pl_r_blue_LED(ON);pl_l_blue_LED(ON);
			while (MODE_SENSER_DEC) {HAL_Delay(1);}
			pl_r_blue_LED(OFF);pl_l_blue_LED(OFF);
			reset_gyro();
			control_fun(5.86);
			pl_FunMotor_start();
			HAL_Delay(600);
			reset_gyro_integral();
			reset_speed();
			reset_distance();
			clear_Ierror();
			record_mode=2;
			for(int i=0;i<8;i++){
			turning_table2(-90, 0, 0, -400, 3000);
			}
		break;
		case 3:// 中心の決定
			mode.WallControlMode=0;
			straight_table2(BACK_TO_CENTER_FRONT, 0, 0, 300, 6000,mode);
		break;
		case 4://探索用壁切れの確認
			record_mode = 7;
			mode.WallControlMode=0;
			mode.WallControlStatus=0;
			mode.WallCutMode=0;
			mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER_FRONT + 135, 0, 300, 300,3000,mode);
			mode.WallCutMode=1;
			mode.WallControlMode=0;
			straight_table2(22.5, 300, 300, 300,3000,mode);
			mode.WallControlMode=0;
			mode.WallCutMode=0;
			straight_table2(45 + 22.5, 300, 0, 300,3000,mode);
		break;
		case 5://角度伝達誤差補償の確認
			pl_r_blue_LED(ON);
			pl_l_blue_LED(ON);
			get_duty(1, 1,&duty_L,&duty_R);
			pl_DriveMotor_duty(duty_L,duty_R);
			pl_DriveMotor_start();
			wait_ms(3000);
			record_mode=11;
			wait_ms(2000);
			record_mode=0;
			//reset_EncoderGyro_MeanVariance();
			pl_DriveMotor_stop();
			//printf("%f\n",g_encY_variance);
			pl_r_blue_LED(OFF);
			pl_l_blue_LED(OFF);
		break;
		case 6://右スラローム(探索)
			testturning(speed300_exploration,0,0,0,0,0);
		break;
		case 7://左スラローム(探索)
			testturning(speed300_exploration,1,0,0,0,0);
		break;
		case 8://壁切れ(直線最短)1111 1000
			highspeed_mode = 1;
			control_fun(5.86);
			pl_FunMotor_start();
			HAL_Delay(600);
			//reset_gyro();
			reset_gyro_integral();
			reset_speed();
			clear_Ierror();
			record_mode=8;
			mode.WallControlMode=0;
			mode.WallControlStatus=0;
			mode.WallCutMode=0;
			mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER_FRONT + 90, 0, 1400, 1400,15000,mode);
			mode.WallCutMode=2;
			mode.WallControlMode=0;
			straight_table2(22.5, 1400, 1400, 1400,15000,mode);
			mode.WallControlMode=0;
			mode.WallCutMode=0;
			straight_table2(90+22.5, 1400, 0, 1400,15000,mode);
		break;
		case 9://壁切れ45度斜め(考え中)右
			mode.WallControlMode=0;//3でもいいかも
			mode.WallControlStatus=0;
			mode.WallCutMode=0;
			mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER2+MAZE_SECTION/2,0,0,200*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			turning_table2(-45, 0, 0, -300*MAZE_SECTION/90, 3000*MAZE_SECTION/90);
			straight_table2(-BACK_TO_CENTER_FRONT_SLANT,0,0,-100*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			highspeed_mode = 1;
			control_fun(5.86);
			pl_FunMotor_start();
			HAL_Delay(600);
			//reset_gyro();
			reset_gyro_integral();
			reset_speed();
			clear_Ierror();
			record_mode=16;
			straight_table2(BACK_TO_CENTER_FRONT_SLANT + 90*sqrt(2), 0, 1400, 1400,15000,mode);
			mode.WallCutMode=3;
			mode.WallControlMode=0;
			straight_table2(22.5*sqrt(2), 1400, 1400, 1400,15000,mode);
			mode.WallControlMode=0;
			mode.WallCutMode=0;
			straight_table2(45*sqrt(2) + 22.5*sqrt(2), 1400, 0, 1400,15000,mode);
		break;
		case 10://壁切れ45度斜め(考え中)左

			mode.WallControlMode=0;//3でもいいかも
			mode.WallControlStatus=0;
			mode.WallCutMode=0;
			mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER2+MAZE_SECTION/2,0,0,200*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			turning_table2(45, 0, 0, 300*MAZE_SECTION/90, 3000*MAZE_SECTION/90);
			straight_table2(-BACK_TO_CENTER_FRONT_SLANT,0,0,-100*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			highspeed_mode=1;
			control_fun(5.86);
			pl_FunMotor_start();
			HAL_Delay(600);
			//reset_gyro();
			reset_gyro_integral();
			reset_speed();
			clear_Ierror();
			record_mode=17;
			straight_table2(BACK_TO_CENTER_FRONT_SLANT+90*sqrt(2), 0, 1400, 1400,15000,mode);
			mode.WallCutMode=4;
			mode.WallControlMode=0;
			straight_table2(22.5*sqrt(2), 1400, 1400, 1400,15000,mode);
			mode.WallControlMode=0;
			mode.WallCutMode=0;
			straight_table2(45*sqrt(2) + 22.5*sqrt(2), 1400, 0, 1400,15000,mode);
		break;
		case 11://斜め直進(90)
			mode.WallControlMode=0;
			mode.WallControlStatus=0;mode.WallCutMode=0;mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER2+MAZE_SECTION/2,0,0,200*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			turning_table2(-45, 0, 0, -300*MAZE_SECTION/90, 3000*MAZE_SECTION/90);
			straight_table2(-BACK_TO_CENTER_FRONT_SLANT,0,0,-100*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			record_mode=9;
			straight_table2(90*3*sqrt(2), 0, 0, 300, 3000,mode);
		break;
		case 12://斜め直進(45)
			mode.WallControlMode=0;
			mode.WallControlStatus=0;mode.WallCutMode=0;mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER2+MAZE_SECTION/2,0,0,200*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			turning_table2(-45, 0, 0, -300*MAZE_SECTION/90, 3000*MAZE_SECTION/90);
			straight_table2(-BACK_TO_CENTER_FRONT_SLANT,0,0,-100*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			record_mode=10;
			straight_table2(90*3*sqrt(2), 0, 0, 300, 3000,mode);
		break;
		case 13://斜め直進(90)
			mode.WallControlMode=0;
			mode.WallControlStatus=0;mode.WallCutMode=0;mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER2+MAZE_SECTION/2,0,0,200*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			turning_table2(-45, 0, 0, -300*MAZE_SECTION/90, 3000*MAZE_SECTION/90);
			straight_table2(-BACK_TO_CENTER_FRONT_SLANT,0,0,-100*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			mode.WallControlMode=3;
			record_mode=9;
			straight_table2(90*3*sqrt(2), 0, 0, 300, 3000,mode);
		break;
		case 14://斜め直進(45)
			mode.WallControlMode=0;
			mode.WallControlStatus=0;mode.WallCutMode=0;mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER2+MAZE_SECTION/2,0,0,200*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			turning_table2(-45, 0, 0, -300*MAZE_SECTION/90, 3000*MAZE_SECTION/90);
			straight_table2(-BACK_TO_CENTER_FRONT_SLANT,0,0,-100*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			mode.WallControlMode=3;
			record_mode=10;
			highspeed_mode = 1;
			control_fun(5.86);
			reset_gyro();
			reset_speed();
			reset_distance();
			clear_Ierror();
			pl_FunMotor_start();
			HAL_Delay(600);
			straight_table2(90*15, 0, 0, 4000, 18000,mode);
		break;
		case 15://直線
			highspeed_mode = 1;
			control_fun(5.86);
			pl_FunMotor_start();
			HAL_Delay(600);
			//reset_gyro();
			reset_gyro_integral();
			reset_speed();
			reset_distance();
			clear_Ierror();
			// record_mode=13;
			record_mode=2;
//			mode.WallControlMode=0;
//			straight_table2(90*32, 0, 0, 4000, 17000,mode);
			mode.WallControlMode=1;
//			straight_table2(BACK_TO_CENTER_FRONT,0,1000,1000,1000*1000/ BACK_TO_CENTER_FRONT/2, mode);
			straight_table2(90*8, 0, 0, 4000, 30000,mode);
			//straight_table_dis(90*8, 0, 0, 4000,30000,11000,mode);
		break;
	}
	record_mode=0;
	pl_R_DriveMotor_mode(MOTOR_BREAK);
	pl_L_DriveMotor_mode(MOTOR_BREAK);
	wait_ms_NoReset(500);
	pl_FunMotor_stop();
	wait_ms_NoReset(500);
	pl_DriveMotor_standby(OFF);
	record_mode=0;
	highspeed_mode = 0;
	//pl_FunMotor_stop();
	while (g_sensor[0][0] <= SENSOR_FINGER_0 || g_sensor[2][0] <= SENSOR_FINGER_2 || g_sensor[4][0] <= SENSOR_FINGER_4) {
		HAL_Delay(1);
	}
	pl_r_blue_LED(ON);
	pl_l_blue_LED(ON);
	record_print();



}


void mode_Tuning0(unsigned char main_modeR){
	MOTOR_MODE mode;
	int duty_L=0, duty_R=0;
	mode.WallControlMode=0;
	mode.WallControlStatus=0;
	mode.WallCutMode=0;
	mode.calMazeMode=0;
	//pl_FunMotor_duty(150);
	//pl_FunMotor_start();
	//HAL_Delay(1000);
	pl_L_DriveMotor_mode(MOTOR_BREAK);
	pl_R_DriveMotor_mode(MOTOR_BREAK);
	reset_gyro();
	reset_speed();
	reset_distance();
	clear_Ierror();
	switch (main_modeR) {
		case 0://タイヤ径　エンコーダ+PID
			record_mode=3;
			mode.WallControlMode=0;
			//straight_table2(720, 0, 0, 3000, 20000,mode);
			straight_table2(540, 0, 0, 1600, 13000,mode);
			turning_table2(180, 0, 0, 400, 4000);
			straight_table2(540, 0, 0, 2000, 15000,mode);
			turning_table2(180, 0, 0, 400, 4000);
			straight_table2(540, 0, 0, 2400, 20000,mode);
			//straight_table2(720, 0, 0, 3000, 20000,mode);
			//straight_table2(450, 0, 0, 1000, 13000,mode);
			//control_test_motor2(0,0,10);
		break;
		case 1://トレッド幅??ジャイロ

			record_mode = 5;
			mode.WallControlMode=1;
//			straight_table2(400, 0, 0, 400, 3000,mode);
			straight_table2(90*15, 0, 0, 300, 6000,mode);
			//straight_table2(-BACK_TO_CENTER, 0,0,-150,1000, mode);
			//turning_table2(360*10, 0, 0, 800, 8000);
			//control_test_motor2(1,0,5);
		break;
		case 2://壁制御
			record_mode=4;
			mode.WallControlMode=1;
			straight_table2(90*8, 0, 0, 300, 6000,mode);
			//control_test_motor2(0,1,6);
		break;
		case 3://右スラローム(探索)
//			straight_table2(180, 0, 300, 300,300 * 300  / 2 / 90,mode);
//			mollifier_slalom_table(300,-90,500);
//			straight_table2(180, 300, 0, 300,300 * 300  / 2 / 90,mode);

			testturning(speed300_exploration,0,0,0,0,0);
		break;
		case 4://左スラローム(探索)
//			straight_table2(180, 0, 300, 300,300 * 300  / 2 / 90,mode);
//			mollifier_slalom_table(300,90,500);
//			straight_table2(180, 300, 0, 300,300 * 300  / 2 / 90,mode);
			testturning(speed300_exploration,1,0,0,0,0);
		break;
		case 5://直進(制御なし)
			//testturning(speed300_exploration,6,1,0,0,1);
			straight_table2(BACK_TO_CENTER_FRONT+45,0,0,200,5000, mode);
			turning_table2(45, 0, 0, 300, 3000);
			straight_table2(-BACK_TO_CENTER_FRONT_SLANT,0,0,-100,5000, mode);
//			highspeed_mode = 1;
//			pl_FunMotor_duty(0.99);
//			pl_FunMotor_start();
//			HAL_Delay(600);
//			reset_gyro();
//			reset_speed();
//			reset_distance();
//			clear_Ierror();
//			record_mode=6;
//			mode.WallControlMode=1;
//			theta_comp_gain=0;
//			straight_table2(90*8, 0, 0, 2800, 18000,mode);
//			mode.WallControlMode=0;
//			theta_comp_gain=0;
//			straight_table2(-90*8, 0, 0, -2000, 14000,mode);
//			mode.WallControlMode=1;
//			theta_comp_gain=1;
//			straight_table2(90*8, 0, 0, 2800, 18000,mode);
//			mode.WallControlMode=0;
//			theta_comp_gain=1;
//			straight_table2(-90*8, 0, 0, -2000, 14000,mode);

		break;
		case 6://斜め直進(制御あり)
			highspeed_mode = 1;
			control_fun(5.86);
			pl_FunMotor_start();
			HAL_Delay(600);
			//reset_gyro();
			reset_gyro_integral();
			reset_speed();
			reset_distance();
			clear_Ierror();
			record_mode=5;
			mode.WallControlMode=1;
			//straight_table_max(90*8, 0, 0, 4000, 17000,30000,mode);
			straight_acceleration_lpf=1000*1000/ BACK_TO_CENTER_FRONT;
			straight_table2(BACK_TO_CENTER_FRONT,0,1000,1000,1000*1000/ BACK_TO_CENTER_FRONT/2, mode);
			straight_table_max(90*8-BACK_TO_CENTER_FRONT, 1000, 0, 4000, 17000,17000,mode);
		break;
		case 7://斜め直進(平松さん式制御あり)
			highspeed_mode = 1;
			control_fun(5.86);
			pl_FunMotor_start();
			HAL_Delay(600);
			//reset_gyro();
			reset_gyro_integral();
			reset_speed();
			reset_distance();
			clear_Ierror();
			record_mode=3;
			mode.WallControlMode=1;
			straight_table_dis(90*8, 0, 0, 4000, 17000,30000,mode);
		break;
		case 8://宴会芸
			record_mode=2;
			pl_r_blue_LED(ON);
			pl_l_blue_LED(ON);
			no_angle();
			pl_r_blue_LED(OFF);
			pl_l_blue_LED(OFF);
		break;
		case 9://システム同定enc

			pl_r_blue_LED(ON);
			pl_l_blue_LED(ON);
//			straight_table_ff(90, 0, 300, 300, 500);
//			straight_table_ff(90, 300, 600, 600, 1500);
//			straight_table_ff(90, 600, 300, 600, 1500);
//			straight_table_ff(90, 300, 0, 300, 500);

			get_duty(1, 1,&duty_L,&duty_R);
			pl_DriveMotor_duty(duty_L,duty_R);
			pl_DriveMotor_start();
			wait_ms(3000);
			record_mode=11;
			wait_ms(2000);
//			while (g_sensor[0][0] <= SENSOR_FINGER_0 || g_sensor[2][0] <= SENSOR_FINGER_2 || g_sensor[4][0] <= SENSOR_FINGER_4) {
//				wait_ms(1);
//				if(record_rupe_flag==1){
//					break;
//				}
//			}
			pl_DriveMotor_stop();
			pl_r_blue_LED(OFF);
			pl_l_blue_LED(OFF);
		break;
		case 10://システム同定gyro
			record_mode=6;
			pl_r_blue_LED(ON);
			pl_l_blue_LED(ON);
//			turning_table_ff(90, 0, 300, 300, 500);
//			turning_table_ff(90, 300, 600, 600, 1500);
//			turning_table_ff(90, 600, 300, 600, 1500);
//			turning_table_ff(90, 300, 0, 300, 500);
			get_duty(-1.0, 1.0,&duty_L,&duty_R);
			pl_DriveMotor_duty(duty_L,duty_R);
			pl_DriveMotor_start();
			while (g_sensor[0][0] <= SENSOR_FINGER_0 || g_sensor[2][0] <= SENSOR_FINGER_2 || g_sensor[4][0] <= SENSOR_FINGER_4) {
				wait_ms(1);
				if(record_rupe_flag==1){
					record_mode=0;
					break;
				}
			}
			pl_DriveMotor_stop();
			pl_r_blue_LED(OFF);
			pl_l_blue_LED(OFF);
		break;
		case 11:
		record_mode=1;
		mode.WallControlMode=0;
		straight_table2(BACK_TO_CENTER_FRONT_SLANT, 0, 0, 300, 6000,mode);
		break;
		case 12:
		record_mode=2;
		for(int i=0;i<40;i++){
		turning_table2(90, 0, 0, 400, 3000);
		}
		break;
		case 13:
			record_mode=2;
			mode.WallControlMode=1;
			straight_table2(BACK_TO_CENTER_FRONT + 135, 0, speed300_exploration.slalom_R.g_speed, speed300_exploration.slalom_R.g_speed,
					speed300_exploration.slalom_R.g_speed * speed300_exploration.slalom_R.g_speed  / 2 / 45,mode);
			for(int i=0;i<40;i++){
			straight_table2(MAZE_OFFSET, speed300_exploration.slalom_R.g_speed, speed300_exploration.slalom_R.g_speed, speed300_exploration.slalom_R.g_speed,
									speed300_exploration.slalom_R.g_speed * speed300_exploration.slalom_R.g_speed  / 2 / 45,mode);
			slalomR(speed300_exploration.slalom_R, OFF,EXPLORATION,0,300);
			}
			mode.WallControlMode=0;
			straight_table2(45, speed300_exploration.slalom_R.g_speed, 0, speed300_exploration.slalom_R.g_speed,
					speed300_exploration.slalom_R.g_speed * speed300_exploration.slalom_R.g_speed  / 2 / 45,mode);

			//pl_r_blue_LED(ON);
			//pl_l_blue_LED(ON);
			//no_frontwall_straight();
			//pl_r_blue_LED(OFF);
			//pl_l_blue_LED(OFF);
		break;
		case 14://宴会芸＋吸引
//			record_mode=2;
//			for(int i=0;i<40;i++){
//			turning_table2(90, 0, 0, 930, 11000);
//			}
			highspeed_mode = 1;
			control_fun(5.86);
			pl_FunMotor_start();
			HAL_Delay(600);
			//reset_gyro();
			reset_gyro_integral();
			reset_speed();
			reset_distance();
			clear_Ierror();
			record_mode=2;
//			mode.WallControlMode=0;
//			straight_table2(90*32, 0, 0, 4000, 17000,mode);
			mode.WallControlMode=1;
//			straight_table2(BACK_TO_CENTER_FRONT,0,1000,1000,1000*1000/ BACK_TO_CENTER_FRONT/2, mode);
			HAL_Delay(600);
			//			straight_table2(90*7, 0, 0, 2500, 14000,mode);
		break;
		case 15:
			highspeed_mode = 1;
			control_fun(5.86);
			reset_gyro();
			reset_speed();
			reset_distance();
			clear_Ierror();
			pl_FunMotor_start();
			HAL_Delay(600);
			record_mode=13;
//			mode.WallControlMode=0;
//			straight_table2(90*32, 0, 0, 4000, 17000,mode);
			mode.WallControlMode=1;
//			straight_table2(BACK_TO_CENTER_FRONT,0,1000,1000,1000*1000/ BACK_TO_CENTER_FRONT/2, mode);
			straight_table2(90*15, 0, 0, 4000, 15000,mode);
		break;
	}
	record_mode=0;
	pl_R_DriveMotor_mode(MOTOR_BREAK);
	pl_L_DriveMotor_mode(MOTOR_BREAK);
	wait_ms_NoReset(500);
	pl_FunMotor_stop();
	wait_ms_NoReset(500);
	pl_DriveMotor_standby(OFF);
	record_mode=0;
	//pl_FunMotor_stop();
	while (g_sensor[0][0] <= SENSOR_FINGER_0 || g_sensor[2][0] <= SENSOR_FINGER_2 || g_sensor[4][0] <= SENSOR_FINGER_4) {
		HAL_Delay(1);
	}
	pl_r_blue_LED(ON);
	pl_l_blue_LED(ON);
	record_print();



}


void mode_WallSensorTuning(unsigned char main_modeR){
	MOTOR_MODE mode;
	mode.WallControlMode=0;
	mode.WallControlStatus=0;
	mode.WallCutMode=0;
	mode.calMazeMode=0;
	//pl_FunMotor_duty(150);
	//pl_FunMotor_start();
	//HAL_Delay(1000);
	pl_DriveMotor_standby(ON);
	pl_L_DriveMotor_mode(MOTOR_BREAK);
	pl_R_DriveMotor_mode(MOTOR_BREAK);
	reset_gyro();
	reset_speed();
	clear_Ierror();

	switch (main_modeR) {
		case 0://横壁追従制御
			record_mode=16;
			mode.WallControlMode=1;
			straight_table2(180*8, 0, 0, 500, 5000,mode);
		break;
		case 1://前壁制御0100 0001
			for(int ii=0;ii<=100;ii++){
			no_frontwall_straight();
			}
		break;
		case 2://壁切れ90度
			record_mode = 7;
			mode.WallControlMode=0;
			mode.WallControlStatus=0;
			mode.WallCutMode=0;
			mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER_FRONT + 135, 0, 300, 300,3000,mode);
			mode.WallCutMode=1;
			mode.WallControlMode=0;
			straight_table2(22.5, 300, 300, 300,3000,mode);
			mode.WallControlMode=0;
			mode.WallCutMode=0;
			straight_table2(45 + 22.5, 300, 0, 300,3000,mode);
		break;
		case 3://壁切れ45度//90°大回り
			record_mode = 8;
			mode.WallControlMode=0;
			mode.WallControlStatus=0;
			mode.WallCutMode=0;
			mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER_FRONT + 90, 0, 300, 300,3000,mode);
			mode.WallCutMode=2;
			mode.WallControlMode=0;
			straight_table2(22.5, 300, 300, 300,3000,mode);
			mode.WallControlMode=0;
			mode.WallCutMode=0;
			straight_table2(45+22.5, 300, 0, 300,3000,mode);
		break;
		case 4://壁切れ45度斜め(考え中)
			record_mode = 8;
			mode.WallControlMode=0;//3でもいいかも
			mode.WallControlStatus=0;
			mode.WallCutMode=0;
			mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER_SLANT + 90*sqrt(2), 0, 300, 300,3000,mode);
			mode.WallCutMode=3;
			mode.WallControlMode=0;
			straight_table2(22.5*sqrt(2), 300, 300, 300,3000,mode);
			mode.WallControlMode=0;
			mode.WallCutMode=0;
			straight_table2(45*sqrt(2) + 22.5*sqrt(2), 300, 0, 300,3000,mode);
		break;
		case 5://壁切れ45度斜め(考え中)
			mode.WallControlMode=0;//3でもいいかも
			mode.WallControlStatus=0;
			mode.WallCutMode=0;
			mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER_SLANT+90*sqrt(2), 0, 300, 300,3000,mode);
			mode.WallCutMode=4;
			mode.WallControlMode=0;
			straight_table2(22.5*sqrt(2), 300, 300, 300,3000,mode);
			mode.WallControlMode=0;
			mode.WallCutMode=0;
			straight_table2(45*sqrt(2) + 22.5*sqrt(2), 300, 0, 300,3000,mode);
		break;
		case 6://斜め直進(90)
			mode.WallControlMode=0;
			mode.WallControlStatus=0;mode.WallCutMode=0;mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER_FRONT+MAZE_SECTION/2,0,0,200*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			turning_table2(-45, 0, 0, -300*MAZE_SECTION/90, 3000*MAZE_SECTION/90);
			straight_table2(-BACK_TO_CENTER_FRONT_SLANT,0,0,-100*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			record_mode=9;
			straight_table2(90*3*sqrt(2), 0, 0, 300, 3000,mode);
		break;
		case 7://斜め直進(45)

			mode.WallControlMode=0;
			mode.WallControlStatus=0;mode.WallCutMode=0;mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER_FRONT+MAZE_SECTION/2,0,0,200*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			turning_table2(-45, 0, 0, -300*MAZE_SECTION/90, 3000*MAZE_SECTION/90);
			straight_table2(-BACK_TO_CENTER_FRONT_SLANT,0,0,-100*MAZE_SECTION/90,5000*MAZE_SECTION/90, mode);
			record_mode=10;
			straight_table2(90*3*sqrt(2), 0, 0, 300, 3000,mode);

		break;
		case 8://斜め直進(平松さん式制御あり)
			record_mode=7;
			mode.WallControlMode=3;
			straight_table2(90*3*sqrt(2), 0, 0, 300, 3000,mode);
		break;
		case 9:
			record_mode=8;
			mode.WallControlMode=3;
			straight_table2(90*7*sqrt(2), 0, 0, 700, 10000,mode);
		break;
		case 10:
		break;
		case 11:
		break;
		case 15:
			mode.WallControlMode=5;
			mode.WallControlStatus=0;
			mode.WallCutMode=0;
			mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER_FRONT + 180, 0, 500, 500,1500,mode);
			mode.WallCutMode=2;
			mode.WallControlMode=0;
			straight_table2(45, 500, 500, 500,1500,mode);
			mode.WallControlMode=0;
			mode.WallCutMode=0;
			straight_table2(90+45, 500, 0, 500,1500,mode);
		break;
	}
	record_mode=0;
	pl_R_DriveMotor_mode(MOTOR_BREAK);
	pl_L_DriveMotor_mode(MOTOR_BREAK);
	wait_ms_NoReset(500);
	pl_FunMotor_stop();
	record_mode=0;
	wait_ms_NoReset(500);
	pl_DriveMotor_standby(OFF);

	//pl_FunMotor_stop();
	while (g_sensor[0][0] <= SENSOR_FINGER_0 || g_sensor[2][0] <= SENSOR_FINGER_2 || g_sensor[4][0] <= SENSOR_FINGER_4) {
		HAL_Delay(1);
	}
	pl_r_blue_LED(ON);
	pl_l_blue_LED(ON);
	record_print();



}





void mode_WallSensorTuning_fast(unsigned char main_modeR){
	MOTOR_MODE mode;
	mode.WallControlMode=0;
	mode.WallControlStatus=0;
	mode.WallCutMode=0;
	mode.calMazeMode=0;
	//pl_FunMotor_duty(150);
	//pl_FunMotor_start();
	//HAL_Delay(1000);
	pl_DriveMotor_standby(ON);
	pl_L_DriveMotor_mode(MOTOR_BREAK);
	pl_R_DriveMotor_mode(MOTOR_BREAK);
	control_fun(5.86);
	pl_FunMotor_start();
	HAL_Delay(600);
	//reset_gyro();
	reset_gyro_integral();
	reset_speed();
	clear_Ierror();
	yaw_angle=0;

	switch (main_modeR) {
		case 0://横壁追従制御
			record_mode=15;
			mode.WallControlMode=1;
			highspeed_mode=1;
			straight_table2(BACK_TO_CENTER_FRONT+180*10, 0, 0, 5000, 24000,mode);
			highspeed_mode=0;
		break;
		case 1://なし
			record_mode=8;
			mode.WallControlMode=1;
			mode.WallControlStatus=0;
			mode.WallCutMode=0;
			mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER_FRONT + 90, 0, 1000, 1000,10000,mode);
			mode.WallCutMode=2;
			mode.WallControlMode=0;
			straight_table2(22.5, 1000, 1000, 1000,10000,mode);
			mode.WallControlMode=1;
			mode.WallCutMode=0;
			straight_table2(90+22.5, 1000, 0, 1000,10000,mode);
		break;
		case 2://なし
			record_mode=8;
			mode.WallControlMode=1;
			mode.WallControlStatus=0;
			mode.WallCutMode=0;
			mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER_FRONT + 90, 0, 1200, 1200,10000,mode);
			mode.WallCutMode=2;
			mode.WallControlMode=0;
			straight_table2(22.5, 1200, 1200, 1200,10000,mode);
			mode.WallControlMode=1;
			mode.WallCutMode=0;
			straight_table2(90+22.5, 1200, 0, 1200,10000,mode);
		break;
		case 3://壁切れ45度//90°大回り
			record_mode=8;
			mode.WallControlMode=1;
			mode.WallControlStatus=0;
			mode.WallCutMode=0;
			mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER_FRONT + 90, 0, 1400, 1400,13000,mode);
			mode.WallCutMode=2;
			mode.WallControlMode=0;
			straight_table2(22.5, 1400, 1400, 1400,13000,mode);
			mode.WallControlMode=1;
			mode.WallCutMode=0;
			straight_table2(90+22.5, 1400, 0, 1400,13000,mode);
		break;
		case 4://壁切れ45度斜め(考え中)右
			record_mode=8;
			mode.WallControlMode=0;//3でもいいかも
			mode.WallControlStatus=0;
			mode.WallCutMode=0;
			mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER_FRONT_SLANT + 90*sqrt(2), 0, 1000, 1000,10000,mode);
			mode.WallCutMode=3;
			mode.WallControlMode=0;
			straight_table2(22.5*sqrt(2), 1000, 1000, 1000,10000,mode);
			mode.WallControlMode=0;
			mode.WallCutMode=0;
			straight_table2(45*sqrt(2) + 22.5*sqrt(2), 1000, 0, 1000,10000,mode);
		break;
		case 5://壁切れ45度斜め(考え中)左
			record_mode=8;
			mode.WallControlMode=0;//3でもいいかも
			mode.WallControlStatus=0;
			mode.WallCutMode=0;
			mode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER_FRONT_SLANT+90*sqrt(2), 0, 1000, 1000,10000,mode);
			mode.WallCutMode=4;
			mode.WallControlMode=0;
			straight_table2(22.5*sqrt(2), 1000, 1000, 1000,10000,mode);
			mode.WallControlMode=0;
			mode.WallCutMode=0;
			straight_table2(45*sqrt(2) + 22.5*sqrt(2), 1000, 0, 1000,10000,mode);
		break;
		case 6://斜め直進(制御なし計測用)
			record_mode=7;
			mode.WallControlMode=0;
			straight_table2(180*3*sqrt(2)+90*sqrt(2), 0, 0, 2000, 20000,mode);
		break;
		case 7://斜め直進(制御なし計測用)
			record_mode=9;
			mode.WallControlMode=0;
			straight_table2(180*3*sqrt(2)+90*sqrt(2), 0, 0, 2000, 20000,mode);

		break;
		case 8://斜め直進(平松さん式制御あり)
			record_mode=7;
			mode.WallControlMode=3;
			straight_table2(180*3*sqrt(2), 0, 0, 3500, 10000,mode);
		break;
		case 9:
			record_mode=8;
			mode.WallControlMode=3;
			straight_table2(BACK_TO_CENTER_SLANT+180*4*sqrt(2), 0, 0, 3500, 20000,mode);
		break;
		case 10:
			record_mode=10;
			mode.WallControlMode=3;
			straight_table2(180*3*sqrt(2), 0, 0, 3500, 20000,mode);
		break;
		case 11:
			record_mode=11;
			mode.WallControlMode=3;
			straight_table2(180*3*sqrt(2), 0, 0, 3000, 15000,mode);
		break;
		case 12:
			record_mode=12;
			mode.WallControlMode=3;
			straight_table2(180*3*sqrt(2), 0, 0, 3000, 15000,mode);
		break;
		case 15:
		break;
	}
	record_mode=0;
	pl_R_DriveMotor_mode(MOTOR_BREAK);
	pl_L_DriveMotor_mode(MOTOR_BREAK);
	wait_ms_NoReset(500);
	pl_FunMotor_stop();
	record_mode=0;
	wait_ms_NoReset(500);
	pl_DriveMotor_standby(OFF);

	//pl_FunMotor_stop();
	while (g_sensor[0][0] <= SENSOR_FINGER_0 || g_sensor[2][0] <= SENSOR_FINGER_2 || g_sensor[4][0] <= SENSOR_FINGER_4) {
		HAL_Delay(1);
	}
	pl_r_blue_LED(ON);
	pl_l_blue_LED(ON);
	record_print();



}




