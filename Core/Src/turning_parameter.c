/*
 * turning_parameter.c
 *
 *  Created on: Jan 9, 2024
 *      Author: sf199
 */


/*
 * turning_parameter.c
 *
 *  Created on: 2023/01/31
 *      Author: sf199
 */


/*
 * parameter.c
 *
 *  Created on: 2019/11/18
 *      Author: sf199
 */
#include "turning_parameter.h"
#include "define.h"


parameter_speed speed300_exploration;
parameter_speed speed250_exploration;

parameter_speed speed300_shortest;

parameter_speed speed600_shortest;

parameter_speed speed600_shortest_mollifier;

parameter_speed speed1000_shortest_mollifier;
parameter_speed speed1200_shortest_mollifier;
parameter_speed speed1400_shortest_mollifier;

parameter_speed speed1600_shortest_mollifier;

float get_center_velocity(parameter_speed Howspeed, int pass_number) {
	float End_velocity;
	if (pass_number == -2) {
		End_velocity = Howspeed.slalom_R.g_speed;
	} else if (pass_number == -3) {
		End_velocity = Howspeed.slalom_L.g_speed;
	} else if (pass_number == -4) {
		End_velocity = Howspeed.turn90_R.g_speed;
	} else if (pass_number == -5) {
		End_velocity = Howspeed.turn90_L.g_speed;
	} else if (pass_number == -6) {
		End_velocity = Howspeed.turn180_R.g_speed;
	} else if (pass_number == -7) {
		End_velocity = Howspeed.turn180_L.g_speed;
	} else if (pass_number == -8) {
		End_velocity = Howspeed.turn45in_R.g_speed;
	} else if (pass_number == -9) {
		End_velocity = Howspeed.turn45in_L.g_speed;
	} else if (pass_number == -10) {
		End_velocity = Howspeed.turn135in_R.g_speed;
	} else if (pass_number == -11) {
		End_velocity = Howspeed.turn135in_L.g_speed;
	} else if (pass_number == -12) {
		End_velocity = Howspeed.turn45out_R.g_speed;
	} else if (pass_number == -13) {
		End_velocity = Howspeed.turn45out_L.g_speed;
	} else if (pass_number == -14) {
		End_velocity = Howspeed.turn135out_R.g_speed;
	} else if (pass_number == -15) {
		End_velocity = Howspeed.turn135out_L.g_speed;
	} else if (pass_number == -16) {
		End_velocity = Howspeed.V90_R.g_speed;
	} else if (pass_number == -17) {
		End_velocity = Howspeed.V90_L.g_speed;
	} else {
		End_velocity = Howspeed.TurnCentervelocity;
	}

	return End_velocity;
}

void input_parameter(void) {

	speed300_exploration.SlalomCentervelocity = 300;
	speed300_exploration.TurnCentervelocity = 300;

	speed300_exploration.slalom_R.g_speed =
			speed300_exploration.SlalomCentervelocity;
	speed300_exploration.slalom_R.t_speed = 1050; //550
	speed300_exploration.slalom_R.t_acc = 20000; //10000
	speed300_exploration.slalom_R.f_ofset = 6; //55;
	speed300_exploration.slalom_R.e_ofset = 16;

	speed300_exploration.slalom_L.g_speed =
			speed300_exploration.SlalomCentervelocity;
	speed300_exploration.slalom_L.t_speed = 1150;
	speed300_exploration.slalom_L.t_acc = 20000;
	speed300_exploration.slalom_L.f_ofset = 4; //50;
	speed300_exploration.slalom_L.e_ofset = 18;

	speed300_exploration.turn45in_R.g_speed =
			speed300_exploration.SlalomCentervelocity;
	speed300_exploration.turn45in_R.t_speed = 570;
	speed300_exploration.turn45in_R.t_acc = 10000;
	speed300_exploration.turn45in_R.f_ofset = 30; //50;
	speed300_exploration.turn45in_R.e_ofset = 24;

	speed250_exploration.SlalomCentervelocity = 250;
	speed250_exploration.TurnCentervelocity = 250;

	speed250_exploration.slalom_R.g_speed =
			speed250_exploration.SlalomCentervelocity;
	speed250_exploration.slalom_R.t_speed = 830; //550
	speed250_exploration.slalom_R.t_acc = 9000; //10000
	speed250_exploration.slalom_R.f_ofset = 0.5; //55;
	speed250_exploration.slalom_R.e_ofset = 20;

	speed250_exploration.slalom_L.g_speed =
			speed250_exploration.SlalomCentervelocity;
	speed250_exploration.slalom_L.t_speed = 730;
	speed250_exploration.slalom_L.t_acc = 9000;
	speed250_exploration.slalom_L.f_ofset = 0.5; //50;
	speed250_exploration.slalom_L.e_ofset = 19;


	speed300_shortest.SlalomCentervelocity =
			speed300_exploration.SlalomCentervelocity;
	speed300_shortest.TurnCentervelocity =
			speed300_exploration.TurnCentervelocity + 200;

	speed300_shortest.slalom_R.g_speed = speed300_exploration.slalom_R.g_speed;
	speed300_shortest.slalom_R.t_speed = speed300_exploration.slalom_R.t_speed;
	speed300_shortest.slalom_R.t_acc = speed300_exploration.slalom_R.t_acc;
	speed300_shortest.slalom_R.f_ofset = speed300_exploration.slalom_R.f_ofset
			+ MAZE_OFFSET; //55;
	speed300_shortest.slalom_R.e_ofset = speed300_exploration.slalom_R.e_ofset;

	speed300_shortest.slalom_L.g_speed = speed300_exploration.slalom_L.g_speed;
	speed300_shortest.slalom_L.t_speed = speed300_exploration.slalom_L.t_speed;
	speed300_shortest.slalom_L.t_acc = speed300_exploration.slalom_L.t_acc;
	speed300_shortest.slalom_L.f_ofset = speed300_exploration.slalom_L.f_ofset
			+ MAZE_OFFSET; //50;
	speed300_shortest.slalom_L.e_ofset = speed300_exploration.slalom_L.e_ofset;

	speed300_shortest.turn90_R.g_speed = speed300_shortest.TurnCentervelocity;
	speed300_shortest.turn90_R.t_speed = 600;
	speed300_shortest.turn90_R.t_acc = 10000;
	speed300_shortest.turn90_R.f_ofset = 76;
	speed300_shortest.turn90_R.e_ofset = 107;

	speed300_shortest.turn90_L.g_speed = speed300_shortest.TurnCentervelocity;
	speed300_shortest.turn90_L.t_speed = 600;
	speed300_shortest.turn90_L.t_acc = 10000;
	speed300_shortest.turn90_L.f_ofset = 77;
	speed300_shortest.turn90_L.e_ofset = 103;

	speed300_shortest.turn180_R.g_speed = speed300_shortest.TurnCentervelocity;
	speed300_shortest.turn180_R.t_speed = 430;
	speed300_shortest.turn180_R.t_acc = 8000;
	speed300_shortest.turn180_R.f_ofset = 65;
	speed300_shortest.turn180_R.e_ofset = 80;

	speed300_shortest.turn180_L.g_speed = speed300_shortest.TurnCentervelocity;
	speed300_shortest.turn180_L.t_speed = 450;
	speed300_shortest.turn180_L.t_acc = 8000;
	speed300_shortest.turn180_L.f_ofset = 60;
	speed300_shortest.turn180_L.e_ofset = 71;

	speed300_shortest.turn45in_R.g_speed = speed300_shortest.TurnCentervelocity;
	speed300_shortest.turn45in_R.t_speed = 600;
	speed300_shortest.turn45in_R.t_acc = 10000;
	speed300_shortest.turn45in_R.f_ofset = 27;
	speed300_shortest.turn45in_R.e_ofset = 89;

	speed300_shortest.turn45in_L.g_speed = speed300_shortest.TurnCentervelocity;
	speed300_shortest.turn45in_L.t_speed = 600;
	speed300_shortest.turn45in_L.t_acc = 10000;
	speed300_shortest.turn45in_L.f_ofset = 33;
	speed300_shortest.turn45in_L.e_ofset = 90;

	speed300_shortest.turn135in_R.g_speed =
			speed300_shortest.TurnCentervelocity;
	speed300_shortest.turn135in_R.t_speed = 610;
	speed300_shortest.turn135in_R.t_acc = 8000;
	speed300_shortest.turn135in_R.f_ofset = 67;
	speed300_shortest.turn135in_R.e_ofset = 73;

	speed300_shortest.turn135in_L.g_speed =
			speed300_shortest.TurnCentervelocity;
	speed300_shortest.turn135in_L.t_speed = 610;
	speed300_shortest.turn135in_L.t_acc = 8000;
	speed300_shortest.turn135in_L.f_ofset = 67;
	speed300_shortest.turn135in_L.e_ofset = 73;

	speed300_shortest.turn45out_R.g_speed =
			speed300_shortest.TurnCentervelocity;
	speed300_shortest.turn45out_R.t_speed = 600;
	speed300_shortest.turn45out_R.t_acc = 10000;
	speed300_shortest.turn45out_R.f_ofset = 57;
	speed300_shortest.turn45out_R.e_ofset = 57;

	speed300_shortest.turn45out_L.g_speed =
			speed300_shortest.TurnCentervelocity;
	speed300_shortest.turn45out_L.t_speed = 600;
	speed300_shortest.turn45out_L.t_acc = 10000;
	speed300_shortest.turn45out_L.f_ofset = 64;
	speed300_shortest.turn45out_L.e_ofset = 52;

	speed300_shortest.turn135out_R.g_speed =
			speed300_shortest.TurnCentervelocity;
	speed300_shortest.turn135out_R.t_speed = 660;
	speed300_shortest.turn135out_R.t_acc = 8000;
	speed300_shortest.turn135out_R.f_ofset = 55;
	speed300_shortest.turn135out_R.e_ofset = 100;

	speed300_shortest.turn135out_L.g_speed =
			speed300_shortest.TurnCentervelocity;
	speed300_shortest.turn135out_L.t_speed = 660;
	speed300_shortest.turn135out_L.t_acc = 8000;
	speed300_shortest.turn135out_L.f_ofset = 55;
	speed300_shortest.turn135out_L.e_ofset = 103;

	speed300_shortest.V90_R.g_speed = speed300_shortest.TurnCentervelocity;
	speed300_shortest.V90_R.t_speed = 700;
	speed300_shortest.V90_R.t_acc = 11000;
	speed300_shortest.V90_R.f_ofset = 35;
	speed300_shortest.V90_R.e_ofset = 56;

	speed300_shortest.V90_L.g_speed = speed300_shortest.TurnCentervelocity;
	speed300_shortest.V90_L.t_speed = 700;
	speed300_shortest.V90_L.t_acc = 11000;
	speed300_shortest.V90_L.f_ofset = 32;
	speed300_shortest.V90_L.e_ofset = 60;



//********************************600********************************************
	speed600_shortest_mollifier.SlalomCentervelocity = 600;
	speed600_shortest_mollifier.TurnCentervelocity = 600;

	speed600_shortest_mollifier.slalom_R.g_speed =
			speed600_shortest_mollifier.SlalomCentervelocity;
	speed600_shortest_mollifier.slalom_R.t_speed = 1100;
	speed600_shortest_mollifier.slalom_R.t_acc = 60000;
	speed600_shortest_mollifier.slalom_R.f_ofset = 5;
	speed600_shortest_mollifier.slalom_R.e_ofset = 28;

	speed600_shortest_mollifier.slalom_L.g_speed =
			speed600_shortest_mollifier.SlalomCentervelocity;
	speed600_shortest_mollifier.slalom_L.t_speed = 1100;
	speed600_shortest_mollifier.slalom_L.t_acc = 60000;
	speed600_shortest_mollifier.slalom_L.f_ofset = 5;
	speed600_shortest_mollifier.slalom_L.e_ofset = 32;

	speed600_shortest_mollifier.turn90_R.g_speed = 600;
	speed600_shortest_mollifier.turn90_R.t_speed = 950;
	speed600_shortest_mollifier.turn90_R.t_acc = 15000;
	speed600_shortest_mollifier.turn90_R.f_ofset = 31;
	speed600_shortest_mollifier.turn90_R.e_ofset = 31;

	speed600_shortest_mollifier.turn90_L.g_speed = 600;
	speed600_shortest_mollifier.turn90_L.t_speed = 950;
	speed600_shortest_mollifier.turn90_L.t_acc = 15000;
	speed600_shortest_mollifier.turn90_L.f_ofset = 31;
	speed600_shortest_mollifier.turn90_L.e_ofset = 35;

	speed600_shortest_mollifier.turn180_R.g_speed =600;
	speed600_shortest_mollifier.turn180_R.t_speed = 840;
	speed600_shortest_mollifier.turn180_R.t_acc = 14000;
	speed600_shortest_mollifier.turn180_R.f_ofset = 7;
	speed600_shortest_mollifier.turn180_R.e_ofset = 20;

	speed600_shortest_mollifier.turn180_L.g_speed = 600;
	speed600_shortest_mollifier.turn180_L.t_speed = 820;
	speed600_shortest_mollifier.turn180_L.t_acc = 14000;
	speed600_shortest_mollifier.turn180_L.f_ofset = 7;
	speed600_shortest_mollifier.turn180_L.e_ofset = 18;

	speed600_shortest_mollifier.turn45in_R.g_speed = 600;
	speed600_shortest_mollifier.turn45in_R.t_speed = 830;
	speed600_shortest_mollifier.turn45in_R.t_acc = 12000;
	speed600_shortest_mollifier.turn45in_R.f_ofset = 13;
	speed600_shortest_mollifier.turn45in_R.e_ofset = 38;

	speed600_shortest_mollifier.turn45in_L.g_speed = 600;
	speed600_shortest_mollifier.turn45in_L.t_speed = 898;
	speed600_shortest_mollifier.turn45in_L.t_acc = 12000;
	speed600_shortest_mollifier.turn45in_L.f_ofset = 12;
	speed600_shortest_mollifier.turn45in_L.e_ofset = 36;

	speed600_shortest_mollifier.turn135in_R.g_speed = 600;
	speed600_shortest_mollifier.turn135in_R.t_speed = 940;
	speed600_shortest_mollifier.turn135in_R.t_acc = 14000;
	speed600_shortest_mollifier.turn135in_R.f_ofset = 9.5;
	speed600_shortest_mollifier.turn135in_R.e_ofset = 36;

	speed600_shortest_mollifier.turn135in_L.g_speed = 600;
	speed600_shortest_mollifier.turn135in_L.t_speed = 900;
	speed600_shortest_mollifier.turn135in_L.t_acc = 14000;
	speed600_shortest_mollifier.turn135in_L.f_ofset = 8;
	speed600_shortest_mollifier.turn135in_L.e_ofset = 34;

	speed600_shortest_mollifier.turn45out_R.g_speed = 600;
	speed600_shortest_mollifier.turn45out_R.t_speed = 720;
	speed600_shortest_mollifier.turn45out_R.t_acc = 12000;
	speed600_shortest_mollifier.turn45out_R.f_ofset = 43;
	speed600_shortest_mollifier.turn45out_R.e_ofset = 22;

	speed600_shortest_mollifier.turn45out_L.g_speed = 600;
	speed600_shortest_mollifier.turn45out_L.t_speed = 750;
	speed600_shortest_mollifier.turn45out_L.t_acc = 12000;
	speed600_shortest_mollifier.turn45out_L.f_ofset = 40;
	speed600_shortest_mollifier.turn45out_L.e_ofset = 24;

	speed600_shortest_mollifier.turn135out_R.g_speed = 600;
	speed600_shortest_mollifier.turn135out_R.t_speed = 920;
	speed600_shortest_mollifier.turn135out_R.t_acc = 15000;
	speed600_shortest_mollifier.turn135out_R.f_ofset = 5;
	speed600_shortest_mollifier.turn135out_R.e_ofset = 22;

	speed600_shortest_mollifier.turn135out_L.g_speed = 600;
	speed600_shortest_mollifier.turn135out_L.t_speed = 890;
	speed600_shortest_mollifier.turn135out_L.t_acc = 14000;
	speed600_shortest_mollifier.turn135out_L.f_ofset = 8;
	speed600_shortest_mollifier.turn135out_L.e_ofset = 36;

	speed600_shortest_mollifier.V90_R.g_speed = 600;
	speed600_shortest_mollifier.V90_R.t_speed = 940;
	speed600_shortest_mollifier.V90_R.t_acc = 15000;
	speed600_shortest_mollifier.V90_R.f_ofset = 17;
	speed600_shortest_mollifier.V90_R.e_ofset = 20;

	speed600_shortest_mollifier.V90_L.g_speed = 600;
	speed600_shortest_mollifier.V90_L.t_speed = 960;
	speed600_shortest_mollifier.V90_L.t_acc = 16000;
	speed600_shortest_mollifier.V90_L.f_ofset = 10;
	speed600_shortest_mollifier.V90_L.e_ofset = 20;




	//********************************1000********************************************
		speed1000_shortest_mollifier.SlalomCentervelocity = 1000;
		speed1000_shortest_mollifier.TurnCentervelocity = 1000;

		speed1000_shortest_mollifier.slalom_R.g_speed =
				speed1000_shortest_mollifier.SlalomCentervelocity;
		speed1000_shortest_mollifier.slalom_R.t_speed = 1100;
		speed1000_shortest_mollifier.slalom_R.f_ofset = 5;
		speed1000_shortest_mollifier.slalom_R.e_ofset = 28;

		speed1000_shortest_mollifier.slalom_L.g_speed =
				speed1000_shortest_mollifier.SlalomCentervelocity;
		speed1000_shortest_mollifier.slalom_L.t_speed = 1100;
		speed1000_shortest_mollifier.slalom_L.f_ofset = 5;
		speed1000_shortest_mollifier.slalom_L.e_ofset = 32;

		speed1000_shortest_mollifier.turn90_R.g_speed = 1000;
		speed1000_shortest_mollifier.turn90_R.t_speed = 1450;
		speed1000_shortest_mollifier.turn90_R.f_ofset = 26;
		speed1000_shortest_mollifier.turn90_R.e_ofset = 31;

		speed1000_shortest_mollifier.turn90_L.g_speed = 1000;
		speed1000_shortest_mollifier.turn90_L.t_speed = 1450;
		speed1000_shortest_mollifier.turn90_L.f_ofset = 32;
		speed1000_shortest_mollifier.turn90_L.e_ofset = 30;

		speed1000_shortest_mollifier.turn180_R.g_speed =1000;
		speed1000_shortest_mollifier.turn180_R.t_speed = 1490;
		speed1000_shortest_mollifier.turn180_R.f_ofset = 17;
		speed1000_shortest_mollifier.turn180_R.e_ofset = 20;

		speed1000_shortest_mollifier.turn180_L.g_speed = 1000;
		speed1000_shortest_mollifier.turn180_L.t_speed = 1490;
		speed1000_shortest_mollifier.turn180_L.f_ofset = 17;
		speed1000_shortest_mollifier.turn180_L.e_ofset = 18;

		speed1000_shortest_mollifier.turn45in_R.g_speed = 1000;
		speed1000_shortest_mollifier.turn45in_R.t_speed = 1300;
		speed1000_shortest_mollifier.turn45in_R.f_ofset = 12;
		speed1000_shortest_mollifier.turn45in_R.e_ofset = 38;

		speed1000_shortest_mollifier.turn45in_L.g_speed = 1000;
		speed1000_shortest_mollifier.turn45in_L.t_speed = 1300;
		speed1000_shortest_mollifier.turn45in_L.f_ofset = 26;
		speed1000_shortest_mollifier.turn45in_L.e_ofset = 36;

		speed1000_shortest_mollifier.turn135in_R.g_speed = 1000;
		speed1000_shortest_mollifier.turn135in_R.t_speed = 1590;
		speed1000_shortest_mollifier.turn135in_R.f_ofset = 7;
		speed1000_shortest_mollifier.turn135in_R.e_ofset = 15;

		speed1000_shortest_mollifier.turn135in_L.g_speed = 1000;
		speed1000_shortest_mollifier.turn135in_L.t_speed = 1560;
		speed1000_shortest_mollifier.turn135in_L.f_ofset = 5;
		speed1000_shortest_mollifier.turn135in_L.e_ofset = 12;

		speed1000_shortest_mollifier.turn45out_R.g_speed = 1000;
		speed1000_shortest_mollifier.turn45out_R.t_speed = 1300;
		speed1000_shortest_mollifier.turn45out_R.f_ofset = 29;
		speed1000_shortest_mollifier.turn45out_R.e_ofset = 22;

		speed1000_shortest_mollifier.turn45out_L.g_speed = 1000;
		speed1000_shortest_mollifier.turn45out_L.t_speed = 1300;
		speed1000_shortest_mollifier.turn45out_L.f_ofset = 41;
		speed1000_shortest_mollifier.turn45out_L.e_ofset = 24;

		speed1000_shortest_mollifier.turn135out_R.g_speed = 1000;
		speed1000_shortest_mollifier.turn135out_R.t_speed = 1660;
		speed1000_shortest_mollifier.turn135out_R.f_ofset = 17;
		speed1000_shortest_mollifier.turn135out_R.e_ofset = 22;

		speed1000_shortest_mollifier.turn135out_L.g_speed = 1000;
		speed1000_shortest_mollifier.turn135out_L.t_speed = 1660;
		speed1000_shortest_mollifier.turn135out_L.f_ofset = 11;
		speed1000_shortest_mollifier.turn135out_L.e_ofset = 30;

		speed1000_shortest_mollifier.V90_R.g_speed = 1000;
		speed1000_shortest_mollifier.V90_R.t_speed = 1750;
		speed1000_shortest_mollifier.V90_R.f_ofset = 12;
		speed1000_shortest_mollifier.V90_R.e_ofset = 23;

		speed1000_shortest_mollifier.V90_L.g_speed = 1000;
		speed1000_shortest_mollifier.V90_L.t_speed = 1750;
		speed1000_shortest_mollifier.V90_L.f_ofset = 14;
		speed1000_shortest_mollifier.V90_L.e_ofset = 23;


		//********************************1200********************************************
			speed1200_shortest_mollifier.SlalomCentervelocity = 1200;
			speed1200_shortest_mollifier.TurnCentervelocity = 1200;

			speed1200_shortest_mollifier.slalom_R.g_speed =
					speed1200_shortest_mollifier.SlalomCentervelocity;
			speed1200_shortest_mollifier.slalom_R.t_speed = 1100;
			speed1200_shortest_mollifier.slalom_R.f_ofset = 5;
			speed1200_shortest_mollifier.slalom_R.e_ofset = 28;

			speed1200_shortest_mollifier.slalom_L.g_speed =
					speed1200_shortest_mollifier.SlalomCentervelocity;
			speed1200_shortest_mollifier.slalom_L.t_speed = 1100;
			speed1200_shortest_mollifier.slalom_L.f_ofset = 5;
			speed1200_shortest_mollifier.slalom_L.e_ofset = 32;

			speed1200_shortest_mollifier.turn90_R.g_speed = 1200;
			speed1200_shortest_mollifier.turn90_R.t_speed = 1510;
			speed1200_shortest_mollifier.turn90_R.f_ofset = 21;
			speed1200_shortest_mollifier.turn90_R.e_ofset = 20;

			speed1200_shortest_mollifier.turn90_L.g_speed = 1200;
			speed1200_shortest_mollifier.turn90_L.t_speed = 1510;
			speed1200_shortest_mollifier.turn90_L.f_ofset = 22;
			speed1200_shortest_mollifier.turn90_L.e_ofset = 25;

			speed1200_shortest_mollifier.turn180_R.g_speed =1200;
			speed1200_shortest_mollifier.turn180_R.t_speed = 1650;
			speed1200_shortest_mollifier.turn180_R.f_ofset = 17;
			speed1200_shortest_mollifier.turn180_R.e_ofset = 20;

			speed1200_shortest_mollifier.turn180_L.g_speed = 1200;
			speed1200_shortest_mollifier.turn180_L.t_speed = 1650;
			speed1200_shortest_mollifier.turn180_L.f_ofset = 15;
			speed1200_shortest_mollifier.turn180_L.e_ofset = 8;

			speed1200_shortest_mollifier.turn45in_R.g_speed = 1200;
			speed1200_shortest_mollifier.turn45in_R.t_speed = 1550;
			speed1200_shortest_mollifier.turn45in_R.f_ofset = 16;
			speed1200_shortest_mollifier.turn45in_R.e_ofset = 39;

			speed1200_shortest_mollifier.turn45in_L.g_speed = 1200;
			speed1200_shortest_mollifier.turn45in_L.t_speed = 1550;
			speed1200_shortest_mollifier.turn45in_L.f_ofset = 20;
			speed1200_shortest_mollifier.turn45in_L.e_ofset = 39;

			speed1200_shortest_mollifier.turn135in_R.g_speed = 1200;
			speed1200_shortest_mollifier.turn135in_R.t_speed = 1850;
			speed1200_shortest_mollifier.turn135in_R.f_ofset = 14;
			speed1200_shortest_mollifier.turn135in_R.e_ofset = 22;

			speed1200_shortest_mollifier.turn135in_L.g_speed = 1200;
			speed1200_shortest_mollifier.turn135in_L.t_speed = 1850;
			speed1200_shortest_mollifier.turn135in_L.f_ofset = 15;
			speed1200_shortest_mollifier.turn135in_L.e_ofset = 20;

			speed1200_shortest_mollifier.turn45out_R.g_speed = 1200;
			speed1200_shortest_mollifier.turn45out_R.t_speed = 1400;
			speed1200_shortest_mollifier.turn45out_R.f_ofset = 33;
			speed1200_shortest_mollifier.turn45out_R.e_ofset = 16;

			speed1200_shortest_mollifier.turn45out_L.g_speed = 1200;
			speed1200_shortest_mollifier.turn45out_L.t_speed = 1400;
			speed1200_shortest_mollifier.turn45out_L.f_ofset = 30;
			speed1200_shortest_mollifier.turn45out_L.e_ofset = 16;

			speed1200_shortest_mollifier.turn135out_R.g_speed = 1200;
			speed1200_shortest_mollifier.turn135out_R.t_speed = 1800;
			speed1200_shortest_mollifier.turn135out_R.f_ofset = 7;
			speed1200_shortest_mollifier.turn135out_R.e_ofset = 30;

			speed1200_shortest_mollifier.turn135out_L.g_speed = 1200;
			speed1200_shortest_mollifier.turn135out_L.t_speed = 1850;
			speed1200_shortest_mollifier.turn135out_L.f_ofset = 3;
			speed1200_shortest_mollifier.turn135out_L.e_ofset = 30;

			speed1200_shortest_mollifier.V90_R.g_speed = 1200;
			speed1200_shortest_mollifier.V90_R.t_speed = 1850;
			speed1200_shortest_mollifier.V90_R.f_ofset = 7;
			speed1200_shortest_mollifier.V90_R.e_ofset = 17;

			speed1200_shortest_mollifier.V90_L.g_speed = 1200;
			speed1200_shortest_mollifier.V90_L.t_speed = 1850;
			speed1200_shortest_mollifier.V90_L.f_ofset = 3;
			speed1200_shortest_mollifier.V90_L.e_ofset = 16;


			//********************************1400********************************************
				speed1400_shortest_mollifier.SlalomCentervelocity = 1400;
				speed1400_shortest_mollifier.TurnCentervelocity = 1400;

				speed1400_shortest_mollifier.slalom_R.g_speed =
						speed1400_shortest_mollifier.SlalomCentervelocity;
				speed1400_shortest_mollifier.slalom_R.t_speed = 1100;
				speed1400_shortest_mollifier.slalom_R.f_ofset = 5;
				speed1400_shortest_mollifier.slalom_R.e_ofset = 28;

				speed1400_shortest_mollifier.slalom_L.g_speed =
						speed1400_shortest_mollifier.SlalomCentervelocity;
				speed1400_shortest_mollifier.slalom_L.t_speed = 1100;
				speed1400_shortest_mollifier.slalom_L.f_ofset = 5;
				speed1400_shortest_mollifier.slalom_L.e_ofset = 32;

				speed1400_shortest_mollifier.turn90_R.g_speed = 1400;
				speed1400_shortest_mollifier.turn90_R.t_speed = 1650;
				speed1400_shortest_mollifier.turn90_R.f_ofset = 13;
				speed1400_shortest_mollifier.turn90_R.e_ofset = 24;

				speed1400_shortest_mollifier.turn90_L.g_speed = 1400;
				speed1400_shortest_mollifier.turn90_L.t_speed = 1650;
				speed1400_shortest_mollifier.turn90_L.f_ofset = 14;
				speed1400_shortest_mollifier.turn90_L.e_ofset = 23;

				speed1400_shortest_mollifier.turn180_R.g_speed =1400;
				speed1400_shortest_mollifier.turn180_R.t_speed = 1860;
				speed1400_shortest_mollifier.turn180_R.f_ofset = 8;
				speed1400_shortest_mollifier.turn180_R.e_ofset = 22;

				speed1400_shortest_mollifier.turn180_L.g_speed = 1400;
				speed1400_shortest_mollifier.turn180_L.t_speed = 1920;
				speed1400_shortest_mollifier.turn180_L.f_ofset = 8;
				speed1400_shortest_mollifier.turn180_L.e_ofset = 19;

				speed1400_shortest_mollifier.turn45in_R.g_speed = 1400;
				speed1400_shortest_mollifier.turn45in_R.t_speed = 1800;
				speed1400_shortest_mollifier.turn45in_R.f_ofset = 7;
				speed1400_shortest_mollifier.turn45in_R.e_ofset = 49;

				speed1400_shortest_mollifier.turn45in_L.g_speed = 1400;
				speed1400_shortest_mollifier.turn45in_L.t_speed = 1800;
				speed1400_shortest_mollifier.turn45in_L.f_ofset = 9;
				speed1400_shortest_mollifier.turn45in_L.e_ofset = 52;

				speed1400_shortest_mollifier.turn135in_R.g_speed = 1400;
				speed1400_shortest_mollifier.turn135in_R.t_speed = 2310;
				speed1400_shortest_mollifier.turn135in_R.f_ofset = 18;
				speed1400_shortest_mollifier.turn135in_R.e_ofset = 40;

				speed1400_shortest_mollifier.turn135in_L.g_speed = 1400;
				speed1400_shortest_mollifier.turn135in_L.t_speed = 2310;
				speed1400_shortest_mollifier.turn135in_L.f_ofset = 16;
				speed1400_shortest_mollifier.turn135in_L.e_ofset = 36;

				speed1400_shortest_mollifier.turn45out_R.g_speed = 1400;
				speed1400_shortest_mollifier.turn45out_R.t_speed = 1700;
				speed1400_shortest_mollifier.turn45out_R.f_ofset = 26;
				speed1400_shortest_mollifier.turn45out_R.e_ofset = 30;

				speed1400_shortest_mollifier.turn45out_L.g_speed = 1400;
				speed1400_shortest_mollifier.turn45out_L.t_speed = 1700;
				speed1400_shortest_mollifier.turn45out_L.f_ofset = 27;
				speed1400_shortest_mollifier.turn45out_L.e_ofset = 27;

				speed1400_shortest_mollifier.turn135out_R.g_speed = 1400;
				speed1400_shortest_mollifier.turn135out_R.t_speed = 2210;
				speed1400_shortest_mollifier.turn135out_R.f_ofset = 10;
				speed1400_shortest_mollifier.turn135out_R.e_ofset = 38;

				speed1400_shortest_mollifier.turn135out_L.g_speed = 1400;
				speed1400_shortest_mollifier.turn135out_L.t_speed = 2210;
				speed1400_shortest_mollifier.turn135out_L.f_ofset = 8;
				speed1400_shortest_mollifier.turn135out_L.e_ofset = 33;

				speed1400_shortest_mollifier.V90_R.g_speed = 1300;
				speed1400_shortest_mollifier.V90_R.t_speed = 2200;
				speed1400_shortest_mollifier.V90_R.f_ofset = 13;
				speed1400_shortest_mollifier.V90_R.e_ofset = 37;

				speed1400_shortest_mollifier.V90_L.g_speed = 1300;
				speed1400_shortest_mollifier.V90_L.t_speed = 2200;
				speed1400_shortest_mollifier.V90_L.f_ofset = 11;
				speed1400_shortest_mollifier.V90_L.e_ofset = 35;

//********************************1600********************************************
			//********************************1400********************************************
				speed1600_shortest_mollifier.SlalomCentervelocity = 1400;
				speed1600_shortest_mollifier.TurnCentervelocity = 1400;

				speed1600_shortest_mollifier.slalom_R.g_speed =
						speed1600_shortest_mollifier.SlalomCentervelocity;
				speed1600_shortest_mollifier.slalom_R.t_speed = 1100;
				speed1600_shortest_mollifier.slalom_R.f_ofset = 5;
				speed1600_shortest_mollifier.slalom_R.e_ofset = 28;

				speed1600_shortest_mollifier.slalom_L.g_speed =
						speed1600_shortest_mollifier.SlalomCentervelocity;
				speed1600_shortest_mollifier.slalom_L.t_speed = 1100;
				speed1600_shortest_mollifier.slalom_L.f_ofset = 5;
				speed1600_shortest_mollifier.slalom_L.e_ofset = 32;

				speed1600_shortest_mollifier.turn90_R.g_speed = 1600;
				speed1600_shortest_mollifier.turn90_R.t_speed = 2000;
				speed1600_shortest_mollifier.turn90_R.f_ofset = 11;
				speed1600_shortest_mollifier.turn90_R.e_ofset = 22;

				speed1600_shortest_mollifier.turn90_L.g_speed = 1600;
				speed1600_shortest_mollifier.turn90_L.t_speed = 2000;
				speed1600_shortest_mollifier.turn90_L.f_ofset = 10;
				speed1600_shortest_mollifier.turn90_L.e_ofset = 24;

				speed1600_shortest_mollifier.turn180_R.g_speed =1400;
				speed1600_shortest_mollifier.turn180_R.t_speed = 1890;
				speed1600_shortest_mollifier.turn180_R.f_ofset = 8;
				speed1600_shortest_mollifier.turn180_R.e_ofset = 8;

				speed1600_shortest_mollifier.turn180_L.g_speed = 1400;
				speed1600_shortest_mollifier.turn180_L.t_speed = 1900;
				speed1600_shortest_mollifier.turn180_L.f_ofset = 8;
				speed1600_shortest_mollifier.turn180_L.e_ofset = 6;

				speed1600_shortest_mollifier.turn45in_R.g_speed = 1400;
				speed1600_shortest_mollifier.turn45in_R.t_speed = 1800;
				speed1600_shortest_mollifier.turn45in_R.f_ofset = 8;
				speed1600_shortest_mollifier.turn45in_R.e_ofset = 61;

				speed1600_shortest_mollifier.turn45in_L.g_speed = 1400;
				speed1600_shortest_mollifier.turn45in_L.t_speed = 1800;
				speed1600_shortest_mollifier.turn45in_L.f_ofset = 6;
				speed1600_shortest_mollifier.turn45in_L.e_ofset = 60;

				speed1600_shortest_mollifier.turn135in_R.g_speed = 1400;
				speed1600_shortest_mollifier.turn135in_R.t_speed = 2150;
				speed1600_shortest_mollifier.turn135in_R.f_ofset = 10;
				speed1600_shortest_mollifier.turn135in_R.e_ofset = 25;

				speed1600_shortest_mollifier.turn135in_L.g_speed = 1400;
				speed1600_shortest_mollifier.turn135in_L.t_speed = 2150;
				speed1600_shortest_mollifier.turn135in_L.f_ofset = 10;
				speed1600_shortest_mollifier.turn135in_L.e_ofset = 22;

				speed1600_shortest_mollifier.turn45out_R.g_speed = 1400;
				speed1600_shortest_mollifier.turn45out_R.t_speed = 1700;
				speed1600_shortest_mollifier.turn45out_R.f_ofset = 27;
				speed1600_shortest_mollifier.turn45out_R.e_ofset = 30;

				speed1600_shortest_mollifier.turn45out_L.g_speed = 1400;
				speed1600_shortest_mollifier.turn45out_L.t_speed = 1700;
				speed1600_shortest_mollifier.turn45out_L.f_ofset = 27;
				speed1600_shortest_mollifier.turn45out_L.e_ofset = 33;

				speed1600_shortest_mollifier.turn135out_R.g_speed = 1400;
				speed1600_shortest_mollifier.turn135out_R.t_speed = 2150;
				speed1600_shortest_mollifier.turn135out_R.f_ofset = 14;
				speed1600_shortest_mollifier.turn135out_R.e_ofset = 35;

				speed1600_shortest_mollifier.turn135out_L.g_speed = 1400;
				speed1600_shortest_mollifier.turn135out_L.t_speed = 2150;
				speed1600_shortest_mollifier.turn135out_L.f_ofset = 14;
				speed1600_shortest_mollifier.turn135out_L.e_ofset = 35;

				speed1600_shortest_mollifier.V90_R.g_speed = 1430;
				speed1600_shortest_mollifier.V90_R.t_speed = 2700;
				speed1600_shortest_mollifier.V90_R.f_ofset = 15;
				speed1600_shortest_mollifier.V90_R.e_ofset = 37;

				speed1600_shortest_mollifier.V90_L.g_speed = 1430;
				speed1600_shortest_mollifier.V90_L.t_speed = 2700;
				speed1600_shortest_mollifier.V90_L.f_ofset = 12;
				speed1600_shortest_mollifier.V90_L.e_ofset = 35;


}
