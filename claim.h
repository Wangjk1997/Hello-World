#pragma once

/*
#ifndef CLAIM_H_
#define CLAIM_H_*/

#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>


const double PI = 3.14159265358979323846;
const int dimension = 3;

//those parameters below can be changed according to the real problem
const int num_plane = 10;
const double Kp_p = 0.5;// proportion parameter of position
const double Ki_p = 0.0001;// integrate parameter of position
const double Kd_p = 0.01;// derivative parameter of position
const double Kp_v = 0.5;// proportion parameter of velocity
const double Ki_v = 0.0001;// integrate parameter of velocity
const double Kd_v = 0.01;// derivative parameter of velocity
const double square_length	= 70;//horizontal 
const double square_width	= 70;//vertical
const double line_length = 20;//horizontal		
const double line_width = 100;//vertical
const double circle_length = 50;//horizontal
const double circle_width = 50;//vertical
const double ellipse_length = 60;//horizontal
const double ellipse_width = 40;//vertical
const double sample_time = 0.05;
const double highlimit_accelerate = 10;
const double lowlimit_accelerate = -10;


void init_leader(struct condition leader);
void init_follower(struct condition leader , struct condition follower[]);
void set_exp_position(struct condition leader, struct condition follower[], int mode);
void cal_exp_position (struct condition leader, struct condition follower[],int if_horizon, double length, double width, char s_or_c);
void cal_exp_velocity(struct condition follower[]);
void cal_exp_accelerate_velocity(struct condition follower[]);
double square(double length, double width, double theta, char axis);
double circle(double length, double width, double theta, char axis);
void update_position_leader(struct condition leader, char mode, double time);
void update_velocity_leader(struct condition leader, double time);
void update_position_follower(struct condition follower[]);
void update_velocity_follower(struct condition follower[]);
double position_function_leader(double time, char axis);
double velocity_function_leader(double time, char axis);

struct condition 
{
	double a[dimension];
	double v[2*dimension];//First three parameter means cal valuel ;4-6 means actual value 
	double pos[2*dimension];//First three parameter means cal value; 4-6 means actual value
	double error_p[3][dimension];//error of position,the last one means sum of error; 
	double error_v[3][dimension];//error of velcoity, the last one means sum of error;
};


//#endif // !CLAIM_H_

