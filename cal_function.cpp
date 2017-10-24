#include "claim.h"

void set_exp_position(struct condition leader, struct condition follower[], int mode)
{

	switch (mode)
	{
	case 0:
		cal_exp_position(leader, follower, 1, circle_length, circle_width, 'c');
		break;//大圆圈水平
	case 1:
		cal_exp_position(leader, follower, 0, circle_length, circle_width, 'c');
		break;//大圆圈竖直
	case 2:
		cal_exp_position(leader, follower, 1, ellipse_length, ellipse_width, 'c');
		break;//椭圆水平
	case 3:
		cal_exp_position(leader, follower, 0, ellipse_length, ellipse_width, 'c');
		break;//椭圆竖直
	case 4:
		cal_exp_position(leader, follower, 1, square_length, square_width, 's');
		break;//正方形水平
	case 5:
		cal_exp_position(leader, follower, 0, square_length, square_width, 's');
		break;//正方形竖直
	case 6:
		cal_exp_position(leader, follower, 1, line_length, line_width, 's');
		break;//竖排水平
	case 7:
		cal_exp_position(leader, follower, 1, line_width, line_length, 's');
		break;//横排水平
	}
}

void cal_exp_position(struct condition leader, struct condition follower[], int if_horizon, double length, double width, char s_or_c)
{
	int i;
	if (if_horizon == 1)
	{
		if (s_or_c == 's')
		{
			for (i = 0; i < num_plane; i++)
			{
				follower[i].pos[0] = leader.pos[0] + square(length, width, 2 * PI / num_plane *i, 'x');
				follower[i].pos[1] = leader.pos[1] + square(length, width, 2 * PI / num_plane *i, 'y');
				follower[i].pos[2] = leader.pos[2];
			}
		}
		else if (s_or_c == 'c')
		{
			for (i = 0; i < num_plane; i++)
			{
				follower[i].pos[0] = leader.pos[0] + circle(length, width, 2 * PI / num_plane *i, 'x');
				follower[i].pos[1] = leader.pos[1] + circle(length, width, 2 * PI / num_plane *i, 'y');
				follower[i].pos[2] = leader.pos[2];
			}
		}
	}
	else
	{
		if (s_or_c == 's')
		{
			for (i = 0; i < num_plane; i++)
			{
				follower[i].pos[0] = leader.pos[0] + square(length, width, 2 * PI / num_plane *i, 'x');
				follower[i].pos[1] = leader.pos[1];
				follower[i].pos[2] = leader.pos[2] + square(length, width, 2 * PI / num_plane *i, 'y');
			}
		}
		else if (s_or_c == 'c')
		{
			for (i = 0; i < num_plane; i++)
			{
				follower[i].pos[0] = leader.pos[0] + circle(length, width, 2 * PI / num_plane *i, 'x');
				follower[i].pos[1] = leader.pos[1];
				follower[i].pos[2] = leader.pos[2] + circle(length, width, 2 * PI / num_plane *i, 'y');
			}
		}
	}
}

void cal_exp_velocity(struct condition follower[])
{
	int i;
	for (i = 0; i < num_plane; i++)
	{
		for (int j = 0; j < dimension; j++)
		{
			follower[i].error_p[j][1] = follower[i].error_p[j][0];// the error of last time
			follower[i].error_p[j][0] = follower[i].pos[j] - follower[i].pos[j + 3];// the error at present 
			follower[i].error_p[j][2] += follower[i].error_p[j][0];// the sum of error

			follower[i].v[j] = Kp_p * follower[i].error_p[j][0] + Ki_p * follower[i].error_p[j][2]
				+ Kd_p * (follower[i].error_p[j][0] - follower[i].error_p[j][1]);
		}
	}
}

void cal_exp_accelerate_velocity(struct condition follower[])
{
	int i;
	for (i = 0; i < num_plane; i++)
	{
		for (int j = 0; j < dimension; j++)
		{
			follower[i].error_v[j][1] = follower[i].error_v[j][0];// the error of last time
			follower[i].error_v[j][0] = follower[i].v[j] - follower[i].v[j + 3];// the error at present 
			follower[i].error_v[j][2] += follower[i].error_v[j][0];// the sum of error

			follower[i].a[j] = Kp_v * follower[i].error_v[j][0] + Ki_v * follower[i].error_v[j][2]
				+ Kd_v * (follower[i].error_v[j][0] - follower[i].error_v[j][1]);
			if (follower[i].a[j] > highlimit_accelerate)
			{
				follower[i].a[j] = highlimit_accelerate;
			}
			else if(follower[i].a[j] < lowlimit_accelerate)
			{
				follower[i].a[j] = lowlimit_accelerate;
			}
		}
	}
}

double square(double length, double width, double theta, char axis)
{
	double angel[4];
	angel[0] = atan(width / length);
	angel[1] = PI - angel[0];
	angel[2] = PI + angel[0];
	angel[3] = 2 * PI - angel[0];
	if (axis == 'x')
	{
		if (((theta >= 0) && (theta < angel[0])) || ((theta >= angel[3]) && (theta <= 2 * PI)))
		{
			return (length / 2);
		}
		else if ((theta >= angel[0]) && (theta < angel[1]))
		{
			return (width / (2 * tan(theta)));
		}
		else if ((theta >= angel[1]) && (theta < angel[2]))
		{
			return (-length / 2);
		}
		else if ((theta >= angel[2]) && (theta < angel[3]))
		{
			return (-width / (2 * tan(theta)));
		}
	}
	if (axis == 'y')
	{
		if (((theta >= 0) && (theta < angel[0])) || ((theta >= angel[3]) && (theta <= 2 * PI)))
		{
			return (length / 2 * tan(theta));
		}
		else if ((theta >= angel[0]) && (theta < angel[1]))
		{
			return (width / 2);
		}
		else if ((theta >= angel[1]) && (theta < angel[2]))
		{
			return (-length / 2 * tan(theta));
		}
		else if ((theta >= angel[2]) && (theta < angel[3]))
		{
			return (-width / 2);
		}
	}
}

double circle(double length, double width, double theta, char axis)
{
	if (axis == 'x')
	{
		return width / 2 * cos(theta);
	}
	else if (axis == 'y')
	{
		return length / 2 * sin(theta);
	}
}

double position_function_leader(double time, char axis)
{
	double x, y, z;
	//the position funciton should be written here
	//using the parameter time
	x = 0;
	y = 0;
	z = 0;
	//end
	switch (axis)
	{

	case 'x':
		return x;
		break;
	case 'y':
		return y;
		break;
	case 'z':
		return z;
		break;
	default:
		break;
	}
}

double velocity_function_leader(double time, char axis)
{
	double x, y, z;
	x = y = z = 0;
	//the velocity funciton should be written here
	//using the parameter time

	//end
	switch (axis)
	{

	case 'x':
		return x;
		break;
	case 'y':
		return y;
		break;
	case 'z':
		return z;
		break;
	default:
		break;
	}
}