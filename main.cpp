/****************************************************文档说明************************************************
 * Copyright(C), 2017, BUAA, 软件与控制研究室
 * 文件名称:    Formation_change.sln
 * 作者:        Junkai Wang 
 * 版本:        1.0        
 * 创建日期:    2017.10.20, 11:00
 * 完成日期:    2017.10.21
 * 文件描述:    通过PID控制器进行无人机队形控制，并进行仿真。
 *              
 * 调用关系:    无
 * 其它:        无
 * 函数列表:    init_leader							进行leader的初始化
				init_follower						进行follower的初始化		
				set_exp_position					设定期望的队形模式
				cal_exp_position					计算follower所期望到达的位置
				cal_exp_velocity					计算follower所期望的速度
				cal_exp_accelerate_velocity			计算follower所期望的加速度
				square								通过极坐标系的极角计算矩形在直角坐标系中的位置
				circle								通过极坐标系的极角计算圆形（椭圆）在直角坐标系中的位置
				update_position_leader				更新leader的位置信息
				update_velocity_leader				更新leader的速度信息
				update_position_follower			更新follower的位置信息
				update_velocity_follower			更新follower的速度信息
				position_function_leader			定义leader的轨迹方程
				velocity_function_leader			定义leader的速度方程
				
 * 
 * 修改历史:
 * 1.   修改日期:   2017.10.21, 19:30
 *      修改人:     Yang Liu
 *      修改功能:   增加文本输出功能。
************************************************************************************************************/

#include "claim.h"

using namespace std;

struct condition leader, follower[num_plane];

int main()
{
	

	init_leader(leader);
	init_follower(leader, follower);

	int number = 2;
	int time = 0;

	fstream mfile("result.txt", ios::out);
	mfile << "Ax \t Ay \t Az \t Vx \t Vy \t Vz \t Posx \t PosY \t PosZ \t\n" << endl;

	while (time < 10000)
	{
		if (time % 100 == 0)
		{
			for (int i = 0; i < dimension; i++)
			{
				mfile << setw(5) << setprecision(2) << follower[number].a[i] << "\t";
			}
			//cout << "v" <<endl;
			for (int i = 0; i < 2 * dimension; i++)
			{
				mfile << setw(5) << setprecision(2) << follower[number].v[i] << "\t";
			}
			//cout << "pos" <<endl;
			for (int i = 0; i < 2 * dimension; i++)
			{
				mfile << setw(5) << setprecision(2) << follower[number].pos[i] << "\t";
			}
			mfile << endl;
		}
		update_position_leader(leader, 'f', time);
		set_exp_position(leader, follower, 4);
		cal_exp_velocity(follower);
		cal_exp_accelerate_velocity(follower);
		update_velocity_follower(follower);
		update_position_follower(follower);

		time++;

	}

	return 0;
}