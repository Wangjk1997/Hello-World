/****************************************************�ĵ�˵��************************************************
 * Copyright(C), 2017, BUAA, ���������о���
 * �ļ�����:    Formation_change.sln
 * ����:        Junkai Wang 
 * �汾:        1.0        
 * ��������:    2017.10.20, 11:00
 * �������:    2017.10.21
 * �ļ�����:    ͨ��PID�������������˻����ο��ƣ������з��档
 *              
 * ���ù�ϵ:    ��
 * ����:        ��
 * �����б�:    init_leader							����leader�ĳ�ʼ��
				init_follower						����follower�ĳ�ʼ��		
				set_exp_position					�趨�����Ķ���ģʽ
				cal_exp_position					����follower�����������λ��
				cal_exp_velocity					����follower���������ٶ�
				cal_exp_accelerate_velocity			����follower�������ļ��ٶ�
				square								ͨ��������ϵ�ļ��Ǽ��������ֱ������ϵ�е�λ��
				circle								ͨ��������ϵ�ļ��Ǽ���Բ�Σ���Բ����ֱ������ϵ�е�λ��
				update_position_leader				����leader��λ����Ϣ
				update_velocity_leader				����leader���ٶ���Ϣ
				update_position_follower			����follower��λ����Ϣ
				update_velocity_follower			����follower���ٶ���Ϣ
				position_function_leader			����leader�Ĺ켣����
				velocity_function_leader			����leader���ٶȷ���
				
 * 
 * �޸���ʷ:
 * 1.   �޸�����:   2017.10.21, 19:30
 *      �޸���:     Yang Liu
 *      �޸Ĺ���:   �����ı�������ܡ�
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