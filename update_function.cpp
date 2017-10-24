#include "claim.h"

void update_position_leader(struct condition leader, char mode, double time)
{
	switch (mode)
	{
	case 'f':
		for (int i = 0; i < dimension; i++)
		{
			leader.pos[i + dimension] = position_function_leader(time, 'x' + i);
		}
		break;
	case 'v':
		update_velocity_leader(leader, time);
		for (int i = 0; i < dimension; i++)
		{
			leader.pos[i + dimension] = leader.pos[i + dimension] + leader.v[i + dimension] * sample_time;
		}
		break;
	default:
		break;
	}
}

void update_velocity_leader(struct condition leader, double time)
{
	for (int i = 0; i < dimension; i++)
	{
		leader.v[i + dimension] = velocity_function_leader(time, 'x' + i);
	}
}

void update_position_follower(struct condition follower[])
{
	for (int i = 0; i < num_plane; i++)
	{
		for (int j = 0; j < dimension; j++)
		{
			follower[i].pos[j + dimension] = follower[i].pos[j + dimension] + follower[i].v[j + dimension] * sample_time;
		}
	}
}

void update_velocity_follower(struct condition follower[])
{
	for (int i = 0; i < num_plane; i++)
	{
		for (int j = 0; j < dimension; j++)
		{
			follower[i].v[j + dimension] = follower[i].v[j + dimension] + follower[i].a[j] * sample_time;
		}
	}
}