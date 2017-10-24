#include "claim.h"

void init_leader(struct condition leader)    //initialize the leader plane
{
	int i;
	for (i = 0; i < dimension; i++)
	{
		leader.a[i] = 0;
		for (int k = 0; k < 3; k++)
		{
			leader.error_p[i][k] = 0;
			leader.error_v[i][k] = 0;
		}
	}
	for (i = 0; i < 2 * dimension; i++)
	{
		leader.pos[i] = 0;
		leader.v[i] = 0;
	}
}

void init_follower(struct condition leader, struct condition follower[])//initialize the follower as the mode 0
{
	set_exp_position(leader, follower, 0);
	int j, i;
	for (j = 0; j < num_plane; j++)
	{
		for (i = 0; i < dimension; i++)
		{
			follower[j].a[i] = 0;
			follower[j].pos[dimension + i] = follower[j].pos[i];
			for (int k = 0; k < 3; k++)
			{
				follower[j].error_p[i][k] = 0;
				follower[j].error_v[i][k] = 0;
			}
		}
		for (i = 0; i < 2 * dimension; i++)
		{
			follower[j].v[i] = 0;
		}
	}
}
