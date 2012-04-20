#ifndef _path_planning_
#define _path_planning_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "robot_if.h" 

typedef enum{
	HEADING_UP, 
	HEADING_DOWN, 
	HEADING_RIGHT, 
	HEADING_LEFT
} robot_heading;

typedef enum{
	MV_FWD, 
	TN_LEFT_90, 
	TN_RGHT_90, 
	CENTER_TWO_PAIR, 
	CENTER_ONE_PAIR
} move_command;

typedef struct _array_map_obj_t_{
	int type;
	int points;
} array_map_obj_t;

int getRow();
int getCol();
int pairsToExpect(robot_heading heading, robot_heading direction_to_move);
int isObstructed(array_map_obj_t *cell);
void makeAMove(robot_if_t *ri);
robot_heading whereToGo(robot_if_t *ri);
void updateMap(array_map_obj_t *map, robot_if_t *ri, int *score1, int *score2);

#endif