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

typedef struct _array_map_obj_t_{
	int type;
	int points;
} array_map_obj_t;

int getRow();
int getCol();
int pairsToExpect(robot_heading heading, robot_heading direction_to_move);
int isObstructed(int cell_type);
void makeAMove(robot_if_t *ri);
robot_heading whereToGo(robot_if_t *ri);
void get_map_array(array_map_obj_t *map, robot_if_t *ri, int *score1, int *score2);

#endif