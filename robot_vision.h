#ifndef _robot_vision_
#define _robot_vision_

#include "robot_if.h"
#include "robot_color.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define area_threshold 0.75
#define multiplier 500

typedef enum _square_state_ {
	hasTwoPair,
	hasOnePair,
	twoLargest,
	onlyLargest,
	noneFound
} square_state;

void center_robot(robot_if_t *ri, IplImage *image, IplImage *final_threshold, char *bot_name);

#endif