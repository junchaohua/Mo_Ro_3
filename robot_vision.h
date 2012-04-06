#ifndef _robot_vision_
#define _robot_vision_

#include "robot_if.h"
#include "robot_color.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define AREA_THRESHOLD 0.8
#define multiplier 500

typedef enum _square_state_ {
	hasTwoPair,
	hasOnePair,
	twoLargest,
	onlyLargest,
	noneFound
} square_state;

<<<<<<< HEAD
=======

>>>>>>> ed58e4ce1c5af96d2b6e5ae5ed9e52ff1f2e19bc
void center_robot(robot_if_t *ri, IplImage *image, IplImage *final_threshold, char *bot_name);

#endif