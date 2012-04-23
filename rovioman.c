/*  
 * Filename: robot_assign2.c  
 * Authors: Tim Kostreva, Junchao Hua, Spencer Krause  
 * Date: 03-20-2012  
 * Purpose: use both image processing and room waypoints to guide the robot through the corrider
 */


#include <robot_if.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include "position.h"
#include "PID_Control.h"
#include "robot_vision.h"
#include "path_planning.h"

/* DEFINES */
#define F_Kp 1.0
#define F_Ki 0.1
#define F_Kd 0.01

#define R_Kp 10.0/M_PI
#define R_Ki 0.5
#define R_Kd 0.20

#define FWD_PID_TOLERANCE  15.0  /*How close should I be to the waypoint before moving onto the next one? */

#define ROWS 5
#define COLS 7
/* USE THIS DEFINE TO ACCESS MAP[i][j] 
 * ex:	array2D(map,i,j).type */
#define array2D(b,i,j) b[(i)*COLS+(j)]

/* GLOBALS */
int 		x,		//current position
			y,
			robotID,// Used to be MINE
			score1, 
			score2,
			firstRun,
			pointsInMap;
float		direction;
PID 		*fwdPID,
			*rotPID;
vector 		*loc,
			*vel;
array_map_obj_t *map;
robot_heading	facing;
IplImage	*image, 
			*final_threshold;

float fwd_speed[] = {  // Forward speeds in [cm/s]  (Halved in code to compensate for stop / lag in bot )
	33.94,
	33.94,
	32.5,
	32.5,
	31.6,
	31.6,
	22.2,
	22.2,
	20.2,
	20.2
};
	
float rot_speed[] = {  // Rotation Speeds in [rad/s]
	3.0,
	3.0,
	2.55,
	2.55,
	2.1,
	2.0,
	1.2,
	1.17,
	1.06,
	1.06
};

/* FUNCTIONS */
int fwdSpeedScaling(float PIDout) {
	float 	fwdScale;
	int	speed;
	
	fwdScale = fabs(PIDout);
		
	if (fwdScale >= 80.0) speed = 1;
	else if (fwdScale >= 60.0 && fwdScale < 80.0) speed = 3;
	else if (fwdScale >= 40.0 && fwdScale < 60.0) speed = 5;
	else if (fwdScale >= 20.0 && fwdScale < 40.0) speed = 7;
	else if (fwdScale < 20.0) speed = 9;
	
	if(PIDout < 0.0) speed *= -1;
	
	return speed;
}

int rotSpeedScaling(float PIDout) {
	float 	rotScale;
	int	speed;
	
	rotScale = fabs(PIDout);
	
	if (rotScale >= 10.0) speed = 4;
	else if (rotScale >= 9.0 && rotScale < 10.0) speed = 4;
	else if (rotScale >= 8.0 && rotScale < 9.0) speed = 5;
	else if (rotScale >= 7.0 && rotScale < 8.0) speed = 5;
	else if (rotScale >= 6.0 && rotScale < 7.0) speed = 5;
	else if (rotScale >= 5.0 && rotScale < 6.0) speed = 5;
	else if (rotScale >= 4.0 && rotScale < 5.0) speed = 6;
	else if (rotScale >= 3.0 && rotScale < 4.0) speed = 6;
	else if (rotScale >= 2.0 && rotScale < 3.0) speed = 6;
	else if (rotScale < 2.0) speed = 6;
	
	if(PIDout < 0.0) speed *= -1;
	
	return speed;
}

float get_euclidian_distance(float start_x, float start_y, float end_x, float end_y){
	float	diff1,
		diff2;
	diff1 = end_x - start_x;
	diff2 = end_y - start_y;
 	return sqrt( diff1 * diff1 + diff2 * diff2 );
}


float calcAngle(int p1_x, int p1_y, int p2_x, int p2_y) {
	
	return atan2( (float)(p2_y - p1_y), (float)(p2_x - p1_x) );
}

void rotate_to_theta(robot_if_t *ri, float target_theta, vector *final_pos){
	float	output,
		rot_amount;
	int  	ang_vel;		
	vector  *current_location,
		*expected_vel;
	
	/* reset PID control for this rotation */
	reset_PID(rotPID);
	
	current_location = (vector *)calloc(1, sizeof(vector));
	expected_vel = (vector *)calloc(1, sizeof(vector));
	
	/* initialize expected velocity vector */
	expected_vel->v[0] = 0.0;
	expected_vel->v[1] = 0.0;
	expected_vel->v[2] = 0.0;
	
	get_Position(ri, current_location, expected_vel, ROTATE);
	
	do {
		printf("\n *********************  ROT PID ENABLED  ********************\n\n");
		printf("Curr Theta = %f\tTarget Theta = %f\n", current_location->v[2], target_theta);
		output = Compute(rotPID, current_location->v[2], target_theta);
		printf("Rotate PID Output = %f\n", output);
		
		// correlate output to an angular velocity
		ang_vel = rotSpeedScaling(output);
		
		if(ang_vel > 0) {
			if(ang_vel < 6) ri_move(ri, RI_TURN_LEFT_20DEG, ang_vel);
			else ri_move(ri, RI_TURN_LEFT, ang_vel);
			
			if(fabs(output) < 2.0) ri_move(ri, RI_STOP, ang_vel);
						
			expected_vel->v[0] = 0.0;
			expected_vel->v[1] = 0.0;
			expected_vel->v[2] = rot_speed[ang_vel - 1] * 0.75;
		 }
		else {
			ang_vel *= -1;
			
			if(ang_vel < 6) ri_move(ri, RI_TURN_RIGHT_20DEG, ang_vel);
			else ri_move(ri, RI_TURN_RIGHT, ang_vel);
			
			if(fabs(output) < 2.0) ri_move(ri, RI_STOP, ang_vel);
						
			expected_vel->v[0] = 0.0;
			expected_vel->v[1] = 0.0;
			expected_vel->v[2] = -1.0 * rot_speed[ang_vel - 1] * 0.75;
		}
		
				 
		get_Position(ri, current_location, expected_vel, ROTATE);
		
		rot_amount = fabs(target_theta - current_location->v[2]);		
	} while (rot_amount > 0.15);
	
	ri_move(ri, RI_STOP, 1);
	
	final_pos->v[0] = current_location->v[0];
	final_pos->v[1] = current_location->v[1];
	final_pos->v[2] = current_location->v[2];
	
	free(current_location);
	free(expected_vel);
}

void go_to_position(robot_if_t *ri, IplImage *image,  float end_x, float end_y, vector *final_pos){
 	float	setpoint,
		x_i,
		y_i,
		current_distance,
		output,
		theta_target,
		theta_error,
		error;
	int	i,
		bot_speed;
	vector 	*current_location,
		*expected_vel;
		
	current_location = (vector *)calloc(1, sizeof(vector));
	expected_vel = (vector *)calloc(1, sizeof(vector));
 	
	/* reset PID control for this move */
	reset_PID(fwdPID);
		
 	//initialize start_x, start_y, start_theta
	get_Position(ri, current_location, expected_vel, FORWARD);
	x_i = current_location->v[0];
	y_i = current_location->v[1];
	
	// find initial theta to target in case we need to rotate immediately
	theta_target = calcAngle(x_i, y_i, end_x, end_y);
	
	// point robot at destination using PID 
	//if( fabs(theta_target - current_location->v[2]) > 0.15) rotate_to_theta(ri, theta_target);	
	  
	i = 10;
	// setpoint is the exact number of cm required to move to target.
	setpoint = get_euclidian_distance(current_location->v[0], current_location->v[1], end_x, end_y);
	
	do {
		current_distance = get_euclidian_distance(x_i, y_i, current_location->v[0], current_location->v[1]);
		error = setpoint - current_distance;
		
		theta_error = theta_target - current_location->v[2];
		
		// For the first 10 iterations of action loop, pull out slowly 
		if ( i > 0 ) {
			bot_speed = i;  
		}
		else bot_speed = RI_FASTEST;
		
		// At the FWD PID Setpoint, turn on PID control to reduce speed and stop.
		if( error <= 40 ) {
			  output = Compute(fwdPID, current_distance, setpoint );
		  
			  printf("\n *********************  FWD PID ENABLED  ********************\n\n");
			  printf("CurrDist = %f\tError = %f\n", current_distance, error);			  
			  printf("FWD PID Output = %f\n", output);
			  
			  // correlate output to a bot_speed NEGATIVE SPEEDS MOVE THE BOT BACKWARDS
			    
			  bot_speed = fwdSpeedScaling(output);
		}
		
		printf("Go To Bot speed = %d\n", bot_speed);
		
		// move the bot based on bot_speed and define expected velocities for kalmann filter
		if(bot_speed > 0) {
			//if 	( theta_error > 0.175 ) ;	//ri_move(ri, RI_MOVE_FWD_LEFT, bot_speed);
			//else if ( theta_error < -0.175 ) ; //	ri_move(ri, RI_MOVE_FWD_RIGHT, bot_speed);
			ri_move(ri, RI_MOVE_FORWARD, bot_speed);//else 				
			
			/* expected velocities now scaled in half to compensate for network lag */
			expected_vel->v[0] = fwd_speed[bot_speed - 1] * cos(current_location->v[2]) * 0.5;
			expected_vel->v[1] = fwd_speed[bot_speed - 1] * sin(current_location->v[2]) * 0.5;
		}
		else {
			bot_speed *= -1;
			ri_move(ri, RI_MOVE_BACKWARD, bot_speed);
			/* expected velocities now scaled in half to compensate for network lag */
			expected_vel->v[0] = -1.0 * fwd_speed[bot_speed - 1] * cos(current_location->v[2]) * 0.5;
			expected_vel->v[1] = -1.0 * fwd_speed[bot_speed - 1] * sin(current_location->v[2]) * 0.5;
		}
		
		expected_vel->v[2] = 0.0;
		
		/* decriment i for wind up */
		i--;
		
		//refresh current position values and see if bot changed rooms.
   		get_Position(ri, current_location, expected_vel, FORWARD);
		
		/* show the bot moving */
		if(ri_get_image(ri, image) == RI_RESP_SUCCESS) {
			cvShowImage("Square Display", image);
			cvWaitKey(10);
		}
		else  printf("Unable to capture an image!\n");
	} while( error > FWD_PID_TOLERANCE );
	
	final_pos->v[0] = current_location->v[0];
	final_pos->v[1] = current_location->v[1];
	final_pos->v[2] = current_location->v[2];
	
	ri_move(ri, RI_STOP, 1);

	//point robot to end theta using PID //code me
	free(current_location);
	free(expected_vel);
}

//how many pairs should I expect in the next cell: will return 0, 1, or 2// call after its faced and moved
int pairsToExpect(robot_heading heading, robot_heading direction_to_move){
	int squares_in_front=0;
	
	if(facing == HEADING_UP){
		while( ( (y-1-squares_in_front) >=0 ) && (array2D(map,y-1-squares_in_front,x).type != MAP_OBJ_POST ) ){
			squares_in_front++;
		}
	}else if(facing == HEADING_DOWN){
		while( ( (y+1+squares_in_front) <= 4 ) && (array2D(map,y+1+squares_in_front,x).type != MAP_OBJ_POST ) ){
			squares_in_front++;
		}
	}else if(facing == HEADING_LEFT){
		while( ( (x-1-squares_in_front) >= 0) && (array2D(map,y,x-1-squares_in_front).type != MAP_OBJ_POST ) ){
			squares_in_front++;
		}
	}else if(facing == HEADING_RIGHT){
		while( ( (x+1+squares_in_front) <= 6) && (array2D(map,y,x+1+squares_in_front).type != MAP_OBJ_POST ) ){
			squares_in_front++;
		}
	}
	if(squares_in_front >=2){
		return 2;
	}else{
		return squares_in_front;
	}
}

int isObstructed(int cell_type){
	if(robotID == 0){//0==2
	      if((cell_type == MAP_OBJ_EMPTY)||
		(cell_type == MAP_OBJ_RESERVE_2)||
		(cell_type == MAP_OBJ_PELLET))
		    return 0;
	      else
		    return 1;
	} else {
		if((cell_type == MAP_OBJ_EMPTY)||
		(cell_type == MAP_OBJ_RESERVE_1)||
		(cell_type == MAP_OBJ_PELLET))
		    return 0;
		else
		    return 1;
	}
	  
}

//doesn't use backward right now
void makeAMove(robot_if_t *ri){//fill in outline comments
	robot_heading where_to_go = whereToGo(ri);
	
	if(facing != where_to_go){//point the robot in the direction its moving
		if((facing==HEADING_UP)&&(where_to_go==HEADING_DOWN)){
			printf("Turning 180 degrees.\n");
				
			ri_move(&ri, RI_TURN_RIGHT, 1);
			ri_move(&ri, RI_TURN_RIGHT, 2);
			ri_move(&ri, RI_TURN_RIGHT, 3);
			ri_move(&ri, RI_TURN_RIGHT, 1);
			ri_move(&ri, RI_TURN_RIGHT, 2);
			ri_move(&ri, RI_TURN_RIGHT, 3);
							
			direction -= M_PI;
		}
		else if((facing==HEADING_LEFT)&&(where_to_go==HEADING_RIGHT)){
			printf("Turning 180 degrees.\n");
				
			ri_move(&ri, RI_TURN_RIGHT, 1);
			ri_move(&ri, RI_TURN_RIGHT, 2);
			ri_move(&ri, RI_TURN_RIGHT, 3);
			ri_move(&ri, RI_TURN_RIGHT, 1);
			ri_move(&ri, RI_TURN_RIGHT, 2);
			ri_move(&ri, RI_TURN_RIGHT, 3);
							
			direction -= M_PI;
		}
		else if((facing==HEADING_RIGHT)&&(where_to_go==HEADING_LEFT)){
			printf("Turning 180 degrees.\n");
				
			ri_move(&ri, RI_TURN_RIGHT, 1);
			ri_move(&ri, RI_TURN_RIGHT, 2);
			ri_move(&ri, RI_TURN_RIGHT, 3);
			ri_move(&ri, RI_TURN_RIGHT, 1);
			ri_move(&ri, RI_TURN_RIGHT, 2);
			ri_move(&ri, RI_TURN_RIGHT, 3);
							
			direction -= M_PI;
		}
		else if((facing==HEADING_DOWN)&&(where_to_go==HEADING_UP)){
			printf("Turning 180 degrees.\n");
				
			ri_move(&ri, RI_TURN_RIGHT, 1);
			ri_move(&ri, RI_TURN_RIGHT, 2);
			ri_move(&ri, RI_TURN_RIGHT, 3);
			ri_move(&ri, RI_TURN_RIGHT, 1);
			ri_move(&ri, RI_TURN_RIGHT, 2);
			ri_move(&ri, RI_TURN_RIGHT, 3);
							
			direction -= M_PI;
		}
		else if((facing==HEADING_DOWN)&&(where_to_go==HEADING_LEFT)){
			printf("Turning right 90 degrees.\n");
			//rotate_to_theta(&ri, direction - M_PI/2.0, loc);
			ri_move(&ri, RI_TURN_RIGHT, 1);
			ri_move(&ri, RI_TURN_RIGHT, 2);
			ri_move(&ri, RI_TURN_RIGHT, 3);
							
			direction -= M_PI/2.0;
		}
		else if((facing==HEADING_LEFT)&&(where_to_go==HEADING_UP)){
			printf("Turning right 90 degrees.\n");
			//rotate_to_theta(&ri, direction - M_PI/2.0, loc);
			ri_move(&ri, RI_TURN_RIGHT, 1);
			ri_move(&ri, RI_TURN_RIGHT, 2);
			ri_move(&ri, RI_TURN_RIGHT, 3);
							
			direction -= M_PI/2.0;
		}
		else if((facing==HEADING_UP)&&(where_to_go==HEADING_RIGHT)){
			printf("Turning right 90 degrees.\n");
			//rotate_to_theta(&ri, direction - M_PI/2.0, loc);
			ri_move(&ri, RI_TURN_RIGHT, 1);
			ri_move(&ri, RI_TURN_RIGHT, 2);
			ri_move(&ri, RI_TURN_RIGHT, 3);
							
			direction -= M_PI/2.0;
		}
		else if((facing==HEADING_RIGHT)&&(where_to_go==HEADING_DOWN)){
			printf("Turning right 90 degrees.\n");
			//rotate_to_theta(&ri, direction - M_PI/2.0, loc);
			ri_move(&ri, RI_TURN_RIGHT, 1);
			ri_move(&ri, RI_TURN_RIGHT, 2);
			ri_move(&ri, RI_TURN_RIGHT, 3);
							
			direction -= M_PI/2.0;
		}
		else if((facing==HEADING_DOWN)&&(where_to_go==HEADING_RIGHT)){
			printf("Turning left 90 degrees.\n");
			//rotate_to_theta(&ri, direction + M_PI/2.0, loc);
			ri_move(&ri, RI_TURN_LEFT, 1);
			ri_move(&ri, RI_TURN_LEFT, 2);
			ri_move(&ri, RI_TURN_LEFT, 3);
			
			direction += M_PI/2.0;
		}
		else if((facing==HEADING_UP)&&(where_to_go==HEADING_LEFT)){
			printf("Turning left 90 degrees.\n");
			//rotate_to_theta(&ri, direction + M_PI/2.0, loc);
			ri_move(&ri, RI_TURN_LEFT, 1);
			ri_move(&ri, RI_TURN_LEFT, 2);
			ri_move(&ri, RI_TURN_LEFT, 3);
			
			direction += M_PI/2.0;
		}
		else if((facing==HEADING_LEFT)&&(where_to_go==HEADING_DOWN)){
			printf("Turning left 90 degrees.\n");
			//rotate_to_theta(&ri, direction + M_PI/2.0, loc);
			ri_move(&ri, RI_TURN_LEFT, 1);
			ri_move(&ri, RI_TURN_LEFT, 2);
			ri_move(&ri, RI_TURN_LEFT, 3);
			
			direction += M_PI/2.0;
		}
		else if((facing==HEADING_RIGHT)&&(where_to_go==HEADING_UP)){
			//turn 90 left
			printf("Turning left 90 degrees.\n");
			//rotate_to_theta(&ri, direction + M_PI/2.0, loc);
			ri_move(&ri, RI_TURN_LEFT, 1);
			ri_move(&ri, RI_TURN_LEFT, 2);
			ri_move(&ri, RI_TURN_LEFT, 3);
			
			direction += M_PI/2.0;
		}
		facing = where_to_go;//set updated heading
	}
	
	//move forward 1 space
	printf("Going ahead 65cm!\n\n");
	// + X 
	if(fabs(direction) <= M_PI/4.0 || fabs(direction) > 7.0*M_PI/4.0) {
					go_to_position(&ri, image, loc->v[0] + 65.0, loc->v[1] + 0.0, loc);
	}
	// + Y
	else if(fabs(direction) > M_PI/4.0 && fabs(direction) <= 3.0*M_PI/4.0) {
		if(direction >= 0) 	go_to_position(&ri, image, loc->v[0] + 0.0, loc->v[1] + 65.0, loc);
		else 			go_to_position(&ri, image, loc->v[0] + 0.0, loc->v[1] - 65.0, loc);
	}
	// - X
	else if(fabs(direction) > 3.0*M_PI/4.0 && fabs(direction) <= 5.0*M_PI/4.0) {
					go_to_position(&ri, image, loc->v[0] - 65.0, loc->v[1] + 0.0, loc);
	}
	// - Y
	else if(fabs(direction) > 5.0*M_PI/4.0 && fabs(direction) <= 7.0*M_PI/4.0){
		if(direction >= 0) 	go_to_position(&ri, image, loc->v[0] + 0.0, loc->v[1] - 65.0, loc);
		else			go_to_position(&ri, image, loc->v[0] + 0.0, loc->v[1] + 65.0, loc);
	}
	
	// center
	if(pairsToExpect(facing, where_to_go)==2){
	    //center with 2 pairs;
		printf("Centering expecting TWO Pairs!\n\n");
		center_robot(&ri, image, final_threshold, argc[1], atoi(argc[2]), flag);
	}
	if(pairsToExpect(facing, where_to_go)==1){
	      //center with 1 pair;
		  printf("Centering expecting ONE Pair!\n\n");
		  center_robot(&ri, image, final_threshold, argc[1],atoi(argc[2]), flag);
	}
	
	if(facing == HEADING_UP){//update heading
		//printf("facing UP\n");//diagnostic
		y--;
	}else if(facing == HEADING_DOWN){
		//printf("facing DOWN\n");//diagnostic
		y++;
	}else if(facing == HEADING_LEFT){
		//printf("facing LEFT\n");//diagnostic
		x--;
	}else if(facing == HEADING_RIGHT){
		//printf("facing RIGHT\n");//diagnostic
		x++;
	}
	
	// update map
	printf("updating map with x = %d and y = %d\n", x, y);//diagnostic
	ri_update_map(ri, x, y);
}
int sumCrawler(robot_heading comingFrom, int xCursor, int yCursor, int movesLeft){//helper method for whereToGo
	if(movesLeft==0){//base case
	    return 0;
	}
	
	robot_heading	direction_to_move, 
			direction_coming_from = comingFrom;
	
	int new_x = xCursor, 
	    new_y = yCursor, 
	    temp, 
	    max_value=-1;
	
	if(xCursor > 0 && comingFrom != HEADING_LEFT){//look left
		if(!isObstructed(array2D(map,yCursor,xCursor-1).type)){//if the spot that im checking isn't obstructed
			temp = array2D(map,yCursor,xCursor-1).points;	
			if(temp>max_value){
				//make this the new spot to go and update max value
				max_value = temp;
				direction_to_move = HEADING_LEFT;
				direction_coming_from = HEADING_RIGHT;
				new_x = xCursor - 1;//+ 1;
				new_y = yCursor;
			}
		}
	}
	if(yCursor>0&&comingFrom!=HEADING_UP){//look up
		if(!isObstructed(array2D(map,yCursor-1,xCursor).type)){//if the spot that im checking isn't obstructed
			temp = array2D(map,yCursor-1,xCursor).points;
			if(temp>max_value){
				//make this the new spot to go and update max value
				max_value = temp;
				direction_to_move = HEADING_UP;
				direction_coming_from = HEADING_DOWN;
				new_x = xCursor;
				new_y = yCursor - 1;
			}
		}
	  
	}
	if(xCursor<6&&comingFrom!=HEADING_RIGHT){//look right
		if(!isObstructed(array2D(map,yCursor,xCursor+1).type)){//if the spot that im checking isn't obstructed
			temp = array2D(map,yCursor,xCursor+1).points;
			if(temp>max_value){
				//make this the new spot to go and update max value
				max_value = temp;
				direction_to_move = HEADING_RIGHT;
				direction_coming_from = HEADING_LEFT;
				new_x = xCursor + 1;//- 1;
				new_y = yCursor;
			}
		}
	  
	}
	if(yCursor<4&&comingFrom!=HEADING_DOWN){//look down
		if(!isObstructed(array2D(map,yCursor+1,xCursor).type)){//if the spot that im checking isn't obstructed
			temp = array2D(map,yCursor+1,xCursor).points;
			if(temp>max_value){
				//make this the new spot to go and update max value
				max_value = temp;
				direction_to_move = HEADING_DOWN;
				direction_coming_from = HEADING_UP;
				new_x = xCursor;
				new_y = yCursor + 1;
			}
		}
	}
	return max_value + sumCrawler(direction_coming_from, new_x, new_y, movesLeft-1);
	
}
//finds biggest adjacent cell and goes there
robot_heading whereToGo(robot_if_t *ri){
	robot_heading direction_to_move;
	int spacestosum,
	    max_value, 
	    temp,
	    new_x,
	    new_y;
	
tryAgain:
	spacestosum = 3; //number of spaces to consider in deciding where to move
	direction_to_move = facing;
	max_value = 0;
	new_x = x;
	new_y = y;


	// repeat in case the square we want to move to somehow gets reserved before we can reserve it 
	do {
		// update map with functioning API calls
		get_map_array(map, ri, &score1, &score2);
		if(x>0){//look left
			if(!isObstructed(array2D(map,y,x-1).type)){//if the spot that im checking isn't obstructed
				temp = array2D(map,y,x-1).points;
				//crawl out here
				temp += sumCrawler(HEADING_RIGHT, (x - 1), y, (spacestosum-1));
				if(temp>max_value){
					//make this the new spot to go and update max value
					max_value = temp;
					direction_to_move = HEADING_LEFT;
					new_x = x - 1;//+ 1;
					new_y = y;
				}
			}
		}
		if(y>0){//look up
			if(!isObstructed(array2D(map,y-1,x).type)){//if the spot that im checking isn't obstructed
				temp = array2D(map,y-1,x).points;
				//crawl out here
				temp += sumCrawler(HEADING_DOWN, x, (y-1), (spacestosum-1));
				if(temp>max_value){
					//make this the new spot to go and update max value
					max_value = temp;
					direction_to_move = HEADING_UP;
					new_x = x;
					new_y = y - 1;
				}
			}
		  
		}
		if(x<6){//look right
			if(!isObstructed(array2D(map,y,x+1).type)){//if the spot that im checking isn't obstructed
				temp = array2D(map,y,x+1).points;
				//crawl out here
				temp += sumCrawler(HEADING_LEFT, (x+1), y, (spacestosum-1));
				if(temp>max_value){
					//make this the new spot to go and update max value
					max_value = temp;
					direction_to_move = HEADING_RIGHT;
					new_x = x + 1;//- 1;
					new_y = y;
				}
			}
		  
		}
		if(y<4){//look down
			if(!isObstructed(array2D(map,y+1,x).type)){//if the spot that im checking isn't obstructed
				temp = array2D(map,y+1,x).points;
				//crawl out here
				temp += sumCrawler(HEADING_UP, x, (y+1), (spacestosum-1));
				if(temp>max_value){
					//make this the new spot to go and update max value
					max_value = temp;
					direction_to_move = HEADING_DOWN;
					new_x = x;
					new_y = y + 1;
				}
			}
		}
		if(max_value == 0){//nothing found
			printf("nothing found within %d spaces.  Incrementing to %d.\n", spacestosum, (spacestosum+1));
			spacestosum++;//look further out next time
		}
		if(spacestosum > 10){
			exit(0);//get the fuck out
		}
	} while(max_value==0);  // try to reserve the square you want to move to
	
	if (ri_reserve_map(ri, new_x, new_y) != 0) goto tryAgain;
	
	printf("max sum = %d, heading = %d\n", max_value, direction_to_move);//diagnostic
	
	printf("Reserving x = %d\ty=%d.  Direction_to_move = %d\n", new_x, new_y, direction_to_move);
	
	return direction_to_move;
}

/* get map_list from server and put it into map array */
void get_map_array(array_map_obj_t *map, robot_if_t *ri, int *score1, int *score2){
	map_obj_t *map_list,
		  *map_list_idx;
	int i, j;
	if(firstRun==1)
		pointsInMap = 0;
	// Get the map from the server and update scores
        map_list = ri_get_map(ri, score1, score2);
	map_list_idx = map_list;
	
	for(i = 0; i < ROWS; i++) {
		for(j = 0; j < COLS; j++) {
			array2D(map,i,j).type  = map_list_idx->type;
			array2D(map,i,j).points  = map_list_idx->points;
			if(firstRun==1)
				pointsInMap += map_list_idx->points;
			map_list_idx = map_list_idx->next;
		}
	}
	
	// free map list
        while(map_list->next != NULL) {
		map_list_idx = map_list->next;
		free(map_list);
		map_list = map_list_idx;
	}
	free(map_list);
	if(firstRun==1)
		printf("%d points in map", pointsInMap);//diagnostic
	firstRun = 0;
}

/* MAIN */
int main(int argv, char **argc) {
	robot_if_t	ri;
	int 	i, 
			j;
	
	image = NULL;
	final_threshold = NULL;			
	score1 = 0;
    score2 = 0;
	firstRun = 1;
	direction = 0.0;
			
	// Make sure we have a valid command line argument
	if(argv <= 2) {
		printf("Usage: robot_test <address of robot>, argc[2]: 0 = BOT 2 start position, 1 = BOT 1 start position\n");	
		exit(-1);
	}

	// Setup the robot with the address passed in
	if(ri_setup(&ri, argc[1], 0)) {
		printf("Failed to setup the robot!\n");
		exit(-1);
	}
	
	if(atoi(argc[2]) != 0 && atoi(argc[2]) != 1){
		printf("Usage: robot_game_example <address of robot> <starting position(1 or 2)\n");
		//printf("argc[2]: 0 = right to left, 1 = left to right\n");	
		exit(-1);
	}

	// Setup the camera
	if(ri_cfg_camera(&ri, RI_CAMERA_DEFAULT_BRIGHTNESS, RI_CAMERA_DEFAULT_CONTRAST, 5, RI_CAMERA_RES_640, RI_CAMERA_QUALITY_HIGH)) {
		printf("Failed to configure the camera!\n");
		exit(-1);
	}
	
	if(ri_getHeadPosition(&ri) != RI_ROBOT_HEAD_LOW ) ri_move(&ri, RI_HEAD_DOWN , 1);
	
	loc = (vector *)calloc(1, sizeof(vector));
	vel = (vector *)calloc(1, sizeof(vector));
	map = (array_map_obj_t*) malloc(sizeof(array_map_obj_t) * ROWS * COLS);
	
	robotID = (int)strtol ( argc[2], NULL, 0 );
	
	loc->v[0] = 0;
	loc->v[1] = 0;
	// set up path finding and initial theta based on starting position
	if(robotID == 1){
	    y = 2;
	    x = 0;
	    facing = HEADING_RIGHT;
		loc->v[2] = M_PI;
	}
	else {
	    y = 2;
	    x = 6;
	    facing = HEADING_LEFT;
		loc->v[2] = 0.0;
	}
	
	direction = loc->v[2];
	
	vel->v[0] = 0;
	vel->v[1] = 0;
	vel->v[2] = M_PI/4.0;

	// Create an image to store the image from the camera
	image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
	final_threshold = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);

	// Initialize PID controllers
	fwdPID = calloc(1, sizeof(PID));
	rotPID = calloc(1, sizeof(PID));
	init_PID(fwdPID, F_Kp, F_Ki, F_Kd);
	init_PID(rotPID, R_Kp, R_Ki, R_Kd);

	// Retrieve initial position, initailize current and last
	init_pos(&ri);

	cvNamedWindow("Square Display", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Thresholded", CV_WINDOW_AUTOSIZE);

	printf("Robot ID = %d\tx = %d, y = %d\n", robotID, x, y);
	
	// run the game until score is >= half the total points
	while((score1 < (pointsInMap+2)/2) && (score2 < (pointsInMap+2)/2)) {
		
		makeAMove(&ri);
		// print the map
		printf("Score: %i to %i\n", score1, score2);
		for(i = 0; i < ROWS; i++) {
			for(j = 0; j < COLS; j++) {
				switch(array2D(map,i,j).type){
					case MAP_OBJ_EMPTY:
						printf(" ");
						break;
					case MAP_OBJ_PELLET:
						printf("%i", array2D(map,i,j).points);
						break;
					case MAP_OBJ_ROBOT_1:
						printf("r");
						break;
					case MAP_OBJ_ROBOT_2:
						printf("R");
						break;
					case MAP_OBJ_POST:
						printf("X");
						break;
				}
				printf("\t");
			}
			printf("\n");
		}		
	}

	free(fwdPID);
	free(rotPID);
	free(loc);
	free(vel);

	exit_pos();
	
	// Free the images
	cvReleaseImage(&final_threshold);
	cvReleaseImage(&image);

	cvDestroyWindow("Square Display");
	cvDestroyWindow("Thresholded");

	return 0;
}