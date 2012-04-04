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

/* DEFINES */
#define F_Kp 1.0
#define F_Ki 0.1
#define F_Kd 0.01

#define R_Kp 10.0/M_PI
#define R_Ki 0.5
#define R_Kd 0.20

#define FWD_PID_TOLERANCE  15.0  /*How close should I be to the waypoint before moving onto the next one? */

/* GLOBALS */
PID 	*fwdPID,
	*rotPID;

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
		
		printf("Kalmann filtered result = %f\t%f\t%f\n\n", current_location->v[0], current_location->v[1], current_location->v[2]);
		
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
		
		// move the bot based on bot_speed and define expected velocities for kalmann filter
		if(bot_speed > 0) {
			if 	( theta_error > 0.175 ) ;	//ri_move(ri, RI_MOVE_FWD_LEFT, bot_speed);
			else if ( theta_error < -0.175 ) ; //	ri_move(ri, RI_MOVE_FWD_RIGHT, bot_speed);
			else 				ri_move(ri, RI_MOVE_FORWARD, bot_speed);
			
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
		
		//refresh current position values and see if bot changed rooms.  If it does, reset scaling factor
   		get_Position(ri, current_location, expected_vel, FORWARD);
		
		/* show the bot moving */
		if(ri_get_image(ri, image) == RI_RESP_SUCCESS) {
			cvShowImage("Square Display", image);
			cvWaitKey(10);
		}
		else  printf("Unable to capture an image!\n");
		
		printf("Kalmann filtered result = %f\t%f\t%f\n", current_location->v[0], current_location->v[1], current_location->v[2]);		
	} while( error > FWD_PID_TOLERANCE );
	
	final_pos->v[0] = current_location->v[0];
	final_pos->v[1] = current_location->v[1];
	final_pos->v[2] = current_location->v[2];
	
	ri_move(ri, RI_STOP, 1);

	//point robot to end theta using PID //code me
	free(current_location);
	free(expected_vel);
}

/* Print out a menu of selections */
int printmenu(){
	int input = -1;
	
	printf("What would you like to do?\n");
	printf("\t0. Quit.\n");
	printf("\t1. Go straight ahead 1 block.\n");
	printf("\t2. Turn right 90 degrees.\n");
	printf("\t3. Turn left 90 degrees.\n");
	printf("\t4. Turn around (180 degrees).\n");
	printf("\t5. Just Center.\n");
	
	while(input < 0 || input > 5) {
		printf("Please choose a command (0 - 5):\t");
		input = getc(stdin) - '0';
		printf("\n");
	}
	
	return input;
}

/* MAIN */
int main(int argv, char **argc) {
	robot_if_t	ri;
	IplImage	*image = NULL, 
			*final_threshold = NULL;
	vector 		*loc,
			*vel;
	float 		target_x,
			target_y;	
	int		flag = 1;

		// Setup the robot with the address passed in
		if(ri_setup(&ri, argc[1], 0)) printf("Failed to setup the robot!\n");

	if(ri_getHeadPosition(&ri) != RI_ROBOT_HEAD_LOW ) ri_move(&ri, RI_HEAD_DOWN , 1);

	// Check condition of battery, exit if not enough charge
	//battery_check(&ri);

	loc = (vector *)calloc(1, sizeof(vector));
	vel = (vector *)calloc(1, sizeof(vector));
	
	loc->v[0] = 0;
	loc->v[1] = 0;
	loc->v[2] = 0;
	
	vel->v[0] = 0;
	vel->v[1] = 0;
	vel->v[2] = 0;

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

	//cvNamedWindow("Rovio Camera", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Square Display", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Thresholded", CV_WINDOW_AUTOSIZE);

	flag = printmenu();
	
	while(flag != 0) {
		switch(flag) {
			case 1:
			{
				go_to_position(&ri, image, loc->v[0] + 65.0, loc->v[1] + 0.0, loc);
				break;
			}
			case 2:
			{
				break;
			}
			case 3:
			{
				break;
			}
			case 4:
			{
				break;
			}
			default:
			{
				break;
			}			  
		}
		
		get_Position(&ri, loc, vel, ROTATE);
		
		center_robot(&ri, image, final_threshold, argc[1]);
		
		get_Position(&ri, loc, vel, FORWARD);

		flag = printmenu();
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