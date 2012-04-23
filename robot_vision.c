/* Filename:  robot_vision.c
 * Author:	Based on robot_camera_example.c from API
 * 
 * 05-16: I added the cvOr() function to compute the new threshold, it definitely has improved our thresholded image.
 * 	 But the thresholded image still needs some work. I tried various combinations of HSV values but it didn't help that much.
 * 	 Also, I added a function (get_diff_in_x()) to calculate the difference in distance between the biggest two squares 
 * 	and the center vertical line. 
 * 
 * A side note: I found that the resolution/quality of the cameras varies with different robots. Please use "Optimus"
 * 
 * 05-17: I tried several different robots and found that Optimus had the best threshold image for the current HSV values.
 *	Also, I added a routine to move the robot through the maze.  It actually works pretty well 50% of the times, until 
 *      the end where the robot has to make a right turn.  Because I didn't implement anything to keep track the distance 
 *      the robot has gone, so it doesn't know when to make that 90 degrees right turn yet.  
 * 
 * (Lines 267- 305 are the movement stuff)
 * 
 * 03/30: 
 * 	if (hasTwoPair) use drift
 * 	if (hasOnePair) use rotate
 * 	if (two largest squares) use rotate 
 * 
 * 04/01:
 * 	- weighted average weight 1: 1404
 * 	- weighted average weight 2: 1643 +-46
 */	

#include "robot_vision.h"

/* Sort the list of squares from Largest to Smallest by Area */
void sort_squares(squares_t *squares) {
	squares_t *sq_idx, 
	          *counter;
	int temp;
	
	if (squares == NULL) {
		printf("List does not exist\n\n");
		return;
	}
	
	sq_idx = squares;
	for(; sq_idx->next != NULL; sq_idx = sq_idx->next)
	{
		for(counter = sq_idx->next; counter != NULL; counter = counter->next)
		{
			if(sq_idx->area < counter->area)
			{
				/* swap areas */
				temp = sq_idx->area;
				sq_idx->area = counter->area;
				counter->area = temp;
				
				/* swap center.x */
				temp = sq_idx->center.x;
				sq_idx->center.x = counter->center.x;
				counter->center.x = temp;
				
				/* swap center.y */
				temp = sq_idx->center.y;
				sq_idx->center.y = counter->center.y;
				counter->center.y = temp;
			}
		}
	}  
}

void copy_square(squares_t *dst, squares_t *src) {
	dst->area = src->area;
	dst->center.x = src->center.x;
	dst->center.y = src->center.y;  
}

/* Compute the difference in distance between the biggest two squares and the center vertical line */
int get_diff_in_x(squares_t *square1, squares_t *square2, IplImage *img){
	CvPoint pt1, pt2;
	int dist_to_center1, dist_to_center2, diff;
	
	pt1 = square1->center;
	pt2 = square2->center;
	
	dist_to_center1 = pt1.x - img->width/2;
	dist_to_center2 = pt2.x - img->width/2;
	
	//negative diff means robot pointing to the right of the origin, positive diff means robot pointing to the left of the origin
	diff = dist_to_center1 + dist_to_center2;
	
	printf("square 1 distance = %d\t square 2 distance = %d\t difference in distance = %d\n", 
	       dist_to_center1, dist_to_center2, diff);
	
	return diff;
}

/* Check the differnce in y between two squares */
int get_diff_in_y(squares_t *square1, squares_t *square2){
	int y_1, y_2, diff;
	
	y_1 = square1->center.y;
	y_2 = square2->center.y;
	
	diff = abs(y_1 - y_2);
	//printf("square_1 y = %d\t square_2 y = %d\tdifference in y = %d\n", y_1, y_2, diff);
	return diff;
}

/* check the difference in area between two squares */
int get_diff_in_area(squares_t *square1, squares_t *square2){
	return abs(square1->area - square2->area);
}

/* Get the ratio of area between two squares as Smaller Area / Larger Area */
float getRatio(int x, int y) {  // x>y
	float r;
	
	if( x < y ) 	r = (float) x / (float) y;
	else if(x > y)	r = (float) y / (float) x;
	else 		r = 1.0;
		
	//printf("Area ratio = %f\n", r);
	return r;
}

/* checks if the same square gets marked twice */
bool is_same_square(squares_t *square1, squares_t *square2){
	CvPoint pt1, pt2;
	int x_diff, y_diff;
	pt1 = square1->center;
	pt2 = square2->center;
	
	x_diff = abs(pt1.x - pt2.x);
	y_diff = abs(pt1.y - pt2.y);
	
	if ((0 <= x_diff && x_diff <= 3) && (0 <= y_diff && y_diff <= 3))return true;
	   
	return false;
}

/* Checks two squares to see if they are a pair based on the ratio between their areas */
int isPair(squares_t *square1, squares_t *square2, float area_ratio_threshold){//set thresh around .75
	//compare areas
	float ratio;
	int diff;
	bool same_square;
	
	ratio = getRatio(square1->area, square2->area);
	
	diff = get_diff_in_y(square1, square2);
	
	same_square = is_same_square(square1, square2);
	
	/* if ratio greater than threshold, the difference in y is small, and they are NOT the same square */
	if( (ratio > area_ratio_threshold ) && ( diff < 30 ) && !same_square )	return 1;
	//if( (ratio > area_ratio_threshold ) && !same_square )	return 1;
	else									return 0;
}

/* Draws an X of the square s, using the RGB values passed in as parameters */
void draw_X(squares_t *s, IplImage *img, int R, int G, int B) {
	CvPoint pt1, pt2;
	int sq_amt = (int) (sqrt(s->area) / 2);	

	// Upper Left to Lower Right
	pt1.x = s->center.x - sq_amt;
	pt1.y = s->center.y - sq_amt;
	pt2.x = s->center.x + sq_amt;
	pt2.y = s->center.y + sq_amt;
	cvLine(img, pt1, pt2, CV_RGB(R, G, B), 3, CV_AA, 0);

	// Lower Left to Upper Right
	pt1.x = s->center.x - sq_amt;
	pt1.y = s->center.y + sq_amt;
	pt2.x = s->center.x + sq_amt;
	pt2.y = s->center.y - sq_amt;
	cvLine(img, pt1, pt2, CV_RGB(R, G, B), 3, CV_AA, 0);
}

/* return the slope of two points */
float get_intersect_slope(int x1, int x2, int y1, int y2){
	return ((y1 - y2)/((float) x1 - (float) x2));
}

/* draw instersection lines from two pairs of squares */
float draw_intersect_line(squares_t *square_1, squares_t *square_2, squares_t *sec_squares_1, squares_t *sec_squares_2,
			 IplImage *image, int R, int G, int B) {
	CvPoint start, end;
	float slope1, slope2;
	
	//square 1 on the left side
	if (square_1->center.x < image->width/2){
		//get square_1 slope
		if (sec_squares_1->center.x < image->width/2)
			slope1 = get_intersect_slope( square_1->center.x, sec_squares_1->center.x, square_1->center.y, sec_squares_1->center.y);
		else 
			slope1 = get_intersect_slope( square_1->center.x, sec_squares_2->center.x, square_1->center.y, sec_squares_2->center.y);
		//printf("square1 area = %d square2 area = %d slope = %f\n", square_1->area, temp->area, slope);
		start.x = square_1->center.x;
		start.y = square_1->center.y;
		end.x = square_1->center.x + multiplier;
		end.y = square_1->center.y + slope1*multiplier;
		
	}
	
	//square 1 on the right side
	else{
		//get square_1 slope
		if (sec_squares_1->center.x > image->width/2)
			slope1 = get_intersect_slope( square_1->center.x, sec_squares_1->center.x, square_1->center.y, sec_squares_1->center.y);
		else 
			slope1 = get_intersect_slope( square_1->center.x, sec_squares_2->center.x, square_1->center.y, sec_squares_2->center.y);
		start.x = square_1->center.x;
		start.y = square_1->center.y;
		end.x = square_1->center.x - multiplier;
		end.y = square_1->center.y - slope1 *multiplier;
	}
	cvLine(image, start, end, CV_RGB(R, G, B), 3, CV_AA, 0);
	
	//square 2 on the left side
	if (square_2->center.x < image->width/2){
		//get square_2 slope
		if (sec_squares_1->center.x < image->width/2)
			slope2 = get_intersect_slope( square_2->center.x, sec_squares_1->center.x, square_2->center.y, sec_squares_1->center.y);
		else 
			slope2 = get_intersect_slope( square_2->center.x, sec_squares_2->center.x, square_2->center.y, sec_squares_2->center.y);
		start.x = square_2->center.x;
		start.y = square_2->center.y;
		end.x = square_2->center.x + multiplier;
		end.y = square_2->center.y + slope2 *multiplier;
	}
	
	//square 2 on the right side
	else{
		//get square_2 slope
		if (sec_squares_1->center.x > image->width/2)
			slope2 = get_intersect_slope( square_2->center.x, sec_squares_1->center.x, square_2->center.y, sec_squares_1->center.y);
		else 
			slope2 = get_intersect_slope( square_2->center.x, sec_squares_2->center.x, square_2->center.y, sec_squares_2->center.y);
		start.x = square_2->center.x;
		start.y = square_2->center.y;
		end.x = square_2->center.x - multiplier;
		end.y = square_2->center.y - slope2 *multiplier;
	}
	cvLine(image, start, end, CV_RGB(R, G, B), 3, CV_AA, 0);
	
	printf("Slope 2 = %f\tSlope 1 = %f\n", slope2, slope1);
	
	return slope1 + slope2;
}

/* get average area of a pair of squares */
int get_pair_average_area(squares_t *square_1, squares_t *square_2){
	return (square_1->area + square_2->area)/2;
}

/* draw a vertical line down the middle of the screen for user reference */
void draw_vertical_line(IplImage *img){
	CvPoint pt1, pt2;
	pt1.x = img->width/2;
	pt1.y = 0;
	pt2.x = img->width/2;
	pt2.y = img->height;
	
	cvLine(img, pt1, pt2, CV_RGB(0, 60, 255), 3, CV_AA, 0);
}

/* print out list of square areas */
void printAreas(squares_t *squares) {
	squares_t *sq_idx;
	printf("Areas of squares: \n");
	
	sq_idx = squares;
	while(sq_idx != NULL) {
               printf("  %d\n", sq_idx->area);
               sq_idx = sq_idx->next;
       }
}

/* Find squres in the thresholded image, sort the list returned, and identify best state in list */
square_state get_squares(robot_if_t *ri, squares_t *square_list, IplImage *image, IplImage *final_threshold, 
			 float *slope_diff, int x, int y, int robot_dir) {
	IplImage	*hsv = NULL, 
			*threshold_1 = NULL, 
			*threshold_2 = NULL;
	squares_t 	*squares,
			*sq_idx;
	square_state 	s = noneFound;

	// Update the robot's sensor information
	if(ri_update(ri) != RI_RESP_SUCCESS) {
		if(ri_update(ri) != RI_RESP_SUCCESS) {
			printf("Failed to update sensor information!\n");
		}
	}
	
	cvWaitKey(75);

	// Get the current camera image
	if(ri_get_image(ri, image) != RI_RESP_SUCCESS) {
		printf("Unable to capture an image!\n");		
	}
	
	*slope_diff = 1.0;
	
	/* initialize threshold image */
	// Create an image to store the HSV version in
	// We configured the camera for 640x480 above, so use that size here
	hsv = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);

	// And an image for each thresholded version
	threshold_1 = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	threshold_2 = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	
	// Convert the image from RGB to HSV
	cvCvtColor(image, hsv, CV_BGR2HSV);
	
	/* replace with x and y and heading */
	//row 4
	if (y == 4){
		printf("HSV Threshold: Dark\n");
		// Pick out the first range of pink color from the image
		cvInRangeS(hsv, RC_PINK_LOW_1_bender2, RC_PINK_HIGH_1_bender2, threshold_1);
		
		// Pick out the second range of pink color from the image
		cvInRangeS(hsv, RC_PINK_LOW_2_bender2, RC_PINK_HIGH_2_bender2, threshold_2);
		
	}
	//row 0 and columns 0-2 and facing right
	else if (y == 0 && x >= 0 && x <= 2 && robot_dir == HEADING_RIGHT ){
		printf("HSV Threshold: Dark\n");
		// Pick out the first range of pink color from the image
		cvInRangeS(hsv, RC_PINK_LOW_1_bender2, RC_PINK_HIGH_1_bender2, threshold_1);
		
		// Pick out the second range of pink color from the image
		cvInRangeS(hsv, RC_PINK_LOW_2_bender2, RC_PINK_HIGH_2_bender2, threshold_2);
	}
	//column 0 and facing up
	else if (x == 0 && robot_dir == HEADING_UP){
		printf("HSV Threshold: Dark\n");
		// Pick out the first range of pink color from the image
		cvInRangeS(hsv, RC_PINK_LOW_1_bender2, RC_PINK_HIGH_1_bender2, threshold_1);
		
		// Pick out the second range of pink color from the image
		cvInRangeS(hsv, RC_PINK_LOW_2_bender2, RC_PINK_HIGH_2_bender2, threshold_2);
	}
	//column 0, row 0, and facing down
	else if (x == 0 && y == 0 && robot_dir == HEADING_DOWN){
		printf("HSV Threshold: Dark\n");
		// Pick out the first range of pink color from the image
		cvInRangeS(hsv, RC_PINK_LOW_1_bender2, RC_PINK_HIGH_1_bender2, threshold_1);
		
		// Pick out the second range of pink color from the image
		cvInRangeS(hsv, RC_PINK_LOW_2_bender2, RC_PINK_HIGH_2_bender2, threshold_2);
	}
	else{
		printf("HSV Threshold: Normal\n");
		// Pick out the first range of pink color from the image
		cvInRangeS(hsv, RC_PINK_LOW_1_bender, RC_PINK_HIGH_1_bender, threshold_1);
			
		// Pick out the second range of pink color from the image
		cvInRangeS(hsv, RC_PINK_LOW_2_bender, RC_PINK_HIGH_2_bender, threshold_2);
	}
	
	// compute the final threshold image by using cvOr
	cvOr(threshold_1, threshold_2, final_threshold, NULL);
	
	/* show final thresholded image for testing */
	//cvShowImage("Thresholded", final_threshold);
	// Find the squares in the image
	squares = ri_find_squares(final_threshold, RI_DEFAULT_SQUARE_SIZE);
	
	/* If any squares are found */
	if( squares != NULL ) {
		/* sort squares from largest to smallest */
		sort_squares(squares);
		
		//printAreas(squares);
		
		//find largest useful pair (if they exist)
		sq_idx = squares;
		
		// Search for pairs
		while(sq_idx != NULL){
			if(sq_idx->next == NULL) break;
			
			else if(isPair(sq_idx, sq_idx->next, AREA_THRESHOLD)){
				if (s == noneFound ){
					printf("Got One Pair!\n");
					copy_square(square_list,sq_idx);
					copy_square(square_list->next, sq_idx->next);
					
					s = hasOnePair;
					sq_idx = sq_idx->next;
				}
				//make sure the same square doesn't appear twice
				else if (s == hasOnePair && !is_same_square(square_list, sq_idx) && 
					!is_same_square(square_list, sq_idx->next) && 
					!is_same_square(square_list->next, sq_idx) &&
					!is_same_square(square_list->next, sq_idx->next)) {
					
					printf("Found Second Pair!\n");
					copy_square(square_list->next->next, sq_idx);
					copy_square(square_list->next->next->next,sq_idx->next);
					s = hasTwoPair;
					
					break;
				}
				
			}
			sq_idx = sq_idx->next;
		}
	
		/* if pair is found, mark them for later use */	
		if(s == hasOnePair || s == hasTwoPair){
			draw_X(square_list, image, 0, 255, 0);
			draw_X(square_list->next, image, 0, 255, 0);
			
			// if two pairs are found, draw the intersect line between them
			if (s == hasTwoPair){
				printf("2 Pairs found.\n");
				draw_X(square_list->next->next, image, 0, 0, 255);
				draw_X(square_list->next->next->next, image, 0, 0, 255);
				
				*slope_diff = draw_intersect_line(square_list, square_list->next, 
								  square_list->next->next, 
								  square_list->next->next->next, 
								  image, 0, 160, 255);
			}
		}
		
		else /* otherwise, mark the largest squares found */
		{
			copy_square( square_list, squares);
			
			s = onlyLargest;
			
			draw_X(square_list, image, 255, 0, 0);
			
			sq_idx = squares;
			
			while(sq_idx != NULL){
				if(sq_idx->next == NULL) break;
				else if(!is_same_square(sq_idx, sq_idx->next) ){
					break;
				}
				sq_idx = sq_idx->next;
			}
			
			if(sq_idx->next != NULL) {
				copy_square(square_list->next, sq_idx->next);
				draw_X(square_list->next, image, 255, 255, 0);
				
				s = twoLargest;
				printf ("Two Largest Found.\n");
			}
			else printf ("Only Largest Found.\n");
		}
	}
	
	// Release the square data
	while(squares != NULL) {
		sq_idx = squares->next;
		free(squares);
		squares = sq_idx;	
	}
	
	// display a straight vertical line
	draw_vertical_line(image);
		
	// Display the image with the drawing oon ito
	cvShowImage("Thresholded", final_threshold);
	cvShowImage("Square Display", image);
		
	// Update the UI (10ms wait)
	cvWaitKey(10);
	
	cvReleaseImage(&hsv);
	cvReleaseImage(&threshold_1);
	cvReleaseImage(&threshold_2);
	
	return s;
}

//try to center the robot
void center_robot(robot_if_t *ri, IplImage *image, IplImage *final_threshold, int x, int y, robot_heading robot_dir, int flag){
	int 		x_dist_diff,
			intersect_x = 0,
			change_dir = 0,
			last_largest_x = -1,
			initial_largest_x = -1,
			avg_area,
			last_turn_dir = 0,  //0 = left, 1 = right
			single_pair_count = 0, //strafe every 10
			i;
	float		slope_diff = 1.0,
			tol = 0.05;
	squares_t 	*square_list = NULL,
			*sq_idx;
	square_state state = noneFound;
	
	// Move the head up to the middle position
	for(i = 0; i < 10; i++) ri_move(ri, RI_HEAD_MIDDLE, RI_FASTEST);
		
	/* Initialize square_list to hold up to four squares */
	square_list = malloc(sizeof(squares_t));
	sq_idx = square_list;
	for( i = 0; i <= 3; i++) {
		if(i != 3) sq_idx->next = malloc(sizeof(squares_t));
		else sq_idx->next = NULL;
		
		sq_idx = sq_idx->next;
	}
	
	//find the squares list
	state = get_squares(ri, square_list, image, final_threshold, &slope_diff, x, y, robot_dir);
		
	/* State machine
	 * 1. pointTo:  Point to Center (find pairs)
	 * 2. strafeTo: strafe to even out areas, repeat 1 if necc
	 * 3. moveTo: move forwards or backwards to ensure centered
	 * 4. Report on center
	 */
	i = 0;  /* now going to use i to count the times we strafe */
	
	pointTo:
		while (state != hasTwoPair){
			//)getc(stdin;
			printf("In pointTo State!\n");
			switch (state){
				case hasOnePair:
				{
					change_dir = 0;
					last_largest_x = -1;
					initial_largest_x = -1;
					
					x_dist_diff = get_diff_in_x(square_list, square_list->next, image);
					
					//rotate to the left
					if (x_dist_diff < 0){
						//don't roate
						if (flag == 5 && x_dist_diff > -50) break;
					
						printf("Has pair.  Diff < 0.  rotate left at speed = 5\n");
						ri_move(ri, RI_TURN_LEFT, 5);
						ri_move(ri, RI_STOP, 10);
						last_turn_dir = 0;
						single_pair_count++;
						
						//strafe left every 10 rotates when single pair was found
						if (single_pair_count == 10){
							single_pair_count = 0;
							printf("Has pair.  Diff < 0.  strafe left at speed = 3\n");
							ri_move(ri, RI_MOVE_LEFT, 3);
							ri_move(ri, RI_STOP, 10);
						}
					}
					
					//rotate to the right
					else if (x_dist_diff > 0){
						//don't roate
						if (flag == 5 && x_dist_diff < 50) break;
						printf("Has pair.  Diff > 0.  rotate right at speed = 5\n");
						ri_move(ri, RI_TURN_RIGHT, 5);
						ri_move(ri, RI_STOP, 10);
						last_turn_dir = 1;
						single_pair_count++;
						
						//strafe left every 10 rotates when single pair was found
						if (single_pair_count == 10){
							single_pair_count = 0;
							printf("Has pair.  Diff <  0.  strafe right at speed = 3\n");
							ri_move(ri, RI_MOVE_RIGHT, 3);
							ri_move(ri, RI_STOP, 10);
						}
					}
					
					break;
				}
				
				case twoLargest:
				{	
					//do not handle rotation with two largest squares, when the bot is facing the wall
					//if (flag != 5){
						change_dir = 0;
						last_largest_x = -1;
						initial_largest_x = -1;
						
						if (square_list->center.x < square_list->next->center.x){
							printf("Larger square to left of smaller.  rotate right at speed = 6\n");
							ri_move(ri, RI_TURN_RIGHT, 3);
							ri_move(ri, RI_STOP, 10);
							last_turn_dir = 1;
						}
						//If largest is to the RIGHT of the next largest, turn left
						else if (square_list->center.x > square_list->next->center.x){
							printf("Larger square to right of smaller.  rotate left at speed = 6\n");
							ri_move(ri, RI_TURN_LEFT, 3);
							ri_move(ri, RI_STOP, 10);
							last_turn_dir = 0;
						}
					//}
					break;
				}
					
				case onlyLargest:
				{
					// if this isn't the first time we've seen only largest 
					if(initial_largest_x > -1 && change_dir == 0) {
						// check to see if square crossed center going left, change direction to right
						// check to see if you've moved a third of the screen with only a single square
						if((square_list->center.x - initial_largest_x) <= -image->width/2.0)
							change_dir = 1;
						// check to see if square crossed center going right, change direction to left
						// check to see if you've moved a third of the screen with only a single square
						else if((square_list->center.x - initial_largest_x) >= image->width/2.0)
							change_dir = 2;
						
						/* YE OLDE CODE
						if(last_largest_x > image->width/2 && square_list->center.x <= image->width/2)
						else if(last_largest_x < image->width/2 && square_list->center.x >= image->width/2)
						*/
					}
					
					if (change_dir == 0) {
					
						if (square_list->center.x < image->width/2){
							printf("Only Largest Found on left. rotate left at speed = 6\n");
							ri_move(ri, RI_TURN_LEFT, 3);
							ri_move(ri, RI_STOP, 10);
							last_turn_dir = 0;
						}
						
						else if (square_list->center.x > image->width/2){
							printf("Only Largest Found on right.  rotate right at speed = 6\n");
							ri_move(ri, RI_TURN_RIGHT, 3);
							ri_move(ri, RI_STOP, 10);
							last_turn_dir = 1;
						} 
						
						// grab center of the first onlyLargest you see
						if(initial_largest_x == -1) initial_largest_x = square_list->center.x;
						/* more old code */
						//last_largest_x = square_list->center.x;
					}
					else if (change_dir == 1) {  // turn right 
						printf("You crossed the line rotating left!  Changing to rotate right!\n");
						ri_move(ri, RI_TURN_RIGHT, 3);
						ri_move(ri, RI_STOP, 10);
						last_turn_dir = 1;
					}
					else if (change_dir == 2) {
						printf("You crossed the line rotating right!  Changing to rotate left!\n");
						ri_move(ri, RI_TURN_LEFT, 3);
						ri_move(ri, RI_STOP, 10);
						last_turn_dir = 0;
					}
					else printf("You should never make it to this else statement!\n");
					
					break;
				}
				
				// none found, possibly return a value to indicate as much 
				default:
				{ 
					if (last_turn_dir == 0){
						printf("No squares found!  Changing to rotate right!\n");
						ri_move(ri, RI_TURN_RIGHT, 3);
						ri_move(ri, RI_STOP, 10);
						last_turn_dir = 1;					  
					}
					else if (last_turn_dir == 1){
						printf("No squares found!  Changing to rotate left!\n");
						ri_move(ri, RI_TURN_LEFT, 1);
						ri_move(ri, RI_STOP, 10);
						last_turn_dir = 0;
					}
					
					break;
				}
			}
			// Center with one pair when facing the wall
			if (flag == 5){
				state = get_squares(ri, square_list, image, final_threshold, &slope_diff, x, y, robot_dir);
				
				if (state == hasOnePair){
					x_dist_diff = get_diff_in_x(square_list, square_list->next, image);
					//the direction of the robot that's facing is pretty much centered, now center it's position
					if (x_dist_diff < 50 && x_dist_diff > -50) break;
					
					//else break;
					single_pair_count++;
				}
			}
			//find the squares list
			state = get_squares(ri, square_list, image, final_threshold, &slope_diff, x, y, robot_dir);
			intersect_x = 0;	
		}
		
	// strafeTo:
		while((slope_diff > tol || slope_diff < -tol) && flag == 4) {
		  	printf("In strafeTo State!\n");
			i++;
			
			//strafe to the left
			if (slope_diff  < 0){
				printf("Pointing right of middle.  strafe left at speed = 2\n");
				if( i < 3 )     ri_move(ri, RI_MOVE_LEFT, 1);
				else 		ri_move(ri, RI_MOVE_LEFT, 2);
				ri_move(ri, RI_STOP, 10);
				last_turn_dir = 0;
			}
			
			//strafe to the right
			else if (slope_diff  > 0){
				printf("Pointing left of middle.  strafe right at speed = 2\n");
				if( i < 3 ) 	ri_move(ri, RI_MOVE_RIGHT, 1);
				else		ri_move(ri, RI_MOVE_RIGHT, 2);
				ri_move(ri, RI_STOP, 10);
				last_turn_dir = 1;
			}
			
			//find the squares list
			state = get_squares(ri, square_list, image, final_threshold, &slope_diff, x, y, robot_dir);
		
			if (state != hasTwoPair) goto pointTo;
		}
		
	// moveTo:
	        avg_area = get_pair_average_area(square_list, square_list->next);
		//printf("Average area = %d\n", avg_area);
		
		// 1500 +- 72  switched to 1400, it was moving too far forward in the square
		while(avg_area < 1328 || avg_area > 1472) { 
			printf("In moveTo State!\n");
			printf("Average area = %d\n", avg_area);
			if(avg_area < 1428) {
				printf("Too far back.  Moving forwards.\n");
				if (flag == 5){
					ri_move(ri, RI_MOVE_FORWARD, 9);
					//ri_move(ri, RI_STOP, 10);
				}
				else ri_move(ri, RI_MOVE_FORWARD, 7);
				//ri_move(ri, RI_STOP, 10);
			}
			else if(avg_area > 1572) {
				printf("Too far forward.  Moving backwards.\n");
				if (flag == 5){
					ri_move(ri, RI_MOVE_BACKWARD, 9);
					//ri_move(ri, RI_STOP, 10);
				}
 				else ri_move(ri, RI_MOVE_BACKWARD, 7);
				//ri_move(ri, RI_STOP, 10);
			}
			
			
			avg_area = get_pair_average_area(square_list, square_list->next);
			
			//find the squares list
			state = get_squares(ri, square_list, image, final_threshold, &slope_diff, x, y, robot_dir);
			
			if (state != hasTwoPair && flag == 4) goto pointTo;
		}
	
	// Release the square list data
	while(square_list != NULL) {
		sq_idx = square_list->next;
		free(square_list);
		square_list = sq_idx;
	}
	
	// put head down for future movement 
	ri_move(ri, RI_HEAD_DOWN, RI_FASTEST);
}
