/* Colors used by OpenCV */

#ifndef __RC_COLORS_H__
#define __RC_COLORS_H__

#define RC_LOW(x)	cvScalar(x - 5, 75, 75, 0)
#define RC_HIGH(x) 	cvScalar(x + 5, 255, 255, 0)

/* Pink */
#define RC_PINK		5
#define RC_PINK_LOW_1_bender	cvScalar(0, 60, 120, 0)		//bender right to left
#define RC_PINK_HIGH_1_bender 	cvScalar(11, 255, 255, 0)

#define RC_PINK_LOW_2_bender	cvScalar(150, 60, 120, 0)	//bender right to left
#define RC_PINK_HIGH_2_bender	cvScalar(190, 255, 255, 0)

#define RC_PINK_LOW_1_bender2	cvScalar(0, 60, 100, 0)		//bender left to right
#define RC_PINK_HIGH_1_bender2 	cvScalar(11, 255, 255, 0)

#define RC_PINK_LOW_2_bender2	cvScalar(150, 60, 100, 0)	//bender left to right
#define RC_PINK_HIGH_2_bender2	cvScalar(190, 255, 255, 0)

/* Pink */
#define RC_PINK		5
#define RC_PINK_LOW_1_optimus	cvScalar(0, 60, 100, 0)		//optimus
#define RC_PINK_HIGH_1_optimus 	cvScalar(11, 255, 255, 0)

#define RC_PINK_LOW_2_optimus	cvScalar(150, 60, 100, 0)	//optmius
#define RC_PINK_HIGH_2_optimus	cvScalar(190, 255, 255, 0)

#define RC_PINK_LOW_1_gort	cvScalar(0, 60, 90, 0)		//gort
#define RC_PINK_HIGH_1_gort	cvScalar(9, 255, 255, 0)

#define RC_PINK_LOW_2_gort	cvScalar(150, 67, 100, 0) 	//gort
#define RC_PINK_HIGH_2_gort	cvScalar(190, 255, 255, 0)

/* Yellow */
#define RC_YELLOW	30
#define RC_YELLOW_LOW	RC_LOW(RC_YELLOW)
#define RC_YELLOW_HIGH	RC_HIGH(RC_YELLOW)

/* Blue */
#define RC_BLUE		100
#define RC_BLUE_LOW	RC_LOW(RC_BLUE)
#define RC_BLUE_HIGH	RC_HIGH(RC_BLUE)

/* Green */
#define RC_GREEN	50
#define RC_GREEN_LOW	RC_LOW(RC_GREEN)
#define RC_GREEN_HIGH	RC_HIGH(RC_GREEN)

/* Purple */		
#define RC_PURPLE	140
#define RC_PURPLE_LOW	RC_LOW(RC_PURPLE)
#define RC_PURPLE_HIGH	RC_HIGH(RC_PURPLE)

#endif /* __RC_COLORS_H__ */
