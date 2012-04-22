#include "path_planning.h"

#define ROWS 5
#define COLS 7
/* USE THIS DEFINE TO ACCESS MAP[i][j] 
 * ex:	array2D(map,i,j).type */
#define array2D(b,i,j) b[(i)*COLS+(j)]

int 		x,		//current position
		y,
		robotID,// Used to be MINE
		score1, 
		score2,
		firstRun,
		pointsInMap;
		//spacestosum;
array_map_obj_t *map;
robot_heading	facing;

/*
int sumCrawler(*map_obj_t backPointer){//helper method for sumPoints
	//code me plz
}
*/
/*
int sumPoints(int x, int cellsToCrawl){
	//code me plz
}
*/

int getRow(){
	return y;
}

int getCol(){
	return x;
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
			//turn180 left or right
		}
		else if((facing==HEADING_LEFT)&&(where_to_go==HEADING_RIGHT)){
			//turn180 left or right
		}
		else if((facing==HEADING_RIGHT)&&(where_to_go==HEADING_LEFT)){
			//turn180 left or right
		}
		else if((facing==HEADING_DOWN)&&(where_to_go==HEADING_UP)){
			//turn180 left or right
		}
		else if((facing==HEADING_DOWN)&&(where_to_go==HEADING_LEFT)){
			//turn 90 right
		}
		else if((facing==HEADING_LEFT)&&(where_to_go==HEADING_UP)){
			//turn 90 right
		}
		else if((facing==HEADING_UP)&&(where_to_go==HEADING_RIGHT)){
			//turn 90 right
		}
		else if((facing==HEADING_RIGHT)&&(where_to_go==HEADING_DOWN)){
			//turn 90 right
		}
		else if((facing==HEADING_DOWN)&&(where_to_go==HEADING_RIGHT)){
			//turn 90 left
		}
		else if((facing==HEADING_UP)&&(where_to_go==HEADING_LEFT)){
			//turn 90 left
		}
		else if((facing==HEADING_LEFT)&&(where_to_go==HEADING_DOWN)){
			//turn 90 left
		}
		else if((facing==HEADING_RIGHT)&&(where_to_go==HEADING_UP)){
			//turn 90 left
		}
		facing = where_to_go;//set updated heading
	}
	
	//move forward 1 space //fill me in
	
	// center
	if(pairsToExpect(facing, where_to_go)==2){
	      //center with 2 pairs;//fill me in
	}
	if(pairsToExpect(facing, where_to_go)==1){
	      //center with 1 pair;//fill me in
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


int main(int argv, char **argc) {
        robot_if_t ri;
	score1 = 0;
        score2 = 0;
	firstRun = 1;
	int i, j;
	//spacestosum = 3;
	// initialize memory for the GLOBAL variable map
	map = (array_map_obj_t*) malloc(sizeof(array_map_obj_t) * ROWS * COLS);

        // Make sure we have a valid command line argument
        if(argv <= 2) {
                printf("Usage: robot_game_example <address of robot> <starting position(1 or 2)\n");
                exit(-1);
        }

	robotID = (int)strtol ( argc[2], NULL, 0 );

        // Setup the robot with the address passed in
        // This robot has been configured to be Robot 1. needs to be flexible. 
	if(ri_setup(&ri, argc[1], robotID)) {
                printf("Failed to setup the robot!\n");
                exit(-1);
        }
        
        // IDs based on code Jun wrote for setting up thresholding, should also fit in with your implementation
        if(robotID == 1){//set up initial position
	    y = 2;
	    x = 0;
	    facing = HEADING_RIGHT;
	}
	else {
	    y = 2;
	    x = 6;
	    facing = HEADING_LEFT;
	}
	
	printf("Robot ID = %d\tx = %d, y = %d\n", robotID, x, y);
	
	// run the game until score is >= 25 so you can make some reservations and moves
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
		
		
		
		getc(stdin);
		
	}
        
	free(map);
	
        return 0;
	
}
