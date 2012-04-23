CFLAGS=-ggdb -g3 -Wall
LIB_FLAGS=-L. -lrobot_if
LIB_LINK=-lhighgui -lcv -lcxcore -lm -lgslcblas -L/usr/lib64/atlas -lclapack -lrt

PROGS= cal pathfinding rovioman

all: ${PROGS}

rovioman: rovioman.o northstar.o wheel_encoder.o filter.o position.o rovioKalmanFilter.o PID_Control.o robot_vision.o
	gcc ${CFLAGS} -o rovioman rovioman.o position.o northstar.o wheel_encoder.o filter.o matvec.o rovioKalmanFilter.o PID_Control.o robot_vision.o  ${LIB_FLAGS} ${LIB_LINK}

pathfinding: path_planning.o
	gcc ${CFLAGS} -o robot_game_example path_planning.o ${LIB_FLAGS} ${LIB_LINK}

cal: theta_cal.o northstar.o wheel_encoder.o filter.o position.o
	gcc ${CFLAGS} -o cal theta_cal.o position.o northstar.o wheel_encoder.o filter.o matvec.o rovioKalmanFilter.o ${LIB_FLAGS} ${LIB_LINK}

path_planning.o: path_planning.c path_planning.h
	gcc ${CFLAGS} -c path_planning.c ${LIB_FLAGS}

robot_vision.o: robot_vision.c robot_vision.h robot_color.h
	gcc ${CFLAGS} -c robot_vision.c ${LIB_FLAGS}

theta_cal.o: theta_cal.c position.o matvec.o rovioKalmanFilter.o
	gcc ${CFLAGS} -c theta_cal.c
	
rovioman.o: rovioman.c position.o matvec.o path_planning.h
	gcc ${CFLAGS} -c rovioman.c
	
position.o: position.c northstar.o wheel_encoder.o matvec.o rovioKalmanFilter.o position.h kalmanFilterDef.h
	gcc ${CFLAGS} -c position.c

northstar.o: northstar.c matvec.o northstar.h
	gcc ${CFLAGS} -c northstar.c

wheel_encoder.o: wheel_encoder.c matvec.o wheel_encoder.h
	gcc ${CFLAGS} -c wheel_encoder.c

filter.o: filter.c filter.h
	gcc ${CFLAGS} -c filter.c

PID_Control.o: PID_Control.c PID_Control.h
	gcc ${CFLAGS} -c PID_Control.c

matvec.o: matvec.c matvec.h
	gcc ${CFLAGS} -c matvec.c

rovioKalmanFilter.o: rovioKalmanFilter.c
	gcc ${CFLAGS} -c rovioKalmanFilter.c

clean:
	rm -rf *.o *~ *.orig
	rm -rf ${PROGS}