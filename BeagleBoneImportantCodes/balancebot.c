/*******************************************************************************
* balancebot.c
*
* Main template code for the BalanceBot Project
* based on rc_balance
* 
*******************************************************************************/

#include <math.h>
#include <stdbool.h>
#include <rc/start_stop.h>
#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/mpu.h>
#include <rc/dsm.h>
#include <rc/cpu.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/pthread.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>
#include <math.h>
#include "balancebot.h"
#include "../common/mb_motor.h"
#include "../common/mb_defs.h"
#include "../common/mb_controller.h"
#include "../common/mb_controller_odometry.h"
#include "../common/mb_odometry.h"

#define INVALID -1
#define OBSTACLE 1
#define NORTH 10
#define SOUTH 100
#define EAST 1000
#define WEST 10000
#define X_SIZE 5
#define Y_SIZE 4
#define MAP_AREA 20
#define FREE 0
#define OBST 1
#define GOAL 2
#define PI 3.14159265

int dd=0;
int iter=0;
int turn=1;

//Cell Structure to hold x and y values
typedef struct
{
  int x;
  int y;
}Cell;

int world_map[4][5] = {
		      {0, 0, 0, 0, 0},
                      {0, 0, 1, 1, 0},
                      {0, 0, 1, 1, 0},
                      {0, 0, 1, 1, 0} };

int start_x = 1;
int start_y = 3;
int goal_x = 4;
int goal_y = 3;

int rear=0; i
int front=0;
int queue[MAP_AREA*2];

float time_start;
float ai = 1;

void enQueue(int x, int y);
void deQueue();
void display();
int isEmpty();
void generate();
//Given a cell, find out which neighbors are valid
void computeNeighbor(Cell currCell);
//Compute path for the map from finish to start
void computePath(int *map,int numCols,int numRows,int startX, int startY);
//Check to see if coordinates are non negative and therefore valid
bool checkCoordinate(int x, int y);

/*******************************************************************************
* int main() 
*
*******************************************************************************/

int main(){

    	// make sure another instance isn't running
    	// if return value is -3 then a background process is running with
    	// higher privaledges and we couldn't kill it, in which case we should
    	// not continue or there may be hardware conflicts. If it returned -4
    	// then there was an invalid argument that needs to be fixed.
    	
	if(rc_kill_existing_process(2.0)<-2) return -1;

    	// start signal handler so we can exit cleanly
    	if(rc_enable_signal_handler()==-1){
        	fprintf(stderr,"ERROR: failed to start signal handler\n");
        	return -1;
    	}

    	if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)<0){
        	fprintf(stderr,"Failed to set governor to PERFORMANCE\n");
        	return -1;
    	}

    	// initialize enocders
    	if(rc_encoder_eqep_init()==-1){
        	fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
        	return -1;
    	}

    	// initialize adc
    	if(rc_adc_init()==-1){
        	fprintf(stderr, "ERROR: failed to initialize adc\n");
        	return -1;
    	}

    	if(rc_dsm_init()==-1){
		fprintf(stderr,"failed to start initialize DSM\n");
		return -1;
    	}

	/*
    	printf("initializing xbee... \n");
    	//initalize XBee Radio
    	int baudrate = BAUDRATE;
    	if(XBEE_init(baudrate)==-1){
	fprintf(stderr,"Error initializing XBee\n");
		return -1;
   	};
	*/
        
	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident that there is no PID file already and we can
	// make our own safely.
     	
        rc_make_pid_file();

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
     	
	/*
        printf("Starting print thread... \n");
	pthread_t  printf_thread;
	rc_pthread_create(&printf_thread, printf_loop, (void*) NULL, SCHED_OTHER, 0);
	*/

	/*
	// start control thread
	printf("Starting Setpoint Thread... \n");
	pthread_t  setpoint_control_thread;
	rc_pthread_create(&setpoint_control_thread, setpoint_control_loop, (void*) NULL, SCHED_FIFO, 50);
	*/

	//start motion capture message recieve thread

	// set up IMU configuration

	printf("Initializing IMU... \n");

	// set up mpu configuration
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	mpu_config.orient = ORIENTATION_Z_DOWN;
	mpu_config.dmp_fetch_accel_gyro = 1;

	// now set up the imu for dmp interrupt operation
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}

	rc_nanosleep(3E9); // wait for imu to stabilize

	//initialize state mutex
        pthread_mutex_init(&state_mutex, NULL);
        pthread_mutex_init(&setpoint_mutex, NULL);

	//initialize state values that otherwise aren't initialized before referenced
	mb_state.left_encoder = 0;
	mb_state.right_encoder = 0;
	mb_state.I1 = 0;
	mb_state.I2 = 0;
	mb_state.diff = 0;
	mb_state.dist = 0;
	mb_state.spHeading = 0;
	mb_state.spPhi = 0;
	mb_state.to_travel = 0;
	mb_state.boxSide = 1;  
	mb_state.theta = 0;
	mb_state.time = 0;

	//attach controller function to IMU interrupt
	printf("Initializing Controller...\n");
	mb_controller_init_odometry();

	printf("Initializing Motors...\n");
	mb_motor_init();

	printf("Resetting Encoders...\n");
	rc_encoder_eqep_write(1, 0);
	rc_encoder_eqep_write(2, 0);

	printf("Initializing Odometry...\n");
	mb_odometry_init(&mb_odometry, 0.0,0.0,0.0);

	printf("Attaching IMU Interupt...\n");
	rc_mpu_set_dmp_callback(&balancebot_controller);

	printf("Yooo..!!..We Are Running!!!...\n");
	rc_set_state(RUNNING); 


	while(rc_get_state()!=EXITING){

		// all the balancing is handled in the imu interupt function
		// other functions are handled in other threads
		// there is no need to do anything here but sleep
		// always sleep at some point
		rc_nanosleep(1E9);
	}
	
	// exit cleanly
	rc_mpu_power_off();
	mb_motor_cleanup();
	mb_controller_cleanup();
	mb_controller_odometry_cleanup();
	rc_led_cleanup();
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file(); // remove pid file LAST 
	return 0;
}

// ***************** PLANNING FUNCTIONS START ************************************//

void enQueue(int x, int y)
{
  if(rear<sizeof(queue))
  {
     queue[rear] = x;
     queue[rear+1] = y;
     rear+=2;
  }
}

void deQueue()
{
     queue[front]=0;
     queue[front+1]=0;
     front+=2;
}

int isEmpty()
{
  int returnValue;
  if (front == rear) {
     returnValue = 0;
  }
  else { 
     returnValue = 1;
  }
  
  return returnValue;
}

//Variable for orientation of robot, default value is north
int orientation = NORTH;
//Variable to contain the directions to move to
int direction[MAP_AREA];
//Next best neighbor
Cell next;
//Neighbors of a cell
Cell northNeigh;
Cell southNeigh;
Cell eastNeigh;
Cell westNeigh;
//Values of neighbors
int northVal;
int southVal;
int eastVal;
int westVal;
// Next Cell 
Cell nextCell;

void generate()
{
  //Initialize goal cell to 2
  world_map[goal_y][goal_x] =2;

  //Put goal cell in the next
  nextCell.x = goal_x;
  nextCell.y = goal_y;
  enQueue(nextCell.x, nextCell.y); 

  while(rear != front)
  {
    //Take next cell from queue
    nextCell.x = queue[front];
    nextCell.y = queue[front+1];

    //Determine non obstacle neighbors
    Cell temp;
    temp.x = nextCell.x;
    temp.y = nextCell.y;
    computeNeighbor(temp);
    display();
    
    int y = queue[front+1];
    int x = queue[front];

    //If non obstacle cell value is 0

    if(northVal == 0)
    {
      world_map[northNeigh.y][northNeigh.x] = world_map[y][x]+1;
      enQueue(northNeigh.x,northNeigh.y);
      display();
    }

    if(southVal == 0)
    {
      world_map[southNeigh.y][southNeigh.x] = world_map[y][x]+1;
      enQueue(southNeigh.x,southNeigh.y);
      display();
    }

    if(eastVal == 0)
    {
      world_map[eastNeigh.y][eastNeigh.x] = world_map[y][x]+1;
      enQueue(eastNeigh.x,eastNeigh.y);
      display();
    }

    if(westVal == 0)
    {
      world_map[westNeigh.y][westNeigh.x] = world_map[y][x]+1;
      enQueue(westNeigh.x,westNeigh.y);
      display();
    }
    deQueue();
  }

  for(int i=0;i<Y_SIZE;i++)
  {
     for(int j=0;j<X_SIZE;j++)
     {
         printf("%d ", world_map[i][j]);
      }
         printf("\n");
  }

}

// ************************ COMPUTE NEIGHBOR *************************

void computeNeighbor(Cell currCell)
{
  northNeigh.x = currCell.x;
  northNeigh.y = currCell.y-1;
  //If coordinates of north neighbor are valid, northVal is the value of north neighbor, else it's invalid
  if (checkCoordinate(northNeigh.x,northNeigh.y)==true){
         northVal = world_map[northNeigh.y][northNeigh.x];
  }
  else {
         northVal = -1 ;
  }  

  southNeigh.x = currCell.x;
  southNeigh.y = currCell.y+1;
  if (checkCoordinate(southNeigh.x,southNeigh.y)==true) {
        southVal = world_map[southNeigh.y][southNeigh.x];
  }
  else {
         southVal = -1;
  }

  eastNeigh.x = currCell.x+1;
  eastNeigh.y = currCell.y;
  if (checkCoordinate(eastNeigh.x,eastNeigh.y)==true) {
         eastVal = world_map[eastNeigh.y][eastNeigh.x];
  }
  else {
    eastVal = -1; 
  }

  westNeigh.x = currCell.x-1;
  westNeigh.y = currCell.y;
  if (checkCoordinate(westNeigh.x,westNeigh.y)==true){ 
      westVal = world_map[westNeigh.y][westNeigh.x];
  }
  else {
      westVal = -1;
  }
}
 

//******************** COMPUTE PATH ************************************

void computePath(int *map, int numCols, int numRows, int startX, int startY)
{
 
  //Initialize start positions
  Cell currCell;
  currCell.x = startX;
  currCell.y = startY;
  int smallestCell;

  //compare all neighbors to find the smallest which is not OBST

  int i = 0;
  while(smallestCell != world_map[goal_y][goal_x])
  { 
    computeNeighbor(currCell); //Compute and set all possible neighbors
    //Check to make sure the neighbors are valid
    //Assign arbitrary smallest cell for comparison
    smallestCell = world_map[currCell.y][currCell.x];

    if(northVal < smallestCell && (northVal != 1 && northVal != -1))
     {
       smallestCell = northVal;
       //found best cell...set current cell to that cexl
       currCell.x = northNeigh.x;
       currCell.y = northNeigh.y;
       direction[i] = 1;
     }
    
     if(southVal < smallestCell && (southVal != 1 && southVal != -1))
     {
       smallestCell = southVal;
       //found best cell...set current cell to that cell
       currCell.x = southNeigh.x;
       currCell.y = southNeigh.y;
       direction[i] = 2;
     }
     if(eastVal < smallestCell && (eastVal != 1&& eastVal != -1))
     {
       smallestCell = eastVal;
       //found best cell...set current cell to that cell
       currCell.x = eastNeigh.x;
       currCell.y = eastNeigh.y;
       direction[i] = 3;
     }
     if(westVal < smallestCell && (westVal != 1 && westVal != -1))
     {
       smallestCell = westVal;
       //found best cell...set current cell to thay cell
       currCell.x = westNeigh.x;
       currCell.y = westNeigh.y;
       direction[i] = 4;
     }
     i++;
  }
  mb_state.value = i;
}

/******************** CHECK COORDINATES *************************************/

bool checkCoordinate(int x, int y)
{
  bool returnValue;
  int dummy = 0; 
  int dummy2 = 0;

  if(x == -1){ 
       returnValue = false;
   }
  else {
   dummy = dummy + 1;
   }   //If x is invalid, return value is false and increment dummy by 1


  if (y == -1) {
     returnValue = false; 
   }
  else{ 
     dummy = dummy + 2; 

   }   //If y is invalid return value is false and increment dummy by 2

  if (x>(X_SIZE-1)){
       returnValue = false; } 
  else {
   dummy2 += 3;
   }   //If x exceeds boundary return value is false, increment dummy2 by 3
  
  if (y>(Y_SIZE-1)){
       returnValue = false; }
  else { 
       dummy2 += 4; }//If y exceeds boundary return value is false, increment dummy2 by 4

  if(dummy==3 && dummy2==7)
  {
    return true;
  }    //If dummy is 3 and dumm2 is 7, then both tests have passed, coordinates are valid
  else 
  {
    return returnValue;
  }   //Otherwise tests failed, return false
}


// *********** PLANNING ENDS **************************************************

/*******************************************************************************
* void balancebot_controller()
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*******************************************************************************/

void balancebot_controller(){
   
   static int loopNum;
   time_start = (rc_nanos_since_boot()/1.0e9);
   mb_state.time = time_start;
   mb_setpoints.box_ctl = 1 ;
   //Lock state mutex
   pthread_mutex_lock(&state_mutex);
   //Read IMU
   //theta_prev = mb_state.theta;
   mb_state.theta = mpu_data.dmp_TaitBryan[TB_PITCH_X];
   mb_state.thetadot = mpu_data.gyro[0]*PI/180.0;
   // Read encoders
   mb_state.prev_left_enc = mb_state.left_encoder;
   mb_state.prev_right_enc = mb_state.right_encoder;
   mb_state.left_encoder = ENC_1_POL*rc_encoder_eqep_read(1);
   mb_state.right_encoder = ENC_2_POL*rc_encoder_eqep_read(2);
   mb_state.phi = (1/2.0)*(2*PI)*(mb_state.left_encoder + mb_state.right_encoder)/(GEAR_RATIO*ENCODER_RES);
   mb_state.speedLeft = (mb_state.left_encoder - mb_state.prev_left_enc)/(DT*GEAR_RATIO*ENCODER_RES);
   mb_state.speedRight = (mb_state.right_encoder - mb_state.prev_right_enc)/(DT*GEAR_RATIO*ENCODER_RES);
   mb_state.phidot = (1/2.0)*(2*PI)*(mb_state.speedLeft + mb_state.speedRight);
   mb_state.dist = (mb_state.phi * WHEEL_DIAMETER/2.0);	
        
   // Update odometry
   mb_odometry_update(&mb_odometry,&mb_state,2);

   if(!mb_setpoints.box_ctl){

     mb_state.spPhi += mb_state.to_travel;
     mb_state.dist = (mb_state.phi * WHEEL_DIAMETER/2.0);	
		
      
     if(mb_state.phi>mb_state.spPhi){
	mb_state.spPhi = mb_state.spPhi+mb_state.to_travel;
     }

     // Calculate controller outputs
     mb_controller_update_odometry(&mb_state, &mb_odometry);

     //send motor commands	
     //mb_motor_set(LEFT_MOTOR, 0);
     //mb_motor_set(RIGHT_MOTOR,0);
     mb_motor_set(LEFT_MOTOR, mb_state.left_cmd);
     mb_motor_set(RIGHT_MOTOR, mb_state.right_cmd);
   }
 
		
   if(mb_setpoints.box_ctl && dd == 0){
     printf("\nGenerating Path\n");		
     generate();
     computePath(*world_map,X_SIZE,Y_SIZE,start_x,start_y);
     dd=dd+1;
     }
    	
   if(mb_setpoints.box_ctl && dd ==1 && iter!= mb_state.value){
		
     //********************   Direction to move to: NORTH    *************** 
     //Orientation: NORTH
     if(direction[iter] == 1 && orientation == NORTH)
     {
        mb_state.spPhi += mb_state.to_travel;
        if mb_state.dist > (0.25*(iter+1)){
		iter=iter+1;	
	} 
     }
     //Orientation: SOUTH
     else if(direction[i] == 1 && orientation == SOUTH)
     {
        if (turn == 1){
                mb_state.spHeading -= PI/2;
	        turn=turn+1;
      	}
      	if (turn == 2){
         	mb_state.spHeading -= PI/2;
         	turn=turn+1;
      	}
    
      	if mb_state.dist > (0.35*(iter+1)){
	 	iter=iter+1;	
         	orientation = NORTH;//Reset Orientation
	 	turn=1;
      	}
    }
    //Orientation: EAST
    else if(direction[iter] == 1 && orientation == EAST)
    {
      	if(turn == 1){
	 	mb_state.spHeading += PI/2;
	 	turn=turn+1;
      	}

      	if mb_state.dist > (0.35*(iter+1)){
	 	orientation = NORTH; //Reset Orientation
	 	iter = iter+1;
	 	turn=1;
      	}
    }
    //Orientation: WEST
    else if(direction[iter] == 1 && orientation == WEST)
    {
      	if(turn == 1){
	 	mb_state.spHeading -= PI/2;
	 	turn=turn+1;
      	}

      	if mb_state.dist > (0.35*(iter+1)){
	 	orientation = NORTH; //Reset Orientation
	 	iter=iter+1;
	 	turn=1;
    	}
    }
    
    //********************  Direction to move to: SOUTH    ***************

    //Orientation: SOUTH
    if(direction[iter] == 2 && orientation == SOUTH)
    { 
      
      	if mb_state.dist > (0.25*(iter+1)){ //Reset Orientation
	 	iter=iter+1;
    	}
    }
    //Orientation: NORTH
    else if(direction[iter] == 2 && orientation == NORTH)
    {
	if (turn == 1){
         	mb_state.spHeading -= PI/2;
	 	turn=turn+1;
      	}
      	if (turn == 2){
         	mb_state.spHeading -= PI/2;
         	turn=turn+1;
      	}
      
      	if mb_state.dist > (0.35*(iter+1)){
        	orientation = SOUTH;//Reset Orientation
	  	iter=iter+1;
	  	turn=1;
      	}
    }
    
    //Orientation: EAST
    else if(direction[iter] == 2 && orientation == EAST)
    {
      	if(turn == 1){
	 	mb_state.spHeading -= PI/2;
	 	turn=turn+1;
      	}
      	if mb_state.dist > (0.35*(iter+1)){
	     	orientation = SOUTH;//Reset Orientation
	     	iter=iter+1
	     	turn=1;
      	}
    }
    
    //Orientation: WEST
    else if(direction[iter] == 2 && orientation == WEST)
    {
      	if(turn == 1){
	 	mb_state.spHeading += PI/2;
	 	turn=turn+1;
      	}
    
      	if mb_state.dist > (0.35*(iter+1)){
	 	orientation = SOUTH;//Reset Orientation
	 	iter=iter+1;
	 	turn=1;
      	}
    }

    // ******************** Direction to move to: EAST     ***************
    //Orientation: EAST
    if(direction[i] == 3 && orientation == EAST)
    {
      	
      	if mb_state.dist > (0.25*(iter+1)){
	     	iter=iter+1;
	}
    }
    //Orientation: NORTH
    else if(direction[iter] == 3 && orientation == NORTH)
    {
      	if(turn == 1){
	 	mb_state.spHeading -= PI/2;
	 	turn=turn+1;
      	}
  
      	if mb_state.dist > (0.35*(iter+1)){
	 	orientation = EAST;//Reset Orientation
	 	iter=iter+1;
	 	turn=1;
      	}
    }
    //ORIENTATION: SOUTH
    else if(direction[iter] == 3 && orientation == SOUTH)
    {
      	if(turn == 1){
	 	mb_state.spHeading += PI/2;
	 	turn=turn+1;
      	}
      
 
      	if mb_state.dist > (0.35*(iter+1)){
	 	orientation = EAST;//Reset Orientation
	 	iter=iter+1;
	 	turn=1;
      	}
    }
    //ORIENTATION: WEST
    else if(direction[iter] == 3 && orientation == WEST)
    {
      	if (turn == 1){
         	mb_state.spHeading -= PI/2;
	 	turn=turn+1;
     	}

      	if (turn == 2){
         	mb_state.spHeading -= PI/2;
         	turn=turn+1;
      	}
      	
	mb_state.spPhi += mb_state.to_travel;
      	if mb_state.dist > (0.35*(iter+1)){
          	orientation = EAST;//Reset Orientation
	  	iter=iter+1;
	  	turn=1;
      	}
    }

    //********************  Direction to move to: WEST     ***************
    //Orientation: WEST
    if(direction[iter] == 4 && orientation == WEST)
    {
   
      	if mb_state.dist > (0.25*(iter+1)){
		iter=iter+1;
	}
    }
    //Orientation: NORTH
    else if(direction[iter] == 4 && orientation == NORTH)
    {
      	if(turn == 1){
	 	mb_state.spHeading += PI/2;
	 	turn=turn+1;
      	}

      	if mb_state.dist > (0.35*(iter+1)){
	 	orientation = WEST;//Reset Orientation
	 	iter=iter+1;
	 	turn=1;
      	}
    }
    //Orientation: WEST
    else if(direction[iter] == 4 && orientation == SOUTH)
    {
      	if(turn == 1){
	 	mb_state.spHeading -= PI/2;
	 	turn=turn+1;
      	}

      
      	if mb_state.dist > (0.35*(iter+1)){
	 	orientation = WEST;//Reset Orientation
	 	iter=iter+1;
	 	turn=1;
      	}
    }
    //Orientation: EAST
    else if(direction[iter] == 4 && orientation == EAST)
    {
      	if (turn == 1){
         	mb_state.spHeading += PI/2;
	 	turn=turn+1;
      	}

      	if (turn == 2){
         	mb_state.spHeading += PI/2;
         	turn=turn+1;
      	}

     
      	if mb_state.dist > (0.35*(iter+1)){
          	orientation = NORTH;//Reset Orientation
	  	iter=iter+1;
	  	turn=1;
      	}
    }	
    
    mb_state.spPhi += mb_state.to_travel;
    mb_controller_update_odometry(&mb_state, &mb_odometry);
    mb_motor_set(LEFT_MOTOR, mb_state.left_cmd);
    mb_motor_set(RIGHT_MOTOR, mb_state.right_cmd);
 }
    /* 
    if(mb_setpoints.box_ctl){
		
    	mb_state.spPhi += mb_state.to_travel;
    	float dist = 0.95;
    	if ((mb_state.boxSide == 1) && (mb_odometry.x > dist - WHEEL_BASE/2.0))
    	{
		mb_state.spHeading += PI/2.0;
		mb_state.boxSide = 2;
    	}
     	else if (mb_state.boxSide == 2 && mb_odometry.y > dist - WHEEL_BASE/2.0)
     	{
     		mb_state.spHeading += PI/2.0;
		mb_state.boxSide = 3;
     	}
     	else if (mb_state.boxSide == 3 && mb_odometry.x < 0 + WHEEL_BASE/2.0)
     	{
		mb_state.spHeading += PI/2.0;
		mb_state.boxSide = 4;
      	}
      	else if (mb_state.boxSide == 4 && mb_odometry.y < 0 + WHEEL_BASE/2.0)
      	{
		mb_state.spHeading += PI/2.0;
		mb_state.boxSide = 1;
      	}
        
	mb_controller_update_odometry(&mb_state, &mb_odometry);
    	mb_motor_set(LEFT_MOTOR, mb_state.left_cmd);
    	mb_motor_set(RIGHT_MOTOR, mb_state.right_cmd);
		
      }*/
        

       /*
       XBEE_getData();
       double q_array[4] = {xbeeMsg.qw, xbeeMsg.qx, xbeeMsg.qy, xbeeMsg.qz};
       double tb_array[3] = {0, 0, 0};
       rc_quaternion_to_tb_array(q_array, tb_array);
       mb_state.opti_x = xbeeMsg.x;
       mb_state.opti_y = -xbeeMsg.y;	    //xBee quaternion is in Z-down, need Z-up
       mb_state.opti_roll = tb_array[0];
       mb_state.opti_pitch = -tb_array[1]; //xBee quaternion is in Z-down, need Z-up
       mb_state.opti_yaw = -tb_array[2];   //xBee quaternion is in Z-down, need Z-up
       */
	
       //unlock state mutex
       pthread_mutex_unlock(&state_mutex);
       
       loopNum += 1;
}


/*******************************************************************************
*  setpoint_control_loop()
*  sets current setpoints based on dsm radio data, odometry, and Optitrak
*******************************************************************************/

/*
void* setpoint_control_loop(void* ptr){

	while(1){


		if(rc_dsm_is_new_data()){
				// Handle the DSM data from the Spektrum radio reciever
				// You may should implement switching between manual and autonomous mode
				// using channel 5 of the DSM data.

				float deadZoneWidth = 0.2;
				float manSwitch = rc_dsm_ch_normalized(5);
				float boxSwitch = rc_dsm_ch_normalized(7); 
				float tVel;
				float fVel;
				if (manSwitch < 0.1)
					mb_setpoints.manual_ctl = 0;
				else if(manSwitch > 0.1)
					mb_setpoints.manual_ctl = 1;
				if(boxSwitch < 0.1)
					mb_setpoints.box_ctl = 0;
				else if(boxSwitch > 0.1)
					mb_setpoints.box_ctl = 1;
				
				if(mb_setpoints.manual_ctl){

					tVel = rc_dsm_ch_normalized(4);
					fVel = rc_dsm_ch_normalized(3);

					if(fabs(tVel) <= deadZoneWidth)
						tVel = 0;
					else {
						if(tVel < 0)
							tVel = tVel + deadZoneWidth;
						else
							tVel = tVel - deadZoneWidth;
						
					}

					if(fabs(fVel) <= deadZoneWidth)
						fVel = 0;
					else {
						if(fVel < 0)
							fVel = fVel + deadZoneWidth;
						else
							fVel = fVel - deadZoneWidth;
						
					}

					mb_setpoints.turn_velocity = tVel;
					mb_setpoints.fwd_velocity = fVel;
				}

		}
	 	rc_nanosleep(1E9 / RC_CTL_HZ);
	}
	return NULL;
}



******************************************************************************
* printf_loop() 
* prints diagnostics to console
* this only gets started if executing from terminal
******************************************************************************/

/*
void* printf_loop(void* ptr){
	rc_state_t last_state, new_state; // keep track of last state
	while(rc_get_state()!=EXITING){
		new_state = rc_get_state();

		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("                 SENSORS               |                                     PID		                       	|");
			printf("\n");
			printf("    θ    |");
			printf("   θset  |");
			printf("    φ    |");
			printf("   diff  |");
			printf(" sp Dist |");
			printf("  Dist   |");
			printf("  L Enc  |");
			printf("  R Enc  |");
			printf("  DutyL  |");
			printf("  DutyR  |");
			printf("    X    |");
			printf("    Y    |");
			printf("    ψ    |");
			printf("   ψset  |");
			printf(" boxSide |");

			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		last_state = new_state;
		
		if(new_state == RUNNING){
			printf("\r");
			//Add Print stattements here, do not follow with /n
			pthread_mutex_lock(&state_mutex);
			printf("%7.3f  |", mb_state.theta);
			printf("%7.3f  |", mb_state.thetadot);
			printf("%7.3f  |", mb_state.spTheta);
			printf("%7.3f  |", mb_state.phi);
			printf("%7.3f  |", mb_state.diff);
			printf("%7.3f  |", mb_state.spDist);
			printf("%7.3f  |", mb_state.dist);
			printf("%7d  |", mb_state.left_encoder);
			printf("%7d  |", mb_state.right_encoder);
			printf("%7.3f  |", mb_state.left_cmd);
			printf("%7.3f  |", mb_state.right_cmd);
			printf("%7.3f  |", mb_state.opti_x);
			printf("%7.3f  |", mb_state.opti_y);
			printf("%7.3f  |", mb_state.opti_yaw);
			printf("%7.3f  |", mb_state.spHeading);
			printf("%7d  |", mb_state.boxSide);
			pthread_mutex_unlock(&state_mutex);
			fflush(stdout);
		}
		rc_nanosleep(1E9/PRINTF_HZ);
	}
	return NULL;
}
*/

