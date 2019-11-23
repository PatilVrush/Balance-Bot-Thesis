#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <rc/math/filter.h>
#include "mb_controller.h"
#include "mb_defs.h"
#include "mb_structs.h"

/*******************************************************************************
* int mb_controller_init()
*
* this initializes the controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/
float overallGain1 = 1;
float kp1 = 0;
float ki1;
float kd1;

float overallGain2 = 1;
float kp2; 
float ki2; 
float kd2;

float kp3;
float kd3;


float bodyAngleOffset;

float actVolt = 12.0;


rc_filter_t F1 = RC_FILTER_INITIALIZER;
rc_filter_t F2 = RC_FILTER_INITIALIZER;
rc_filter_t FI1 = RC_FILTER_INITIALIZER;
rc_filter_t FI2 = RC_FILTER_INITIALIZER;


int mb_controller_init(){
    mb_controller_load_config();
    /* TODO initialize your controllers here*/
    // set up inner loop PID controller
    if(rc_filter_pid(&F1, kp1, 0, kd1, 2*DT, DT)){
        fprintf(stderr, "ERROR in mb_controller, failed to make inner loop filter\n");
        return -1;
    }
    if(rc_filter_pid(&F2, kp2, 0, kd2, 2*DT, DT)){
        fprintf(stderr, "ERROR in mb_controller, failed to make outer loop filter\n");
        return -1;
    }

    if(rc_filter_pid(&FI1, 0, ki1, 0, 2*DT, DT)){
        fprintf(stderr, "ERROR in mb_controller, failed to make inner loop integral filter\n");
        return -1;
    }

    if(rc_filter_pid(&FI2, 0, ki2, 0, 2*DT, DT)){
        fprintf(stderr, "ERROR in mb_controller, failed to make outer loop integral filter\n");
        return -1;
    }


    return 0;
}

/*******************************************************************************
* int mb_controller_load_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_controller_load_config(){
    FILE* file = fopen("pid.cfg", "r");
    if (file == NULL){
        printf("Error opening %s\n", CFG_PATH );
    }
    /* TODO parse your config file here*/

    fscanf(file, "%f", &overallGain1);
    fscanf(file, "%f", &kp1);
    fscanf(file, "%f", &ki1);
    fscanf(file, "%f", &kd1);
    fscanf(file, "%f", &overallGain2);
    fscanf(file, "%f", &kp2);
    fscanf(file, "%f", &ki2);
    fscanf(file, "%f", &kd2);
    fscanf(file, "%f", &kp3);
    fscanf(file, "%f", &kd3);
    fscanf(file, "%f", &bodyAngleOffset);
    fscanf(file, "%f", &actVolt);


    kp1 = kp1/overallGain1;
    ki1 = ki1/overallGain1;
    kd1 = kd1/overallGain1;

    kp2 = kp2/overallGain2;
    ki2 = ki2/overallGain2;
    kd2 = kd2/overallGain2;


    fclose(file);
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
* 
* 
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* this should only be called in the imu call back function, no mutex needed
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints){
    /*TODO: Write your controller here*/
    float val1;
    float val2;

    if(!mb_setpoints->manual_ctl){

        val1 = rc_filter_march(&F2, mb_state->spPhi - mb_state->phi);
        val2 = rc_filter_march(&FI2,  mb_state->spPhi - mb_state->phi);
        
        if(mb_state->theta < -0.4 || mb_state->theta > 0.4){ //reset the outer loop integral term if the robot falls over (to prevent wind-up)
            val2 = 0;
            rc_filter_reset(&FI2);
        }
	
       mb_state->spTheta = val1 + val2 + bodyAngleOffset;
    	
        //mb_state->spTheta = bodyAngleOffset;
    }
    else{
        mb_state->spTheta = mb_setpoints->fwd_velocity + bodyAngleOffset;

    }
    

    val1 = rc_filter_march(&F1, mb_state->spTheta - mb_state->theta);
    val2 = rc_filter_march(&FI1, mb_state->spTheta - mb_state->theta);
    
    if(mb_state->theta < -0.4 || mb_state->theta > 0.4){ //reset the inner loop integral term if the robot falls over
        val2 = 0;
        rc_filter_reset(&FI1);
    }
     
    float duty = (val1 + val2)*(NOM_VOLTAGE/actVolt); //This accounts for the actual voltage being less than the nominal that we used to measured the motor parameters

    mb_state->diff = kp3*mb_setpoints->turn_velocity + kd3*(mb_state->speedLeft - mb_state->speedRight)/2.0;

    if(mb_state->theta < -0.4 || mb_state->theta > 0.4) //if the robot is tipped over, turn the motors off
    {
        mb_state->left_cmd = 0;
        mb_state->right_cmd = 0;
    }
    else if (mb_setpoints->manual_ctl)
    {
        mb_state->left_cmd = duty - mb_state->diff;
        mb_state->right_cmd = duty + mb_state->diff;
    }
    else
    {
        mb_state->left_cmd = duty; 
        mb_state->right_cmd = duty;   
    }
    
   
    /*
    FILE* fp = fopen("pidData_vpin.csv", "a");
    
    fprintf(fp, "%f", kp1);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%f", ki1);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->theta);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->thetadot);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->P1);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->I1);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->D1);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->torque);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->left_cmd);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->right_cmd);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%d", mb_state->left_encoder);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%d", mb_state->right_encoder);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->spTheta);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_setpoints->turn_velocity);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->diff);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->speedLeft);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->speedRight);
    fprintf(fp, "%s", "\n");
    fclose(fp);
    */
    return 0;
}



/*******************************************************************************
* int mb_controller_cleanup()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_cleanup(){
    rc_filter_free(&F1);
    rc_filter_free(&F2);
    rc_filter_free(&FI1);
    rc_filter_free(&FI2);

    return 0;
}
