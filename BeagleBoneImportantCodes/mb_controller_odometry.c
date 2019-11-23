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


float overallGain1_o;
float kp1_o;
float ki1_o;
float kd1_o;

float overallGain2_o;
float kp2_o; 
float ki2_o; 
float kd2_o;

float kp3_o;
float kd3_o;

float t;
float t_dot;
float p;
float p_dot;
float N_b;
float Ki;
float kpt;

float bodyAngleOffset_o;
float actVolt_o;
float to_travel_o;
float phi_error_prev=0;
float phi_error;
float integration=0;
float theta_error;
float head_error;

float k3b;
float k5b;

rc_filter_t F1_o = RC_FILTER_INITIALIZER;
rc_filter_t F2_o = RC_FILTER_INITIALIZER;
rc_filter_t F3_o = RC_FILTER_INITIALIZER;

int mb_controller_odometry_load_config(){

    FILE* file = fopen("ss_cnt.cfg", "r");
    if (file == NULL){
        printf("Error opening %s\n", CFG_PATH );
    }

    /* parse your config file here */
    
    // ====== State Space ===========
    fscanf(file, "%f", &t);
    fscanf(file, "%f", &t_dot);
    fscanf(file, "%f", &p);
    fscanf(file, "%f", &p_dot);
    fscanf(file, "%f", &N_b);
    fscanf(file, "%f", &Ki);
    fscanf(file, "%f", &kpt);
    fscanf(file, "%f", &to_travel_o);
    /*
    // ========== PID ==============
    
    fscanf(file, "%f", &overallGain1_o);
    fscanf(file, "%f", &kp1_o);
    fscanf(file, "%f", &ki1_o);
    fscanf(file, "%f", &kd1_o);
    fscanf(file, "%f", &overallGain2_o);
    fscanf(file, "%f", &kp2_o);
    fscanf(file, "%f", &ki2_o);
    fscanf(file, "%f", &kd2_o);
    fscanf(file, "%f", &kp3_o);
    fscanf(file, "%f", &kd3_o);
    fscanf(file, "%f", &bodyAngleOffset_o);
    fscanf(file, "%f", &actVolt_o);
    fscanf(file, "%f", &to_travel_o);

    kp1_o = kp1_o/overallGain1_o;
    ki1_o = ki1_o/overallGain1_o;
    kd1_o = kd1_o/overallGain1_o;

    kp2_o = kp2_o/overallGain2_o;
    ki2_o = ki2_o/overallGain2_o;
    kd2_o = kd2_o/overallGain2_o;
    
    // ===== Back Stepping ==========

    fscanf(file, "%f", &k5b);
    fscanf(file, "%f", &k3b);
    fscanf(file, "%f", &bodyAngleOffset_o);
    */
    
    fclose(file);
    return 0;
}


int mb_controller_init_odometry(){

    mb_controller_odometry_load_config();

   
    if(rc_filter_pid(&F1_o, kp1_o, ki1_o, kd1_o, 2*DT, DT)){
    fprintf(stderr, "ERROR in mb_controller, failed to make inner loop filter\n");
        return -1;
    }
    if(rc_filter_pid(&F2_o,  kp2_o, ki2_o, kd2_o, 2*DT, DT)){
    fprintf(stderr, "ERROR in mb_controller, failed to make outer loop filter\n");
        return -1;
    }
    
    if(rc_filter_pid(&F3_o, kp3_o, 0, kd3_o, 2*DT, DT)){
    fprintf(stderr, "ERROR in mb_controller, failed to make steering loop filter\n");
        return -1;
    }

   

    return 0;
}


int mb_controller_update_odometry(mb_state_t* mb_state, mb_odometry_t*mb_odometry){
    
    // ++++++++++++++++++++++++++++++++++++++++++++ PID CONTROLLER +++++++++++++++++++++++++++++ \\
    
    /*
    float u_1;
    float u;
    float phi_error;
    float theta_error;
    float head_error;

    mb_state->to_travel = to_travel_o;
    
    phi_error = mb_state->spPhi - mb_state->phi;
    mb_state->spTheta = rc_filter_march(&F2_o, phi_error) + bodyAngleOffset_o;

    //mb_state->spTheta = bodyAngleOffset_o;    
    
    theta_error = mb_state->spTheta - mb_state->theta;
    u_1 = rc_filter_march(&F1_o, theta_error);
    
    head_error = mb_state->spHeading - mb_odometry->heading;
    mb_state->diff = kp3_o*(head_error);
    
    u = u_1;
    
    if(-1.2<=u<=1.2){
   	mb_state->left_cmd = u-mb_state->diff;
    	mb_state->right_cmd = u+mb_state->diff;
    }

    else if (u<-1){	
   	mb_state->left_cmd =  -1;
    	mb_state->right_cmd = -1;
    }

    else if (u>1){
	mb_state->left_cmd = 1;
    	mb_state->right_cmd = 1;
    }
    */ 
    // ++++++++++++++++++++++++++++++ STATE SPACE CONTROLLER +++++++++++++++++++++++++++++++++++++++++++++++++++ \\

    
    mb_state->to_travel = to_travel_o;
    theta_error = mb_state->theta - 0.0;
    phi_error = mb_state->spPhi - mb_state->phi;
   
    integration = integration + (phi_error*DT);

    float K[4][1] = { {t},{t_dot}, {p},{p_dot} };
	
    float u   = 0.0;
    float u_1 = 0.0;
    float u_2 = 0.0;
    float u_3 = 0.0;
    float u_4 = 0.0;
    float u_t = 0.0;
    float N_bar = N_b;
    
    
    u_1=K[0][0]*mb_state->theta;
    u_2=K[1][0]*mb_state->thetadot;
    u_3=K[2][0]*mb_state->phi;
    u_4=K[3][0]*mb_state->phidot;

    u_t = u_1+u_2+u_3+u_4;

    u = (Ki*integration) + (-1*u_t);

    				       
    head_error = mb_state->spHeading - mb_odometry->heading;
    //mb_state->diff = rc_filter_march(&F3_o, head_error);
    mb_state->diff = kpt*(head_error);
               
						       
    //mb_state->diff = 0;
    mb_state->left_cmd = u - mb_state->diff;
    mb_state->right_cmd = u + mb_state->diff;
								
    
    
    // ++++++++++++++++++++++++++++++ BACK STEPPING CONTROLLER FOR SWING UP +++++++++++++++++++++++++++++++++++++++++++++++++++ \\
    
    /*
    float u =0;
    k3b = k3
    k5b = k4
    mb_state->spTheta = bodyAngleOffset_o; 
   
    float b1 =1;
    float b2 =0.02;
    float a1 =0.002;
    float a2 =0.0046;
    float a3 =0.018;
    float a4 =1.1317;
    
    e2 = mb_state.thetadot + k3*(mb_state.theta-mb_state.spTheta);
    
    num1 = (a4*a1-a2*a2*(cos(mb_state.theta))*mb_state.thetadot*mb_state.thetadot)* sin(mb_state.theta)
    num2 = (k4*e2+k3*mb_state.thetadot)*(a1*a3-a2*a2*(cos(mb_state.theta))(cos(mb_state.theta))
    num3 = b2/b1*(mb_state.phidot-mb_state.thetadot)
    den1 = (a1+a2*(cos(mb_state.theta)))*b1

    u = ((num1+num2)/den1)+num3
    mb_state->left_cmd = u;
    mb_state->right_cmd = u;
   
    */

    // ++++++++++++++++++++++++++++++++++++++++++++ MODEL ESTIMATION ++++++++++++++++++++++++++++++++++++++++++++++++++++++++ \\
    
    
    /*
    if (mb_state->theta >= 0.05 || mb_state->theta <= -0.05){
	
   	float u_1;
	float u;
    	float phi_error;
    	float theta_error;
    	float head_error;

    	mb_state->to_travel = to_travel_o;
    
    	phi_error = mb_state->spPhi - mb_state->phi;
    	mb_state->spTheta = rc_filter_march(&F2_o, phi_error) + bodyAngleOffset_o;

    	theta_error = mb_state->spTheta - mb_state->theta;
    	u_1 = rc_filter_march(&F1_o, theta_error);
    
    	head_error = mb_state->spHeading - mb_odometry->heading;
    	//mb_state->diff = kp3_o*(head_error);
	mb_state->diff = 0*(head_error);
   	u = u_1-mb_state->diff;

    	if(-1<=u<=1){
   		mb_state->left_cmd = u;
    		mb_state->right_cmd = u;
   	}

    	else if (u<-1){	
   		mb_state->left_cmd =  -1;
    		mb_state->right_cmd = -1;
    	}

    	else if (u>1){
		mb_state->left_cmd = 1;
    		mb_state->right_cmd = 1;
    	}
	}
   
   
    else {
	
    	mb_state->left_cmd = 0;
    	mb_state->right_cmd = 0;
	}

    */

    
    FILE* fp = fopen("box_cntrl_ssp.csv", "a");

  
    fprintf(fp, "%7.3f", mb_state->theta);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->thetadot);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->phi);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->phidot);
    fprintf(fp, "%s", ","); 
    fprintf(fp, "%7.3f", mb_state->spTheta);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->spPhi);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->dist);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", phi_error);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", phi_error_prev);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", integration);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", theta_error);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%d", mb_state->left_encoder);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%d", mb_state->right_encoder);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_odometry->x);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_odometry->y);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_odometry->heading);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->spHeading);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->diff);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->left_cmd);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->right_cmd);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->time);
    fprintf(fp, "%s", "\n");
    fclose(fp);
   
    
    //phi_error_prev = phi_error;

}


int mb_controller_odometry_cleanup(){
    rc_filter_free(&F1_o);
    rc_filter_free(&F2_o);
    rc_filter_free(&F3_o);

    return 0;
}
