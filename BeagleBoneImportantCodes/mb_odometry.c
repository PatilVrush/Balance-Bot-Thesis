/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry functionality 
*
*******************************************************************************/
#include "../balancebot/balancebot.h"
#include "../common/mb_defs.h"
#include <rc/encoder_eqep.h>
#include <math.h>
#include <rc/math.h>
#include <rc/time.h>
#define PI 3.14159265

static double thres = 0.001;

void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y, float theta){
	mb_odometry->x = x;
	mb_odometry->y = y;
	mb_odometry->heading = theta;
}

/* phase_mode=1: only use encoder 
   phase_mode=2: use encoder and gyro */

void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state, int phase_mode){
    
    	double dtheta,dx,dy;
    	double d_gyro,d_odo;
	
	// get rotated angles for both wheels
	double dphi_l = (2*PI)*(mb_state->left_encoder-mb_state->prev_left_enc)/(GEAR_RATIO*ENCODER_RES);       
	double dphi_r = (2*PI)*(mb_state->right_encoder-mb_state->prev_right_enc)/(GEAR_RATIO*ENCODER_RES);

	// calculate displacement
	double ds_l = dphi_l * WHEEL_DIAMETER/2.0;
	double ds_r = dphi_r * WHEEL_DIAMETER/2.0;
	double ds = (ds_l + ds_r)/2.0;


	if (phase_mode==1){
		dtheta = (ds_r - ds_l)/WHEEL_BASE;
		dx = ds * cos(mb_odometry->heading + dtheta);
		dy = ds * sin(mb_odometry->heading + dtheta);
		mb_odometry->x += dx;
	    	mb_odometry->y += dy;
	    	mb_odometry->heading += dtheta;
	}

	else if (phase_mode==2){
		d_odo = (ds_r - ds_l)/WHEEL_BASE;
		d_gyro = (mpu_data.gyro[2]*PI/180)*DT;
		if (fabs(d_gyro - d_odo)>thres) {
			dtheta = d_gyro;
		//	printf("Taking Gyro\n");
		} 
		else {
			dtheta = d_odo;
		//	printf("Taking Odo\n");
		}
		dx = ds * cos(mb_odometry->heading + dtheta);
		dy = ds * sin(mb_odometry->heading + dtheta);
		mb_odometry->x += dx;
	    	mb_odometry->y += dy;
	    	mb_odometry->heading += dtheta;
	}
    	
//	mb_state->dist = ds;

}

float mb_clamp_radians(float angle){
	if (angle > PI){
		angle -= 2*PI;
	}
	else if (angle < -PI){
		angle += 2*PI;
	}
    return 0;
}
