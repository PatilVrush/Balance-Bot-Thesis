/*******************************************************************************
* measure_motors.c
*
* Use this template to write data to a file for analysis in Python or Matlab
* to determine the parameters for your motors
*
* TODO: Capture encoder readings, current readings, timestamps etc. to a file
*       to analyze and determine motor parameters
*
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>	// for mkdir and chmod
#include <sys/types.h>	// for mkdir and chmod
#include <rc/start_stop.h>
#include <rc/cpu.h>
#include <rc/encoder_eqep.h>
#include <rc/adc.h>
#include <rc/time.h>
#include "../common/mb_motor.h"
#include "../common/mb_defs.h"

FILE* f1;
FILE* fp;

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

    // initialize motors
    if(mb_motor_init()<0){
        fprintf(stderr,"ERROR: failed to initialze mb_motors\n");
        return -1;
    }

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();


    rc_set_state(RUNNING);


    //mb_motor_set(RIGHT_MOTOR, dc);
    //mb_motor_set(LEFT_MOTOR, dc);


    //while(rc_get_state()!=EXITING){
    
	int sa = 5000; //No of samples in the chirp_hyperbolic.csv file you pass 
        int mn;
	int i;
	int j;
	float* enc1; 
	float* enc2; 
    	float* curr1;
	float* curr2;
	float* time1;
	float* time2;
	float* volt;
	float k;
	int polarity;
	enc1 = (float*)malloc(sa * sizeof(float));
	enc2 = (float*)malloc(sa * sizeof(float)); 
	time1 = (float*)malloc(sa * sizeof(float));
	time2 = (float*)malloc(sa * sizeof(float));
	curr1 = (float*)malloc(sa * sizeof(float)); 
	curr2 = (float*)malloc(sa * sizeof(float)); 
	volt = (float*)malloc(sa * sizeof(float)); 
/*
	FILE *myFile;

  	float fValues[sa];
	int n=0,cnt = 0;
	myFile = fopen("chirp_hyperbolic.csv", "r");
	if (myFile == NULL){
		printf("Failed to open signal file\n");
		return 1;
	}
	while (fscanf(myFile, "%f", &fValues[n++]) ==1){
  	fscanf(myFile, ",");
	}
*/
	while(1)
	{

    	for(j=0;j<sa;j=j+1){
	  //k=fValues[j];  //Use this if reading data from chirp_hyperbolic.csv file

	  k = (0.5+0.5*sin(j));
	  mb_motor_set(RIGHT_MOTOR, k);
	  mb_motor_set(LEFT_MOTOR, k);
          for(i=1;i<=2;i++){

            if (i == 1){
                polarity = ENC_1_POL;
		time1[j] = (rc_nanos_since_boot()/1.0e9);
		enc1[j] = rc_encoder_eqep_read(i)*polarity;
		curr1[j] = mb_motor_read_current(LEFT_MOTOR);
		volt[j]=k;
            }
            else
            {
                polarity = ENC_2_POL;
		time2[j] = (rc_nanos_since_boot()/1.0e9);
		enc2[j] = rc_encoder_eqep_read(i)*polarity;
		curr2[j] = mb_motor_read_current(RIGHT_MOTOR);
            }
	}
	    //usleep(1000000);
      }
	printf("\nBreaking\n");
	break;
    }

printf("Total time taken: %f" , (time1[(sa-1)]-time1[0]));

FILE *fp = fopen("mtd_sin_ns3.csv", "w+");
if(fp == NULL){

    printf("ERROR - Failed to open file for writing\n");
    exit(1);
}

// Write Buffer
 if (fp){
	 for(mn=0;mn<sa;mn=mn+1){
		fprintf(fp, "%f", time1[mn]);
         	fprintf(fp, "%s", "   |   ");
         	fprintf(fp, "%f", enc1[mn]);
         	fprintf(fp, "%s", "   |   "); 
         	fprintf(fp, "%f", time2[mn]);
         	fprintf(fp, "%s", "   |   ");
		fprintf(fp, "%f", enc2[mn]);
         	fprintf(fp, "%s", "   |   "); 
		
        	fprintf(fp, "%f", curr1[mn]);
        	fprintf(fp, "%s", "|  ");
        	fprintf(fp, "%f", curr2[mn]); 
        	fprintf(fp, "%s", "|  ");
		fprintf(fp, "%f", volt[mn]);
		fprintf(fp, "%s", "\n");
 	}
  }
    else{
    printf("Can't open file for write");
	}

//fclose(myFile);
fclose(fp);
fp = NULL;
free(curr1);
free(curr2);
free(enc1);
free(enc2);
free(time1);
free(time2);
//return 0;     
     

   // }

rc_adc_cleanup();
rc_encoder_eqep_cleanup();
mb_motor_cleanup();
rc_remove_pid_file();   // remove pid file LAST
return 0;
}
