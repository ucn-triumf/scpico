/********************************************************************\
  Name:         scpico.cxx
  Created by:   PAA
  $Id$
                Monitor the current through the Keithley picommeter
                using the GPIB/MSCB interface
\********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "midas.h"
#include "mscb.h"
#include <unistd.h>
#include <math.h>
//#include "fonctions_pico.h"

/* make frontend functions callable from the C framework */
#ifdef __cplusplus
extern "C" {
#endif

  /*-- Globals -------------------------------------------------------*/


  /* The frontend name (client name) as seen by other MIDAS clients   */
  char const *frontend_name = "scPico";
  /* The frontend file name, don't change it */
  char const *frontend_file_name = __FILE__;

  /* frontend_loop is called periodically if this variable is TRUE    */
  BOOL frontend_call_loop = TRUE;

  /* a frontend status page is displayed with this frequency in ms */
  INT display_period = 0000;

/*polarity of jonctions. 1 or -1 */
  INT begin_run=0;

/*polarity of jonctions. 1 or -1 */
  INT polarity = 0;

  /* maximum event size produced by this frontend */
  INT max_event_size = 10000;

  /* maximum event size for fragmented events (EQ_FRAGMENTED) */
  INT max_event_size_frag = 5 * 1024 * 1024;

  /* buffer size to hold events */
  INT event_buffer_size = 100 * 10000;

  char const mydevname[] = {"GPIB410"};
  HNDLE hDB, hDD, hSet, hControl;

#define PICO_SETTINGS_STR(_name) char const *_name[] = {\
    "[DD]",\
    "MSCB Device1 = STRING : [32] mscbfff.triumf.ca",\
    "MSCB Pwd = STRING : [32] ",\
    "Base Address1 = INT : 1",\
    "Base Address2 = INT : 2",\
    "",\
    "[.]",\
    "Names = STRING[3] :",\
     "[32] Current",\
     "[32] Temp Chip",\
     "[32] Temp Sense",\
     "CURRENT_RANGE_VALUE = INT : 0",\
     "CURRENT_RANGE = CHAR : A",\
     "MAN_VOLT_SET = FLOAT : 10",\
     "SET_VOLTAGE = BOOL : n", \
     "I_LIMIT = INT : 5", \
     "SET_ILIMIT = BOOL : n", \
     "SET_CURR_RANGE = BOOL : n", \
     "READ_CURRENT = BOOL : n", \
     "POL_FINDER = BOOL : n", \
   "IV_CURVE_VSTART = FLOAT : 10",\
   "IV_CURVE_STEP = INT : 100",\
    "TAKE_IV_CURVE = BOOL : n",  \
     			\
    "",		\
    NULL }

  typedef struct {
    struct {
      char      mscb_device[32];
      char      mscb_pwd[32];
      INT       base_address[2];
    } dd;
    WORD      control;
    char      names[32];
    float     param;
    BOOL      SET_VOLTAGE;
    BOOL      SET_ILIMIT;
    BOOL      SET_CURR_RANGE;
    BOOL      READ_CURRENT;
    BOOL      POL_FINDER;
    float     IV_CURVE_VSTART;
    INT       IV_CURVE_STEP;
    BOOL      TAKE_IV_CURVE;
  } PICO_SETTINGS;

  PICO_SETTINGS pico_settings;
 
int lmscb_fd;
HNDLE hmyFlag;  

  /*-- Function declarations -----------------------------------------*/
  INT frontend_init();
  INT frontend_exit();
  INT begin_of_run(INT run_number, char *error);
  INT end_of_run(INT run_number, char *error);
  INT pause_run(INT run_number, char *error);
  INT resume_run(INT run_number, char *error);
  INT frontend_loop();
  
  INT read_trigger_event(char *pevent, INT off);
  INT read_scaler_event(char *pevent, INT off);
  INT read_mscb_event(char *pevent, INT off);
  INT localmscb_init(char const *eqname);
  void param_callback(INT hDB, INT hKey, void *info);
  void register_cnaf_callback(int debug);
  void seq_callback(INT hDB, INT hseq, void *info);


#ifdef __cplusplus
extern "C" {
#endif
  //----------------------------------------------------------------
  //set voltage source to v volts and choose auto range. the ilim is by default 2.5e-5 A.
  int set_v(float v) {
    char mscbstr[64];
    
    // sprintf(mscbstr, "*RST;\0");
    //  mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr, strlen(mscbstr)+1); \
    //    ss_sleep(500);
    if (abs(v)<10)
      {
	sprintf(mscbstr, "!SOUR:VOLT:RANG 10\0");
	
      }
    else if (abs(v)<50)
      {
	sprintf(mscbstr, "!SOUR:VOLT:RANG 50\0");
	
      }
    
    else if (abs(v)<500)
      {
	sprintf(mscbstr, "!SOUR:VOLT:RANG 500\0");
	
      }
    else
      {
	printf("Error, invalide voltage. Must be -500< V < 500 .  \n");
	return -1;
      }
    
    printf("ready for writting:%s \n", mscbstr);
    mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr, strlen(mscbstr)+1);
    ss_sleep(500);
    
    sprintf(mscbstr, "!SOUR:VOLT %g;\0",v);
    printf("ready for writting:%s \n", mscbstr);
    mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr, strlen(mscbstr)+1);
    ss_sleep(500);
    
    sprintf(mscbstr, "!SOUR:VOLT:STAT ON;\0");
    printf("ready for writting:%s \n", mscbstr);
    mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr, strlen(mscbstr)+1);
    ss_sleep(500);

    sprintf(mscbstr, "READ?\0");
    mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr, strlen(mscbstr)+1);
    ss_sleep(500);
    
    return 0;
    
  };

  //-------------------------------------------------------
//Turn voltage source off but keep settings.
void v_off(){
char mscbstr[64];
sprintf(mscbstr, "!SOUR:VOLT:STAT OFF;\0");
printf("ready for writting:%s \n", mscbstr);
mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr, strlen(mscbstr)+1);
ss_sleep(2300);
};

  //-------------------------------------------------------
//Turn voltage source on but keep settings.
void v_on(){
char mscbstr[64];
sprintf(mscbstr, "!SOUR:VOLT:STAT ON;\0");
printf("ready for writting:%s \n", mscbstr);
mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr, strlen(mscbstr)+1);
ss_sleep(500);
};


  //-------------------------------------------------------
//set ilim at 2.5e-(value). choice for value are: 5, 4, 3 or 2. (2.5e-2 not available for v>10) If the value is not permitted, ilim=lowest.
void set_ilim(int ilim){
char mscbstr[64];

sprintf(mscbstr, "!*RST;\0");
mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr, strlen(mscbstr)+1);
ss_sleep(500);
if (ilim==4)
{
sprintf(mscbstr, "!SOUR:VOLT:ILIM 2.5E-4;\0");

}
else if (ilim == 3)
{
sprintf(mscbstr, "!SOUR:VOLT:ILIM 2.5E-3;\0");

}

else if (ilim == 2)
{
sprintf(mscbstr, "!SOUR:VOLT:RANG 10;\0");
printf("ready for writting:%s \n", mscbstr);
mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr, strlen(mscbstr)+1);
ss_sleep(2300);
sprintf(mscbstr, "!SOUR:VOLT:ILIM 2.5E-2;\0");

}
else
{
sprintf(mscbstr, "!SOUR:VOLT:ILIM 2.5E-5;\0");
}
printf("ready for writting:%s \n", mscbstr);
mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr, strlen(mscbstr)+1);
ss_sleep(2300);

};

//-------------------------------------------------------
// return 0 if ok and -1 if error. Set current range with the entry: m(mili),u(micro) or n(nano)and scale: 2,20 or 200.
  int set_curr_range(char range_param[2], int range_param_value)
{
  char mscbstr[64];

	if (range_param_value == 0)
		sprintf(mscbstr, "!SENS:CURR:RANG:AUTO ON;\0");
	else{

	  //		sprintf(mscbstr, "'SENS:CURR:RANG:AUTO Off'\0");

		switch (range_param[0])
		{
		case  'm':

			if (range_param_value == 20)
				sprintf(mscbstr, "!SENS:CURR:RANG 2E-2;\0");
			else if (range_param_value == 2)
				sprintf(mscbstr, "!SENS:CURR:RANG 2E-3;\0");
			else
			{
				printf("Error, Value must be  : 2 or 20 \n");
			sprintf(mscbstr, "!SENS:CURR:RANG:AUTO ON;\0");
			return -1;
			}

			break;

		case 'u':

			if (range_param_value == 200)
				sprintf(mscbstr, "!SENS:CURR:RANG 2E-4;\0");
			else if (range_param_value == 20)
				sprintf(mscbstr, "!SENS:CURR:RANG 2E-5;\0");
			else if (range_param_value == 2)
				sprintf(mscbstr, "!SENS:CURR:RANG 2E-6;\0");
			else
			{
				printf("Error, Value must be  : 2 , 20 or 200  \n");
			sprintf(mscbstr, "!SENS:CURR:RANG:AUTO ON;\0");
			return -1;
			}
			break;
		case 'n':
			if (range_param_value == 200)
				sprintf(mscbstr, "!SENS:CURR:RANG 2E-7;\0");
			else if (range_param_value == 20)
				sprintf(mscbstr, "!SENS:CURR:RANG 2E-8;\0");
			else if (range_param_value == 2)
				sprintf(mscbstr, "!SENS:CURR:RANG 2E-9;\0");
			else
			{
				printf("Error, Value must be  : 2 , 20 or 200  \n");
			sprintf(mscbstr, "!SENS:CURR:RANG:AUTO ON;\0");
			return -1;
			}
			break;

		default:
			printf("Error, Range must be  : m , u or n  \n");
			sprintf(mscbstr, "!SENS:CURR:RANG:AUTO ON;\0");
			return -1;
		}
	}	
	printf("ready for writting:%s lenght: %i \n",mscbstr,strlen(mscbstr) );
	mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr, strlen(mscbstr)+1);
	ss_sleep(500);
	return 0;
};

 //-------------------------------------------------------
float read_curr(){
	char mscbstr[64];
  int status, size;
  
  //	sprintf(mscbstr, "!SYST:ZCH ON;\0");
  //	status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,strlen(mscbstr)+1);
  //	ss_sleep(250);

  //	sprintf(mscbstr, "!RANG 2e-9;\0");
  //	status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,strlen(mscbstr)+1);
  //	ss_sleep(250);

  //	sprintf(mscbstr, "!INIT;\0");
  //	status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,strlen(mscbstr)+1);
  //	ss_sleep(400);

  //	sprintf(mscbstr, "!SYST:ZCOR:ACQ;\0");
  //	status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,strlen(mscbstr)+1);
  //	ss_sleep(400);

  //	sprintf(mscbstr, "!SYST:ZCOR ON;\0");
  //	status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,strlen(mscbstr)+1);
  //	ss_sleep(400);

	sprintf(mscbstr, "!RANG:AUTO ON;\0");
  	status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,strlen(mscbstr)+1);
  	ss_sleep(400);

  	sprintf(mscbstr, "!SYST:ZCH OFF;\0");
  	status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,strlen(mscbstr)+1);
  	ss_sleep(500);

	sprintf(mscbstr, "READ?;\0");
	status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,strlen(mscbstr)+1);
	ss_sleep(500);

  char reponse[64];
  size = sizeof(reponse);
  reponse[size]='\0';
  status = mscb_read(lmscb_fd, pico_settings.dd.base_address[0], 2, &reponse, &size);
  if (status != MSCB_SUCCESS) {
    cm_msg(MINFO,"feSCPico","mscb_read failed [%d] on %d-ch%d"
	   , status, pico_settings.dd.base_address[0], 2);
  }
  //char *saveptr;
//char *foo, *bar;

 char *c = strchr(reponse, 'A');
  if (c != NULL) *c = 0; else return 0;
  float value;

  //foo = strtok_r(reponse, "E", &saveptr);
  //bar = strtok_r(NULL, "E", &saveptr);
  // char *cc = strchr(reponse, '+');
  // if (c != NULL) *cc = '0'; else return 0;
  // *bar=0;
  // ++bar;
 value = (float)atof(reponse);
 // printf("reponse en float avec simple: %g",value);
 //float value2= (float)atof(bar);
 // float value3 = pow(10,-value2);
 // value=value*value3;	
	return value;
};
  //Find the polaity of jonctions and set the global variable at 1 or -1;
  int pol_finder(){
    float cpos;
    float cneg;
    //   set_ilim(2);
    set_v(2);
    // ss_sleep(1000);
    cpos=read_curr();
    //  ss_sleep(1000);
 set_v(-2);
 // ss_sleep(1000);
 cneg=read_curr();
 // ss_sleep(1000);
 printf("CURR_POS: %g\n",cpos);
 printf("CURR_NEG: %g\n",cneg);

 if(cpos > -3*cneg)
      {
	printf("positif\n");
	polarity=1;
	 v_off();
      }
         else if(-1*cneg > 3*cpos)
      {
printf("negatif\n");
	polarity=-1;
	v_off();
      } 
    else
      printf("Error, raise voltage or check devices\n");
};

  /*-- Equipment list ------------------------------------------------*/

  EQUIPMENT equipment[] = {
    {"scPico",                 /* equipment name */
     {18, 0,                   /* event ID, trigger mask */
      "",                      /* event buffer */
      EQ_PERIODIC,   /* equipment type */
      0,                      /* event source */
      "MIDAS",                /* format */
      TRUE,                   /* enabled */
      RO_ALWAYS |   /* read when running and on transitions */
      RO_ODB,                 /* and update ODB */
      500,                  /* read every 20 sec */
      0,                      /* stop run after this event limit */
      0,                      /* number of sub events */
      1,                      /* log history */
      "", "", "",},
     read_mscb_event,       /* readout routine */
    },

    {""}
  };

#ifdef __cplusplus
}
#endif

/********************************************************************\
              Callback routines for system transitions

  These routines are called whenever a system transition like start/
  stop of a run occurs. The routines are called on the following
  occations:

  frontend_init:  When the frontend program is started. This routine
                  should initialize the hardware.

  frontend_exit:  When the frontend program is shut down. Can be used
                  to releas any locked resources like memory, commu-
                  nications ports etc.

  begin_of_run:   When a new run is started. Clear scalers, open
                  rungates, etc.

  end_of_run:     Called on a request to stop a run. Can send
                  end-of-run event and close run gates.

  pause_run:      When a run is paused. Should disable trigger events.

  resume_run:     When a run is resumed. Should enable trigger events.
\********************************************************************/

/*-- Sequencer callback info  --------------------------------------*/


void seq_set_voltage(INT hDB, INT hseq, void *info)
{
float volt_set;
 int size,status;
  if(pico_settings.SET_VOLTAGE==0)
    {
    v_off();
    return;
    }
size = sizeof(volt_set); 
  status = db_get_value(hDB, 0,"/Equipment/scPico/Settings/MAN_VOLT_SET", &volt_set, &size, TID_FLOAT, FALSE);
 set_v(volt_set);
  return;
}

void seq_pol_finder(INT hDB, INT hseq, void *info)
{
 if(pico_settings.POL_FINDER==0)
    return;

 pol_finder();
  // example db_set_value
  BOOL flag=0;
   db_set_value(hDB,0,"/equipment/scpico/settings/POL_FINDER", &flag, sizeof(flag), 1, TID_BOOL);
  return;
}
void seq_set_ilim(INT hDB, INT hseq, void *info)
{
int ilim_set;
 int size,status;
  if(pico_settings.SET_ILIMIT==0)
    return;

size = sizeof(ilim_set); 
  status = db_get_value(hDB, 0,"/Equipment/scPico/Settings/I_LIMIT", &ilim_set, &size, TID_INT, FALSE);
 set_ilim(ilim_set);
  // example db_set_value
  BOOL flag=0;
   db_set_value(hDB,0,"/equipment/scpico/settings/SET_ILIMIT", &flag, sizeof(flag), 1, TID_BOOL);
  return;
}
void seq_set_current_range(INT hDB, INT hseq, void *info)
{
   int range_param_value;
  char  range_param[2];
 int size,status;
  if(pico_settings.SET_CURR_RANGE==0)
    return;

 size = sizeof(range_param_value); 
  status = db_get_value(hDB, 0,"/Equipment/scPico/Settings/CURRENT_RANGE_VALUE", &range_param_value, &size, TID_INT, FALSE);

  size = sizeof(range_param); 
  status = db_get_value(hDB, 0,"/Equipment/scPico/Settings/CURRENT_RANGE", &range_param[0], &size, TID_CHAR, FALSE);

 status=set_curr_range(range_param,range_param_value);
  // example db_set_value
  BOOL flag=0;
   db_set_value(hDB,0,"/equipment/scpico/settings/SET_CURR_RANGE", &flag, sizeof(flag), 1, TID_BOOL);
  return;
}

void seq_read_current(INT hDB, INT hseq, void *info)
{
 int size,status;
  if(pico_settings.READ_CURRENT==0)
    return;
  float valuee;
 valuee=read_curr();

  // example db_set_value
 printf("response du seq:%g \n",valuee );
  BOOL flag=0;
   db_set_value(hDB,0,"/equipment/scpico/settings/READ_CURRENT", &flag, sizeof(flag), 1, TID_BOOL);
  return;
}

/*-- Frontend Init -------------------------------------------------*/
INT frontend_init()
{
 char  mscbstr[64];
  int status, size;

sprintf(mscbstr, "0\0");

mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 3, mscbstr, strlen(mscbstr)+1);
 ss_sleep(1000);

mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 5, mscbstr, strlen(mscbstr)+1);
 ss_sleep(1000);

	sprintf(mscbstr, "!SYST:ZCH OFF;\0");
  	status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,strlen(mscbstr)+1);
  	ss_sleep(1100);


  /* hardware initialization */
  /* print message and return FE_ERR_HW if frontend should not be started */
  status = localmscb_init("SCPico");
  if (status != FE_SUCCESS) {
    cm_msg(MERROR,"feSCPico","Access to mscb failed [%d]", status);
    return status;
  }
  printf("localmscb_init status:%d\n", status);

  // Setup Open record for notification

  status = db_find_key(hDB, 0, "/equipment/scpico/settings/SET_VOLTAGE", &hmyFlag);
  if (status == DB_SUCCESS) {
    /* Enable hot-link on settings/ of the equipment */
    if (hmyFlag) {
      size = sizeof(BOOL);
      if ((status = db_open_record(hDB, hmyFlag, &(pico_settings.SET_VOLTAGE)
				   , size, MODE_READ
				   , seq_set_voltage, NULL)) != DB_SUCCESS)
	return status;
    }
  }else{
    cm_msg(MERROR, "fepico", "cannot access SET_VOLTAGE");
  }

 status = db_find_key(hDB, 0, "/equipment/scpico/settings/SET_ILIMIT", &hmyFlag);
  if (status == DB_SUCCESS) {
    /* Enable hot-link on settings/ of the equipment */
    if (hmyFlag) {
      size = sizeof(BOOL);
      if ((status = db_open_record(hDB, hmyFlag, &(pico_settings.SET_ILIMIT)
				   , size, MODE_READ
				   , seq_set_ilim, NULL)) != DB_SUCCESS)
	return status;
    }
  }else{
    cm_msg(MERROR, "fepico", "cannot access SET_ILIMT");
  }

status = db_find_key(hDB, 0, "/equipment/scpico/settings/SET_CURR_RANGE", &hmyFlag);
  if (status == DB_SUCCESS) {
    /* Enable hot-link on settings/ of the equipment */
    if (hmyFlag) {
      size = sizeof(BOOL);
      if ((status = db_open_record(hDB, hmyFlag, &(pico_settings.SET_CURR_RANGE)
				   , size, MODE_READ
				   , seq_set_current_range, NULL)) != DB_SUCCESS)
	return status;
    }
  }else{
    cm_msg(MERROR, "fepico", "cannot access SET_CURR-RANGE");
  }

status = db_find_key(hDB, 0, "/equipment/scpico/settings/READ_CURRENT", &hmyFlag);
  if (status == DB_SUCCESS) {
    /* Enable hot-link on settings/ of the equipment */
    if (hmyFlag) {
      size = sizeof(BOOL);
      if ((status = db_open_record(hDB, hmyFlag, &(pico_settings.READ_CURRENT)
				   , size, MODE_READ
				   , seq_read_current, NULL)) != DB_SUCCESS)
	return status;
    }
  }else{
    cm_msg(MERROR, "fepico", "cannot access READ_CURRENT");
  }

status = db_find_key(hDB, 0, "/equipment/scpico/settings/POL_FINDER", &hmyFlag);
  if (status == DB_SUCCESS) {
    /* Enable hot-link on settings/ of the equipment */
    if (hmyFlag) {
      size = sizeof(BOOL);
      if ((status = db_open_record(hDB, hmyFlag, &(pico_settings.POL_FINDER)
				   , size, MODE_READ
				   , seq_pol_finder, NULL)) != DB_SUCCESS)
	return status;
    }
  }else{
    cm_msg(MERROR, "fepico", "cannot access POL_FINDER");
  }


  return status;
}

/*-- Frontend Exit -------------------------------------------------*/
INT frontend_exit()
{
  return SUCCESS;
}

/*-- Begin of Run --------------------------------------------------*/
INT begin_of_run(INT run_number, char *error)
{

  
  HNDLE hDB;
  int size,status;
  int range_param_value;
  char  mscbstr[64];
  char  range_param[2];
  float volt_set;
  int i_lim;

  begin_run=1;

  // Get handle of the ODB for that experiment
  cm_get_experiment_database(&hDB, NULL);
  
  // Get value of your key (parameter)
  size = sizeof(range_param_value); 
  status = db_get_value(hDB, 0,"/Equipment/scPico/Settings/CURRENT_RANGE_VALUE", &range_param_value, &size, TID_INT, FALSE);

  size = sizeof(range_param); 
  status = db_get_value(hDB, 0,"/Equipment/scPico/Settings/CURRENT_RANGE", &range_param[0], &size, TID_CHAR, FALSE);

size = sizeof(volt_set); 
  status = db_get_value(hDB, 0,"/Equipment/scPico/Settings/MAN_VOLT_SET", &volt_set, &size, TID_FLOAT, FALSE);

size = sizeof(i_lim); 
  status = db_get_value(hDB, 0,"/Equipment/scPico/Settings/I_LIMIT", &i_lim, &size, TID_INT, FALSE);


  if(status != DB_SUCCESS)
    {
      cm_msg(MERROR,"BOR","cannot get value");
      return FE_ERR_ODB;
    }
 printf("range :%s \n",range_param );
 printf("range_value :%i \n",range_param_value );

 status=set_curr_range(range_param,range_param_value);


size = sizeof(mscbstr);
 printf("size :%i len:  %i \n",size,strlen(mscbstr) );


  return SUCCESS;
}

/*-- End of Run ----------------------------------------------------*/
INT end_of_run(INT run_number, char *error)
{
  begin_run=0;
  return SUCCESS;
}

/*-- Pause Run -----------------------------------------------------*/
INT pause_run(INT run_number, char *error)
{
  return SUCCESS;
}

/*-- Resume Run ----------------------------------------------------*/
INT resume_run(INT run_number, char *error)
{
  return SUCCESS;
}

/*-- Frontend Loop -------------------------------------------------*/
INT frontend_loop()
{
  /* if frontend_call_loop is true, this routine gets called when
     the frontend is idle or once between every event */
  ss_sleep(100);
  return SUCCESS;
}

/*------------------------------------------------------------------*/
// Readout routines for different events
/*-- Trigger event routines ----------------------------------------*/

extern "C" INT poll_event(INT source, INT count, BOOL test)
/* Polling routine for events. Returns TRUE if event
   is available. If test equals TRUE, don't return. The test
   flag is used to time the polling */
{
  int i;
  DWORD lam;
  
  for (i = 0; i < count; i++) {
    lam = 1;
    if (lam & LAM_SOURCE_STATION(source))
      if (!test)
	return lam;
  }
  return 0;
}

/*-- Interrupt configuration ---------------------------------------*/
extern "C" INT interrupt_configure(INT cmd, INT source, POINTER_T adr)
{
  switch (cmd) {
  case CMD_INTERRUPT_ENABLE:
    break;
  case CMD_INTERRUPT_DISABLE:
    break;
  case CMD_INTERRUPT_ATTACH:
    break;
  case CMD_INTERRUPT_DETACH:
    break;
  }
  return SUCCESS;
}

/*-- MSCB event --------------------------------------------------*/
INT read_mscb_event(char *pevent, INT off)
{
  int status, size;
  float *pfdata;
  char mscbstr[64];
 
  //  printf(" In read_mscb_event()\n");

  /* init bank structure */
   bk_init(pevent);

  /* create Pico bank */
   bk_create(pevent, "PICO", TID_FLOAT, (void **) &pfdata);
   if(begin_run==1)
     { float curr=read_curr();
  ss_sleep(200);
   *pfdata++ = curr;
 printf("valeur dans la banque: %g \n",curr);
     }

   bk_close(pevent, pfdata);
    
#if 0
  /* init bank structure */
  bk_init(pevent);
  /* create Pico bank */
  bk_create(pevent, "PICO", TID_FLOAT, (void **) &pfdata);
  
  size = sizeof(mscbstr);
  status = mscb_read(lmscb_fd, pico_settings.dd.base_address[0], 6, mscbstr, &size);
  if (status != MSCB_SUCCESS) {
    cm_msg(MINFO,"feSCPico","mscb_read failed [%d] on %d-ch%d"
	   , status, pico_settings.dd.base_address[0], 2);
  }
  //  printf("response:%s [%d]\n", mscbstr, status);

  char *c = strchr(mscbstr, 'A');
  if (c != NULL) *c = 0; else return 0;
  *pfdata++ = atof(mscbstr);
  bk_close(pevent, pfdata);
  sprintf(mscbstr, "READ?");
  status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 5, mscbstr, 5);
#endif

  return bk_size(pevent);
}

/*-- Local MSCB event --------------------------------------------------*/
INT localmscb_init(char const *eqname)
{
  int  status, size;
  MSCB_INFO node_info;
  char set_str[80];
  PICO_SETTINGS_STR(pico_settings_str);
  
  cm_get_experiment_database(&hDB, NULL);
  
  /* Map /equipment/Trigger/settings for the sequencer */
  sprintf(set_str, "/Equipment/%s/Settings", eqname);
  //  status = db_create_key(hDB, 0, set_str, TID_KEY);
  
  /* create PARAM settings record */
  status = db_create_record(hDB, 0, set_str, strcomb(pico_settings_str));
  if (status != DB_SUCCESS)  return FE_ERR_ODB;
  
  /* create MSCB settings record */
  //  status = db_find_key (hDB, 0, set_str, &hSet);
  //  status = db_create_record(hDB, hSet, "DD", LMSCB_SETTINGS_STR);

  status = db_find_key(hDB, 0, set_str, &hSet);
  status = db_find_key(hDB, hSet, "DD", &hDD);
  if (status == DB_SUCCESS) {
    size = sizeof(pico_settings.dd);
    db_get_record(hDB, hDD, &(pico_settings.dd), &size, 0);
    
    /* open device on MSCB */
    lmscb_fd = mscb_init(pico_settings.dd.mscb_device, NAME_LENGTH, pico_settings.dd.mscb_pwd, FALSE);
    if (lmscb_fd < 0) {
      cm_msg(MERROR, "mscb_init",
	     "Cannot access MSCB submaster at \"%s\". Check power and connection.",
	     pico_settings.dd.mscb_device);
      return FE_ERR_HW;
    }
    
#if 0
    // check if FGD devices are alive 
    status = mscb_ping(lmscb_fd, pico_settings.dd.base_address[0], 1);
    if (status != FE_SUCCESS) {
      cm_msg(MERROR, "mscb_init",
	     "Cannot ping MSCB address 0. Check power and connection.");
      return FE_ERR_HW;
    }
#endif
    
    // Check for right device
    status = mscb_info(lmscb_fd, pico_settings.dd.base_address[0], &node_info);
    if (strcmp(node_info.node_name, mydevname) != 0) {
      cm_msg(MERROR, "mscb_init",
	     "Found one expected node \"%s\" at address \"%d\"."
	     , node_info.node_name, pico_settings.dd.base_address[0]);
      return FE_ERR_HW;
    }
    printf("device %s found\n", node_info.node_name);
  }
  return FE_SUCCESS;
}}

