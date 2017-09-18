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


 //-------------------------------------------------------
double read_curr(){
	char mscbstr[64];
  int status, size;

  sprintf(mscbstr, "READ?;");
  printf("READ?;");
  status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,strlen(mscbstr)+1);
  printf("Status write : %i\n",status);
  ss_sleep(3000); // wait for measurement to finish

  char reponse[64];
  size = sizeof(reponse);
  status = mscb_read(lmscb_fd, pico_settings.dd.base_address[0], 2, &reponse, &size);
  printf("response:%s| \n",reponse);
  if (status != MSCB_SUCCESS) {
    cm_msg(MINFO,"feSCPico","mscb_read failed [%d] on %d-ch%d"
	   , status, pico_settings.dd.base_address[0], 2);
  }

 char *c = strchr(reponse, 'A');
 if (c != NULL){ 
   *c = 0;
 }
 else return 0;
 double value;
 value = atof(reponse);
 return value;
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
      3000,                  /* read every 3 sec */
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

/*-- Frontend Init -------------------------------------------------*/
INT frontend_init()
{
  char  mscbstr[64];
  int ival;
  int status, size;


  /* hardware initialization */
  /* print message and return FE_ERR_HW if frontend should not be started */
  status = localmscb_init("SCPico");
  if (status != FE_SUCCESS) {
    cm_msg(MERROR,"feSCPico","Access to mscb failed [%d]", status);
    return status;
  }
  printf("localmscb_init status:%d\n", status);
  ss_sleep(100);


  printf("Resetting the picoammeter\n");
  
  sprintf(mscbstr, "*RST                    ");
  status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,24);
  if(status != MSCB_SUCCESS)
    printf("reset return status = %i\n",status);
  ss_sleep(5000);


  sprintf(mscbstr, "0");

  
  ival = 0;
  status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 3, &ival, sizeof(ival));
  if(status != MSCB_SUCCESS)
    printf("Reset address 3; status = %i.\n",status);
  ss_sleep(1000);
  

  status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 5, &ival, sizeof(ival));
  if(status != MSCB_SUCCESS)
    printf("Reset address 5; status = %i.\n",status);
  ss_sleep(1000);

  printf("Setting range %s \n", mscbstr);

  sprintf(mscbstr, "SYST:ZCH ON         ");
  printf(" %s \n", mscbstr);
  status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,20);
  if(status != MSCB_SUCCESS)
    printf("zch on return status = %i\n",status);
  ss_sleep(10000);

  sprintf(mscbstr, "RANG 0.0002 ");
  status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,12);
  if(status != MSCB_SUCCESS)  printf("rang return status = %i\n",status);
  ss_sleep(1000);
  sprintf(mscbstr, "INIT          ");
  status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,14);
  if(status != MSCB_SUCCESS) printf("init return status = %i\n",status);
  ss_sleep(1000);

  sprintf(mscbstr, "SYST:ZCOR:ACQ ");
  status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,14);
  if(status != MSCB_SUCCESS)  printf("zcor:acq return status = %i\n",status);
  ss_sleep(1000);
  sprintf(mscbstr, "SYST:ZCH OFF  ");
  status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,14);
  if(status != MSCB_SUCCESS)  printf("zch off return status = %i\n",status);
  ss_sleep(1000);
  sprintf(mscbstr, "SYST:ZCOR ON  ");
  status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,14);
  if(status != MSCB_SUCCESS) printf("zcor on return status = %i\n",status);
  ss_sleep(1000);
  sprintf(mscbstr, "MED 0         ");
  status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,14);
  if(status != MSCB_SUCCESS) printf("med return status = %i\n",status);
  ss_sleep(1000);
  sprintf(mscbstr, "AVER ON       ");
  status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,14);
  if(status != MSCB_SUCCESS)  printf("aver return status = %i\n",status);
  ss_sleep(1000);
  sprintf(mscbstr, "AVER:COUNt 100 ");
  status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,15);
  if(status != MSCB_SUCCESS) printf("avg:count return status = %i\n",status);
  ss_sleep(1000);


  
  printf("Checking parameters\n");
  sprintf(mscbstr, "AVER?          ");
  status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,15);
  if(status != MSCB_SUCCESS)
    printf("aver return status = %i\n",status);
  ss_sleep(3000);



  sprintf(mscbstr, "ZCH?");
  status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,5);
  if(status != MSCB_SUCCESS)
    printf("aver return status = %i\n",status);
  ss_sleep(3000);

  sprintf(mscbstr, "ZCOR?");
  status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr,5);
  if(status != MSCB_SUCCESS)
    printf("aver return status = %i\n",status);
  ss_sleep(3000);







   printf("Finished reset\n");

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


  return SUCCESS;
}

/*-- End of Run ----------------------------------------------------*/
INT end_of_run(INT run_number, char *error)
{
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
  double *pfdata;
  char mscbstr[64];
 
  printf(" In read_mscb_event()\n");
  double curr=read_curr();

  /* init bank structure */
  bk_init(pevent);
  
#if 1
  /* create Pico bank */
  bk_create(pevent, "PICO", TID_DOUBLE, (void **) &pfdata);

  *pfdata++ = curr;
  printf("valeur dans la banque: %e \n",curr);
  
  
  bk_close(pevent, pfdata);
  printf("size %i\n", bk_size(pevent));
#endif

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
  printf("response:%s [%d]\n", mscbstr, status);

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

