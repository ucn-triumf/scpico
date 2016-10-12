using namespace std;
#include <iostream>
#include <ostream>


// return 0 if ok and -1 if error. 
int set_curr_range(char range_param[2], int range_param_value)

{
	char mscbstr[32];


	if (range_param_value == 0)
		sprintf(mscbstr, "'SENS:CURR:RANG:AUTO ON'");

	else{

		sprintf(mscbstr, "'SENS:CURR:RANG:AUTO Off'");


		switch (range_param[0])
		{
		case  'm':

			if (range_param_value == 20)
				sprintf(mscbstr, "SENS:CURR:RANG 2E-2;");
			else if (range_param_value == 2)
				sprintf(mscbstr, "SENS:CURR:RANG 2E-3;");
			else
			{
				printf("Error, Value must be  : 2 or 20 \n");
			sprintf(mscbstr, "SENS:CURR:RANG:AUTO ON;");
			return -1;
			}

			break;

		case 'u':

			if (range_param_value == 200)
				sprintf(mscbstr, "SENS:CURR:RANG 2E-4;");
			else if (range_param_value == 20)
				sprintf(mscbstr, "SENS:CURR:RANG 2E-5;");
			else if (range_param_value == 2)
				sprintf(mscbstr, "SENS:CURR:RANG 2E-6;");
			else
			{
				printf("Error, Value must be  : 2 , 20 or 200  \n");
			sprintf(mscbstr, "SENS:CURR:RANG:AUTO ON;");
			return -1;
			}
			break;

		case 'n':

			if (range_param_value == 200)
				sprintf(mscbstr, "SENS:CURR:RANG 2E-7;");
			else if (range_param_value == 20)
				sprintf(mscbstr, "SENS:CURR:RANG 2E-8;");
			else if (range_param_value == 2)
				sprintf(mscbstr, "SENS:CURR:RANG 2E-9;");
			else
			{
				printf("Error, Value must be  : 2 , 20 or 200  \n");
			sprintf(mscbstr, "SENS:CURR:RANG:AUTO ON;");
			return -1;
			}
			break;

		default:
			printf("Error, Range must be  : m , u or n  \n");
			sprintf(mscbstr, "SENS:CURR:RANG:AUTO ON;");
			return -1;
		}
	}

	printf("ready for writting:%s \n", mscbstr);
	status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr, strlen(mscbstr));

	return 0;
};

int read(char c){
	char mscbstr[32];

	if (c=='c'||'C')
	{
		sprintf(mscbstr, "FUNC ‘CURR’;");
		//status = mscb_write(lmscb_fd, pico_settings.dd.base_address[0], 1, mscbstr, len);
	}
	return 1;

};
