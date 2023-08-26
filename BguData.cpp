#include "BguData.h"
#include <string.h>
#include <stdio.h>

#include "HFLogger.h"
  
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
//#include "rapidjson/writer.h"
//#include "rapidjson/prettywriter.h"
//#include "rapidjson/stringbuffer.h"
#include "rapidjson/error/en.h"

#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>

using namespace rapidjson;

#include <errno.h>
#include <thread>
#include <chrono>
 
BguData::BguData() 
{
	 count = 0;
	 mlState = 0;
	 tetherVoltage = 0;
	 batteryVoltage = 0;
	 totalCurrent = 0;
	 powerLossCount = 0;
	 craftRoll = 0;
	 craftPitch = 0;
	 craftHeading = 0;
	 windSpeed = 0;
	 windBearing = 0;
	 windHeading = 0;
	 atsHealth = 1;
	 gps1Trust = 0;
	 gps2Trust = 0;

	 //gpsOn = false;
	 
	 
	 
}



BguData::~BguData() 
{

}

int BguData::UpdateFromFile(char *name, int lenName)
{
	int retVal = 0;
	char filename[256];
	memset(filename, 0, 256);

	// local copies of file data - members only updated if entire read is successful
	int lcount = 0;
	int lmlState = 0;
	int ltetherVoltage = 0;
	int lbatteryVoltage = 0;
	int ltotalCurrent = 0;
	int lpowerLossCount = 0;
	int lcraftRoll = 0;
	int lcraftPitch = 0;
	int lcraftHeading = 0;
	int lwindSpeed = 0;
	int lwindBearing = 0;
	int lwindHeading = 0;
	int latsHealth = 1;
	int lgps1Trust = 0;
	int lgps2Trust = 0;

    try
    {
		if ((name != NULL) && (lenName > 0))
		{
			memcpy(filename, name, lenName);

			FILE *fp = fopen(filename, "r");
			char readBuffer[65535];
			Document document;
			
			if (fp != NULL)
			{
				FileReadStream is(fp, readBuffer, sizeof(readBuffer));
				document.ParseStream(is);
				if (document.IsObject())
				{
					if (document.HasMember("Count"))
					{
						lcount = document["Count"].GetInt();
					}
					if (document.HasMember("MavLinkState"))
					{
						lmlState = document["MavLinkState"].GetInt();
					}
					if (document.HasMember("TetherVoltage"))
					{
						ltetherVoltage = document["TetherVoltage"].GetInt();
					}
					if (document.HasMember("BatteryVoltage"))
					{
						lbatteryVoltage = document["BatteryVoltage"].GetInt();
					}
					if (document.HasMember("TotalCurrent"))
					{
						ltotalCurrent = document["TotalCurrent"].GetInt();
					}
					if (document.HasMember("CountNoTetherPower"))
					{
						lpowerLossCount = document["CountNoTetherPower"].GetInt();
					}
					if (document.HasMember("CraftRoll"))
					{
						lcraftRoll = document["CraftRoll"].GetInt();
					}
					if (document.HasMember("CraftPitch"))
					{
						lcraftPitch = document["CraftPitch"].GetInt();
					}
					if (document.HasMember("CraftHeading"))
					{
						lcraftHeading = document["CraftHeading"].GetInt();
					}
					if (document.HasMember("WindSpeed"))
					{
						lwindSpeed = document["WindSpeed"].GetInt();
					}
					if (document.HasMember("WindBearing"))
					{
						lwindBearing = document["WindBearing"].GetInt();
					}
					if (document.HasMember("WindHeading"))
					{
						lwindHeading = document["WindHeading"].GetInt();
					}
					if (document.HasMember("ATSHealth"))
					{
						latsHealth = document["ATSHealth"].GetInt();
					}
					else
					{
						latsHealth = 1;
					}
					if (document.HasMember("GPS1Trust"))
					{
						lgps1Trust = document["GPS1Trust"].GetInt();
					}
					else
					{
						lgps1Trust = 100;
					}
					if (document.HasMember("GPS2Trust"))
					{
						lgps2Trust = document["GPS2Trust"].GetInt();
					}
					else
					{
						lgps2Trust = 100;
					}
				}
				else
				{
					HFLogger::logMessage("BGU Monitor :: UpdateError : document is not an object");
					retVal = -3;
				}
				fclose(fp);
			}// end file opened
			else
			{
				// Could not open file
				int errnum = errno;
				char errStr[512];
				sprintf(errStr, "FILE OPEN ERROR: %s", strerror(errnum));
				
				HFLogger::logMessage("BGU Monitor :: UpdateError : %s", errStr);
				
				// file did not open
				retVal = -2;
			}// file didn't open
					
		}
		else
		{
			HFLogger::logMessage("BGU Monitor :: UpdateError : missing file name");
			// bad/missing file name
			retVal = -1;
		}
	}
	catch (const std::exception& e)
	{
		HFLogger::logMessage("BGU Monitor :: UpdateError : Exception opening/reading file = %s", e.what() );
		retVal = -10;
	}

	// if there were no errors, update the member
	if (retVal == 0)
	{
			count = lcount;
			mlState = lmlState;
			tetherVoltage = ltetherVoltage;
			batteryVoltage = lbatteryVoltage;
			totalCurrent = ltotalCurrent;
			powerLossCount = lpowerLossCount;
			craftRoll = lcraftRoll;
			craftPitch = lcraftPitch;
			craftHeading = lcraftHeading;
			windSpeed = lwindSpeed;
			windBearing = lwindBearing;
			windHeading = lwindHeading;
			atsHealth = latsHealth;
			gps1Trust = lgps1Trust;
			gps2Trust = lgps2Trust;
	}
	
    return retVal;
}// end UpdateFromFile




