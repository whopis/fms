#include "BguCommand.h"
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

#include "HFLogger.h"

#include <thread>
#include <chrono>

#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>

using namespace std;
 
BguCommand::BguCommand() 
{
	count = -1;
	ledState = -1;
	laserState = -1;
	tetherPos = -1;
	gpsOn = -1;
	//corrOn = -1;
	corrOn = 1;
	swingOn = -1;
	gpsGuiLat = 2000000000;
	gpsGuiLon = 2000000000;
	
	countFile = -2;
	ledStateFile = -2;
	laserStateFile = -2;
	tetherPosFile = -2;
	gpsOnFile = -1;
	corrOnFile = -1;	
	swingOnFile = -1;
	gpsGuiLatFile = 2000000000;
	gpsGuiLonFile = 2000000000;
	
}



BguCommand::~BguCommand() 
{

}

int32_t BguCommand::UpdateFile(char *name, int lenName)
{
	int retVal = 0;
	char filename[256];
	char filenameTemp[256];
	char catTemp[256];
	
	memset(filename, 0, 256);
	strncpy(catTemp, ".temp", 6);
	
	if (DataHasChanged() != true)
	{
		HFLogger::logMessage("BGU Command :: UpdateError : No changes needed.");
		// No need to update file
		retVal = -1;
	}
	else
	{
		try
		{
			// Data has changed, so update the file
			count++;
			if ((name != NULL) && (lenName > 0))
			{
				UpdateBguCommandFileOptions();
				memcpy(filename, name, lenName);
				memcpy(filenameTemp, name, lenName);
				strncat(filenameTemp, catTemp, strlen(catTemp) ); 
				//FILE *fp = fopen(filename, "w");
				FILE *fp = fopen(filenameTemp, "w");
				if (fp != NULL)
				{
					fprintf(fp, "{\n");
					fprintf(fp, "\t\"count\" : %d,\n", count);
					fprintf(fp, "\t\"ledState\" : %d,\n", ledState);
					fprintf(fp, "\t\"laserState\" : %d,\n", laserState);
					fprintf(fp, "\t\"tetherPos\" : %d,\n", tetherPos);
					fprintf(fp, "\t\"gpsState\" : %d,\n", gpsOn);
					fprintf(fp, "\t\"corrState\" : %d,\n", corrOn);
					fprintf(fp, "\t\"swingState\" : %d,\n", swingOn);
					fprintf(fp, "\t\"gpsGuiLat\" : %d,\n", gpsGuiLat);
					fprintf(fp, "\t\"gpsGuiLon\" : %d\n", gpsGuiLon);
					fprintf(fp, "}\n");
					fclose(fp);
					
					countFile = count;
					ledStateFile = ledState;
					laserStateFile = laserState;
					tetherPosFile = tetherPos;
					gpsOnFile = gpsOn;
					corrOnFile = corrOn;
					swingOnFile = swingOn;
					gpsGuiLatFile = gpsGuiLat;
					gpsGuiLonFile = gpsGuiLon;
					
					if (rename(filenameTemp, filename) == 0)
					{
						//HFLogger::logMessage("BGU Command :: UpdateFile written : count=%d, ledState=%d, laserState=%d, tetherPos=%d", count, ledState, laserState, tetherPos);
						//HFLogger::logMessage("BGU Command :: UpdateFile written : count=%d, ledState=%d, laserState=%d, tetherPos=%d, gpsOn=%d, corrOn=%d, swingOn=%d", count, ledState, laserState, tetherPos, gpsOn, corrOn, swingOn);
						HFLogger::logMessage("BGU Command :: UpdateFile written : count=%d, ledState=%d, laserState=%d, tetherPos=%d, gpsOn=%d, corrOn=%d, swingOn=%d, gpsGuiLat=%d, gpsGuiLon=%d",
												 count, ledState, laserState, tetherPos, gpsOn, corrOn, swingOn, gpsGuiLat, gpsGuiLon);
					}
					else
					{
						HFLogger::logMessage("BGU Command :: UpdateError : Failed to rename file.");
						retVal = -4;
					}
				}
				else
				{
					HFLogger::logMessage("BGU Command :: UpdateError : Could not access file =%s", filenameTemp);
					retVal = -3;
				}
			}
			else
			{
				HFLogger::logMessage("BGU Command :: UpdateError : missing file name");
				// bad/missing file name
				retVal = -2;
			}
		}
		catch (const std::exception& e)
		{
			HFLogger::logMessage("BGU Command :: UpdateError : Exception opening/writing file = %s", e.what() );
			retVal = -10;
		}
		
	}// end DataHasChanged
	
    return retVal;
}// end UpdateFile

bool BguCommand::DataHasChanged()
{
	//TESTING
	return true;
	
	if (ledState != ledStateFile)
		return true;
	if (laserState != laserStateFile)
		return true;
	if (tetherPos != tetherPosFile)
		return true;

	return false;
}// end DataHasChanged

int32_t BguCommand::UpdateBguCommandFileOptions()
{
	int32_t retVal = 0;
	
	char filename[256];
	strcpy(filename, "/home/pi/hti/apps/bguLogServer/bguCommandOptions.txt");
	try
	{
		ifstream bguCmdOpt(filename);
		if (bguCmdOpt.is_open())
		{
			string dataLine = "";
			while (getline(bguCmdOpt, dataLine))
			{
				/*
				 * This is now handled from the GUI
				if (dataLine.compare("[gpsState]") == 0)
				{
					getline(bguCmdOpt, dataLine);
					gpsOn = stoi(dataLine);
				}
				
				else 
				*/
				/*
				if (dataLine.compare("[corrState]") == 0)
				{
					getline(bguCmdOpt, dataLine);
					corrOn = stoi(dataLine);
				}
				*/
				
				//else 
				
				if (dataLine.compare("[swingState]") == 0)
				{
					getline(bguCmdOpt, dataLine);
					swingOn = stoi(dataLine);
				}
				else
				{
					// just advance to next line
				}
				
			} // while reading file line by line
			bguCmdOpt.close();
		}
	}
	catch (const std::exception& e)
	{
		HFLogger::logMessage("BGU Command :: UpdateError : Exception opening/reading the bguCommandOptions.txt file: %s", e.what() );
		retVal = -1;
	}
	return retVal;
}// end UpdateBguCOmmandFileOptions





