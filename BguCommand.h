#ifndef BGUCOMMAND_H
#define BGUCOMMAND_H

#include <stdint.h>

class BguCommand
{
	public:
		BguCommand();
		~BguCommand();
		
		int32_t UpdateFile(char *name, int lenName);
		bool DataHasChanged();
		int32_t UpdateBguCommandFileOptions();
			
		int32_t count;
		int32_t ledState;
		int32_t laserState;
		int32_t tetherPos;
		int32_t gpsOn;
		int32_t corrOn;
		int32_t swingOn;
		int32_t gpsGuiLat;
		int32_t gpsGuiLon;
		
		int32_t countFile;
		int32_t ledStateFile;
		int32_t laserStateFile;
		int32_t tetherPosFile;
		int32_t gpsOnFile;
		int32_t corrOnFile;
		int32_t swingOnFile;
		int32_t gpsGuiLatFile;
		int32_t gpsGuiLonFile;
};

#endif //BGUCOMMAND_H
