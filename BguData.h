#ifndef BGUDATA_H
#define BGUDATA_H

class BguData
{
	public:
		BguData();
		~BguData();
		
		int UpdateFromFile(char *name, int lenName);
			
		int count;
		int mlState;
		int tetherVoltage;
		int batteryVoltage;
		int totalCurrent;
		int powerLossCount;
		int craftRoll;
		int craftPitch;
		int craftHeading;
		int windSpeed;
		int windBearing;
		int windHeading;
		int atsHealth;
		int gps1Trust;
		int gps2Trust;

		//bool gpsOn;
		
};

#endif //BGUDATA_H
