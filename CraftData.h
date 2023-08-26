#ifndef CRAFTDATA_H
#define CRAFTDATA_H

#include <stdint.h>


#include "BoxcarFilter.h"



enum RcChannelsType {
    RC_PITCH = 0,
    RC_ROLL = 1,
    RC_THROTTLE = 2,
    RC_YAW = 3,
    RC_MODE = 4,
    RC_PAN = 5,
    RC_ZOOM = 6,
    RC_TILT = 7
};

class CraftData
{
    public:
        CraftData();
        virtual ~CraftData();

        uint16_t servoOutputs[8];
        uint16_t rcChannels[16];

        int32_t latitude;
        int32_t longitude;
        int32_t relative_alt;
        uint16_t heading;
        float pressAbs;
        int temperature;

        float pressAbs2;
        int temperature2;

        float pressAbs3;
        int temperature3;

        bool armed;

        uint32_t custom_mode;

        uint16_t gpsHdop;
        uint16_t gpsVdop;
        uint8_t gpsFixType;
        uint8_t gpsNumberSatellites;

        uint16_t gpsHdop2;
        uint16_t gpsVdop2;
        uint8_t gpsFixType2;
        uint8_t gpsNumberSatellites2;



        uint16_t batteryCurrent;
        uint16_t batteryVoltage;

        void SetRequirements(int satellites, int hdop, bool debugSkipGps);

        bool IsGpsGood();
        bool IsGps1Good();
        bool IsGps2Good();

        int heartbeatCount;
        int gimbalHeartbeatCount;
        

        int32_t targetLatitude();
        int32_t targetLongitude();

        float rangefinderDistance;

        int totalFlightTime;
        int totalRunTime;
        int totalBootCount;
        int timeSinceReset;


        float vibrationX;
        float vibrationY;
        float vibrationZ;
        
        uint32_t clipping0;
        uint32_t clipping1;
        uint32_t clipping2;

        BoxcarFilter vibrationXFilter;
        BoxcarFilter vibrationYFilter;
        BoxcarFilter vibrationZFilter;
        
        void setVibrationFilterLength(int length);
        void setVibrationData(float vX, float vY, float vZ, uint32_t clip0, uint32_t clip1, uint32_t clip2);
        
        
        float ek2_mag_rst_alt;
        bool ek2_mag_rst_alt_set;
        float ek3_mag_rst_alt;
        bool ek3_mag_rst_alt_set;
        
        int gps_auto_switch;
        bool gps_auto_switch_set;
        
        int wpnav_speed_up;
        bool wpnav_speed_up_set;
        
        bool voltage_set;
               
    protected:
        int m_minSatellites;
        int m_maxHdop;
        bool m_debugSkipGps;


    private:
};

#endif // CRAFTDATA_H
