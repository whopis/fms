#include "CraftData.h"

#include "HFLogger.h"

#include <math.h>


CraftData::CraftData()
{
    //ctor

    for (int i = 0; i < 8; i++)
    {
        servoOutputs[i] = 0;
    }

    for (int i = 0; i < 16; i++)
    {
        rcChannels[i] = 0;
    }
    latitude = 0;
    longitude = 0;
    relative_alt = 0;
    heading = 0;
    armed = false;
    gpsHdop = 0;
    gpsVdop = 0;
    gpsFixType = 0;
    gpsNumberSatellites = 0;
    batteryCurrent = 0;
    batteryVoltage = 0;
    pressAbs = 0.0f;
    temperature = 0;
    pressAbs2 = 0.0f;
    temperature2 = 0;
    pressAbs3 = 0.0f;
    temperature3 = 0;

    heartbeatCount = 0;
    gimbalHeartbeatCount = 0;

    m_minSatellites = 8;
    m_maxHdop = 250;
    m_debugSkipGps = false;

    rangefinderDistance = 0.0f;

    totalFlightTime = 0;
    totalRunTime = 0;
    totalBootCount = 0;
    timeSinceReset = 0;
    

    vibrationX = 0;
    vibrationY = 0;
    vibrationZ = 0;

    clipping0 = 0;
    clipping1 = 0;
    clipping2 = 0;


    vibrationXFilter.setLength(20);
    vibrationYFilter.setLength(20);
    vibrationZFilter.setLength(20);
    
    ek2_mag_rst_alt = 0;
    ek2_mag_rst_alt_set = false;
    ek3_mag_rst_alt = 0;
    ek3_mag_rst_alt_set = false;

    gps_auto_switch = 0;
    gps_auto_switch_set = 0;
    
    wpnav_speed_up = 0;
    wpnav_speed_up_set = false;
    
    voltage_set = false;

}

CraftData::~CraftData()
{
    //dtor
}




void CraftData::SetRequirements(int satellites, int hdop, bool debugSkipGps)
{
    m_minSatellites = satellites;
    m_maxHdop = hdop;
    m_debugSkipGps = debugSkipGps;
    HFLogger::logMessage("CraftData::SetRequirements #sat %d, maxHdop %d, skipGps %d", m_minSatellites, m_maxHdop, m_debugSkipGps);
}


int32_t CraftData::targetLatitude()
{
    int32_t tiltAngle = ((rcChannels[7] - 1000) * 90) / 1000;

    if (tiltAngle > 85)
        return 0;

    float tiltRadians = tiltAngle * M_PI / 180.0;
    float altitude = (float)relative_alt / 1000.0;
    float targetDistance = altitude * tan(tiltRadians);


    float angleRadians = (heading / 100) * M_PI / 180.0;
    float deltaY = targetDistance * cos(angleRadians);

    float metersPerDegree = 110940.6;

    return (int32_t) metersPerDegree * deltaY + latitude;

}

int32_t CraftData::targetLongitude()
{
    int32_t tiltAngle = ((rcChannels[7] - 1000) * 90) / 1000;

    if (tiltAngle > 85)
        return 0;

    float tiltRadians = tiltAngle * M_PI / 180.0;
    float altitude = (float)relative_alt / 1000.0;
    float targetDistance = altitude * tan(tiltRadians);


    float angleRadians = (heading / 100) * M_PI / 180.0;
    float deltaX = targetDistance * sin(angleRadians);

    float latRadians = ((float)latitude / 10000000.0) * M_PI / 180.0;

    float metersPerDegree = cos(latRadians) * 111319.5;

    return (int32_t) metersPerDegree * deltaX + longitude;
}



bool CraftData::IsGpsGood()
{

    if (m_debugSkipGps == true)
    {
        return true;
    }
    
    bool goodGps1 = IsGps1Good();
    bool goodGps2 = IsGps2Good();
    
    bool goodGps;
    
    switch (gps_auto_switch)
    {
        case 0:
            // Use GPS 1 only
            goodGps = goodGps1;
            break;
        case 1:
            // Use best GPS
            goodGps = goodGps1 || goodGps2;
            break;
        case 2:
            // Blend GPS
            goodGps = goodGps1 && goodGps2;
            break;
        case 3:
            // Use GPS 2 only
            goodGps = goodGps2;
            break;
    }
    

    return goodGps;
}

bool CraftData::IsGps1Good()
{
    bool goodGps = false;
    if (gpsFixType > 2)
    {
        if (gpsNumberSatellites >= m_minSatellites)
        {
            if (gpsHdop <= m_maxHdop)
            {
                goodGps = true;
            }
        }
    }
    
    return goodGps;
}


bool CraftData::IsGps2Good()
{
    bool goodGps = false;
    if (gpsFixType2 > 2)
    {
        if (gpsNumberSatellites2 >= m_minSatellites)
        {
            if (gpsHdop2 <= m_maxHdop)
            {
                goodGps = true;
            }
        }
    }
    
    return goodGps;    
}


void CraftData::setVibrationFilterLength(int length)
{
    vibrationXFilter.setLength(20);
    vibrationYFilter.setLength(20);
    vibrationZFilter.setLength(20);
}

void CraftData::setVibrationData(float vX, float vY, float vZ, uint32_t clip0, uint32_t clip1, uint32_t clip2)
{
    vibrationX = vibrationXFilter.addSample(vX);
    vibrationY = vibrationXFilter.addSample(vY);
    vibrationZ = vibrationXFilter.addSample(vZ);
    
    clipping0 = clip0;
    clipping1 = clip1;
    clipping2 = clip2;
    
    HFLogger::logMessage("Vibe-Raw: %f, %f, %f", vX, vY, vZ);
    HFLogger::logMessage("Vibe-Filtered: %f, %f, %f", vibrationX, vibrationY, vibrationZ);
    HFLogger::logMessage("Clipping: %d, %d, %d", clipping0, clipping1, clipping2);
}

