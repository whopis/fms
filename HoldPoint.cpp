#include "HoldPoint.h"

#include <cmath>

#include "HFLogger.h"


HoldPoint::HoldPoint()
{
    //ctor

    m_flightFloor = 5.0f;
    m_flightCeiling = 60.96f;
    m_powerLimitCeiling = m_flightCeiling;
    m_powerLimitCeilingMin = 10.0f;
    m_flightCeilingFollow = 45.72f;

    m_translateDeltaLatMeters = 0;
    m_translateDeltaLonMeters = 0;

    m_noLocationLogging = false;

    m_inFollowMode = false;
    
    m_inGpsDeniedMode = false;
    m_gpsDeniedCeiling = 40.00f;
}

HoldPoint::~HoldPoint()
{
    //dtor
}


void HoldPoint::SetToHomePosition()
{
    HFLogger::logMessage("HoldPoint: SetToHomePosition");
    if (m_noLocationLogging == true)
    {
        HFLogger::logMessage("HoldPoint: Current %0.7f, %0.7f", m_craftCurrentHoldLatitude, m_craftCurrentHoldLongitude);
        HFLogger::logMessage("HoldPoint: New     %0.7f, %0.7f", m_craftStartLatitude, m_craftStartLongitude);
    }
    m_craftCurrentHoldLatitude = m_craftStartLatitude;
    m_craftCurrentHoldLongitude = m_craftStartLongitude;

    m_translateDeltaLatMeters = 0;
    m_translateDeltaLonMeters = 0;
}



void HoldPoint::UpdateCraftPosition(float fwd, float right, int heading)
{

    float hdg = heading / 100.0f;

    double PI = 3.141592653589793;

    double hdgRadians = (hdg * PI / 180.0);
    double cosHeading = cos(hdgRadians);
    double sinHeading = sin(hdgRadians);

    double north = fwd * cosHeading - right * sinHeading;
    double east = right * cosHeading + fwd * sinHeading;


    // Calculate the total delta
    double deltaNorth = m_translateDeltaLatMeters + north;
    double deltaEast = m_translateDeltaLonMeters + east;

    if (deltaNorth < 0)
    {
      deltaNorth = deltaNorth * -1;
    }

    if (deltaEast < 0)
    {
        deltaEast = deltaEast * -1;
    }


    // Verify the total delta is within bounds
    if ((deltaNorth > 10.0) || (deltaEast > 10.0))
    {
        // Do not allow movement past +/-10 meters;
        HFLogger::logMessage("HOLD POINT: Bounding should happen here");
		HFLogger::logMessage("HOLD POINT: tranDeltaLat:%f  tranDeltaLon:%f, fwd:%f, right:%f, hdg:%d", m_translateDeltaLatMeters, m_translateDeltaLonMeters, fwd, right, heading);
        

        // Do it anyway for now
        m_translateDeltaLatMeters = m_translateDeltaLatMeters + north;
        m_translateDeltaLonMeters = m_translateDeltaLonMeters + east;


        double deltaLatPerMeter = 0.000009f;
        double currentLatitudeRadians = (m_craftCurrentHoldLatitude / 180.0) * PI;
        double deltaLonPerMeter = deltaLatPerMeter * cos(currentLatitudeRadians);


        double deltaLat = north * deltaLatPerMeter;
        double deltaLon = east * deltaLonPerMeter;

        m_craftCurrentHoldLatitude += deltaLat;
        m_craftCurrentHoldLongitude += deltaLon;

    }
    else
    {

        m_translateDeltaLatMeters = m_translateDeltaLatMeters + north;
        m_translateDeltaLonMeters = m_translateDeltaLonMeters + east;


        double deltaLatPerMeter = 0.000009f;
        double currentLatitudeRadians = (m_craftCurrentHoldLatitude / 180.0) * PI;
        double deltaLonPerMeter = deltaLatPerMeter * cos(currentLatitudeRadians);


        double deltaLat = north * deltaLatPerMeter;
        double deltaLon = east * deltaLonPerMeter;

        m_craftCurrentHoldLatitude += deltaLat;
        m_craftCurrentHoldLongitude += deltaLon;

        if (m_noLocationLogging == false)
        {
            HFLogger::logMessage("UpdateCraftPosition: %0.7f, %0.7f", m_craftCurrentHoldLatitude, m_craftCurrentHoldLongitude);
        }
    }

}

void HoldPoint::SetCraftPosition(float lat, float lon)
{
    m_craftCurrentHoldLatitude = lat;
    m_craftCurrentHoldLongitude = lon;

    if (m_noLocationLogging == false)
    {
        HFLogger::logMessage("SetCraftPosition: %0.7f, %0.7f", m_craftCurrentHoldLatitude, m_craftCurrentHoldLongitude);
    }

}

void HoldPoint::SetFloorCeiling(float floor, float ceiling)
{
    m_flightFloor = floor;
    m_flightCeiling = ceiling;
    m_powerLimitCeiling = m_flightCeiling;
    
    if (m_flightCeiling < 45.72f)
    {
        m_flightCeilingFollow = m_flightCeiling;
    }
    else
    {
        m_flightCeilingFollow = 45.72f;
    }
}



void HoldPoint::SetPowerLimitCeiling(float ceiling)
{
    // Ensure we don't drop below min ceiling
    if (ceiling < m_powerLimitCeilingMin) 
    {
        m_powerLimitCeiling = m_powerLimitCeilingMin;
    }
    else
    {
        m_powerLimitCeiling = ceiling;
    }
}


void HoldPoint::SetPowerLimitCeilingMin(float min)
{
    if (min >= 10.0f)
    {
        m_powerLimitCeilingMin = min;
    }
    else
    {
        m_powerLimitCeilingMin = 10.0f;
    }
    
    HFLogger::logMessage("PowerLimitCeilingMin: %f", m_powerLimitCeilingMin);
}



void HoldPoint::SetGpsDeniedCeiling(float ceiling)
{
    m_gpsDeniedCeiling = ceiling;
}

void HoldPoint::SetGpsDeniedMode(bool enabled)
{
    m_inGpsDeniedMode = enabled;
}

bool HoldPoint::GetGpsDeniedMode()
{
    return m_inGpsDeniedMode;
}


void HoldPoint::ResetHoldPoint(float craftLat, float craftLon, float groundLat, float groundLon)
{
    HFLogger::logMessage("HoldPoint: ResetHoldPoint");
    if (m_noLocationLogging == false)
    {
        HFLogger::logMessage("HoldPoint: Pos %0.7f, %0.7f", craftLat, craftLon);
    }

    m_craftStartLatitude = craftLat;
    m_craftStartLongitude = craftLon;

    m_groundStartLatitude = groundLat;
    m_groundStartLongitude = groundLon;

    m_craftCurrentHoldLatitude = craftLat;
    m_craftCurrentHoldLongitude = craftLon;

    m_translateDeltaLatMeters = 0;
    m_translateDeltaLonMeters = 0;



}


void HoldPoint::ResetHoldPoint(int32_t craftLat, int32_t craftLon, int32_t groundLat, int32_t groundLon)
{
    ResetHoldPoint((float)craftLat/10000000.0f, (float)craftLon/10000000.0f, (float)groundLat/10000000.0f, (float)groundLon/10000000.0f);
}






void HoldPoint::UpdateGroundPoint(float groundLat, float groundLon)
{
    m_deltaLatitude = groundLat - m_groundStartLatitude;
    m_deltaLongitude = groundLon - m_groundStartLongitude;

    m_craftCurrentHoldLatitude = m_craftStartLatitude + m_deltaLatitude;
    m_craftCurrentHoldLongitude = m_craftStartLongitude + m_deltaLongitude;

    if (m_noLocationLogging == false)
    {
        HFLogger::logMessage("UpdateGroundPoint: Ground %0.7f, %0.7f", groundLat, groundLon);
        HFLogger::logMessage("UpdateGroundPoint: Craft  %0.7f, %0.7f", m_craftCurrentHoldLatitude, m_craftCurrentHoldLongitude);
    }
}


void HoldPoint::UpdateGroundPoint(int32_t groundLat, int32_t groundLon)
{
    UpdateGroundPoint((float)groundLat/10000000.0f, (float)groundLon/10000000.0f);
}




void HoldPoint::AdjustAltitude(float deltaAltitude)
{
    float maxAltitude;
    if (m_inFollowMode == false)
        maxAltitude = m_flightCeiling;
    else
        maxAltitude = m_flightCeilingFollow;
        
    if (m_powerLimitCeiling < maxAltitude)
        maxAltitude = m_powerLimitCeiling;
        
    if (m_inGpsDeniedMode == true)
    {
        if (maxAltitude > m_gpsDeniedCeiling)
        {
            maxAltitude = m_gpsDeniedCeiling;
        }
    }

    float tempAltitude = m_craftTargetAltitude + deltaAltitude;
    if (tempAltitude > maxAltitude)
        tempAltitude = maxAltitude;
    if (tempAltitude < m_flightFloor)
        tempAltitude = m_flightFloor;

    m_craftTargetAltitude = tempAltitude;
}


void HoldPoint::SetAltitude(float altitude)
{
    float maxAltitude;
    if (m_inFollowMode == false)
        maxAltitude = m_flightCeiling;
    else
        maxAltitude = m_flightCeilingFollow;


    if (m_powerLimitCeiling < maxAltitude)
        maxAltitude = m_powerLimitCeiling;


    if (m_inGpsDeniedMode == true)
    {
        if (maxAltitude > m_gpsDeniedCeiling)
        {
            maxAltitude = m_gpsDeniedCeiling;
        }
    }
    

    float tempAltitude = altitude;
    if (tempAltitude > maxAltitude)
        tempAltitude = maxAltitude;
    if (tempAltitude < m_flightFloor)
        tempAltitude = m_flightFloor;

    m_craftTargetAltitude = tempAltitude;
}


void HoldPoint::SetFollowMode(bool inFollow)
{
    m_inFollowMode = inFollow;

    if (m_inFollowMode)
    {
        if (m_craftTargetAltitude > m_flightCeilingFollow)
        {
            m_craftTargetAltitude = m_flightCeilingFollow;
        }
    }
}
