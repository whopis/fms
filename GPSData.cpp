#include "GPSData.h"

GPSData::GPSData()
{
    //ctor

    latitude = 0;
    longitude = 0;
    relative_alt = 0;
    heading = 0;

    gpsFixType = 0;
    gpsHdop = 0;
    gpsVdop = 0;
    gpsNumberSatellites = 0;

    pressAbs = 0.0f;
    temperature = 0;

    pressAbs2 = 0.0f;
    temperature2 = 0;

    m_minSatellites = 8;
    m_maxHdop = 250;
    m_debugSkipGps = false;
    m_baroCompensate = false;
}

GPSData::~GPSData()
{
    //dtor
}


void GPSData::SetRequirements(int satellites, int hdop, bool debugSkipGps, bool baroCompensate)
{
    m_minSatellites = satellites;
    m_maxHdop = hdop;
    m_debugSkipGps = debugSkipGps;
    m_baroCompensate = baroCompensate;
}


bool GPSData::IsGpsGood()
{
    if (m_debugSkipGps == true)
    {
        return true;
    }

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
