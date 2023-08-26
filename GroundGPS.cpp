#include "GroundGPS.h"

GroundGPS::GroundGPS()
{
    //ctor
    m_gpsConnection = nullptr;
}

GroundGPS::~GroundGPS()
{
    //dtor
}


void GroundGPS::AttachConnection(GPSConnection* gpsConnection)
{
    m_gpsConnection = gpsConnection;
    m_gpsConnection->AttachGpsData(&Data);
}


unsigned int GroundGPS::GetHeartbeatCount()
{
    if (m_gpsConnection == nullptr)
    {
        return 0;
    }
    else
    {
        return m_gpsConnection->GetHeartbeatCount();
    }
}



bool GroundGPS::DataStreamValid()
{
    if (m_gpsConnection == nullptr)
    {
        return false;
    }
    else
    {
        return m_gpsConnection->DataStreamValid();
    }
}

bool GroundGPS::PressureReceived()
{
    if (m_gpsConnection == nullptr)
    {
        return false;
    }
    else
    {
        return m_gpsConnection->PressureReceived();
    }
}

void GroundGPS::RequestDataStream()
{
    if (m_gpsConnection !=  nullptr)
    {
        m_gpsConnection->RequestDataStream(Source_FMS, 0);
    }
}


bool GroundGPS::IsConnectionActive(int maxSeconds)
{
    if (m_gpsConnection == nullptr)
    {
        return false;
    }
    else
    {
        return m_gpsConnection->IsConnectionActive(maxSeconds);
    }
}
