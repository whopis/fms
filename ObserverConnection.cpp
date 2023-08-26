#include "ObserverConnection.h"

#include <iostream>
#include "HFLogger.h"

using namespace std;

ObserverConnection::ObserverConnection()
{
    //ctor
    m_heartbeatCount = 0;
    m_observerData = nullptr;

 //   m_landingRequested = false;
    m_lowerAltitudeRequested = false;
    m_tetherLength = 0;
    strncpy(m_deviceId, "ObserverConnection", 25);


}


ObserverConnection::~ObserverConnection()
{
    //dtor
}

/*
bool ObserverConnection::IsLandingRequested()
{
    return m_landingRequested;
}
*/

bool ObserverConnection::IsLowerAltitudeRequested()
{
    return m_lowerAltitudeRequested;
}


int ObserverConnection::GetTetherLength()
{
    return m_tetherLength;
}



bool ObserverConnection::DataStreamValid()
{
    bool valid = false;
    if (m_heartbeatCount > 5)
    {
        valid = true;
    }

    return valid;
}


void ObserverConnection::AttachObserverData(ObserverData* observerData)
{
    m_observerData = observerData;
}

void ObserverConnection::HandleHeartbeat(mavlink_message_t* msg)
{
    m_heartbeatCount++;
    mavlink_msg_heartbeat_decode(msg, &m_heartbeat);

    m_tetherLength = m_heartbeat.custom_mode;

    switch (m_heartbeat.base_mode)
    {
        case 0:
            m_lowerAltitudeRequested = false;
            break;
        case 9:
            m_lowerAltitudeRequested = true;
            break;
    }
}
