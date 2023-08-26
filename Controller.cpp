#include "Controller.h"

#include "HFLogger.h"


Controller::Controller()
{
    //ctor
    m_controllerConnection = nullptr;
}

Controller::~Controller()
{
    //dtor
}


void Controller::AttachConnection(ControllerConnection* controllerConnection)
{
    HFLogger::logMessage("Controller::AttachConnection");
    m_controllerConnection = controllerConnection;
    m_controllerConnection->AttachControllerData(&Data);
}


unsigned int Controller::GetHeartbeatCount()
{
    if (m_controllerConnection == nullptr)
    {
        return 0;
    }
    else
    {
        return m_controllerConnection->GetHeartbeatCount();
    }
}



bool Controller::DataStreamValid()
{
    if (m_controllerConnection == nullptr)
    {
        return false;
    }
    else
    {
        return m_controllerConnection->DataStreamValid();
    }
}

bool Controller::IsConnectionActive(int maxSeconds)
{
    if (m_controllerConnection == nullptr)
    {
        return false;
    }
    else
    {
        return m_controllerConnection->IsConnectionActive(maxSeconds);
    }
}


void Controller::ClearManualCommands()
{
    if (m_controllerConnection != nullptr)
    {
        m_controllerConnection->ClearManualCommands();
    }
}

void Controller::SendGpsState(bool craft, uint8_t fixType, uint8_t satellites, uint16_t hdop, uint16_t vdop)
{
    if (IsConnectionActive(5))
    {
        SourceDeviceType device;
        if (craft == true)
        {
            device = Source_Craft;
        }
        else
        {
            device = Source_GroundGps;
        }

        m_controllerConnection->SendGpsRawInt(device, (uint16_t) 0, fixType, (uint32_t) 0, (uint32_t) 0, (uint32_t) 0, hdop, vdop, (uint16_t) 0, (uint16_t) 0, satellites);

    }
}


void Controller::SendGlobalPosition(bool craft, uint32_t lat, uint32_t lon, uint32_t alt, uint16_t hdg)
{
    if (IsConnectionActive(5))
    {
        SourceDeviceType device;
        if (craft == true)
        {
            device = Source_Craft;
        }
        else
        {
            device = Source_GroundGps;
        }

        m_controllerConnection->SendGlobalPositionInt(device, (uint32_t) 0, lat, lon, alt, alt, (uint16_t)0, (uint16_t)0, (uint16_t)0, hdg);

    }
}
