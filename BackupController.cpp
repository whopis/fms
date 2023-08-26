#include "BackupController.h"

#include "HFLogger.h"

BackupController::BackupController()
{
    //ctor
    m_backupControllerConnection = nullptr;
}

BackupController::~BackupController()
{
    //dtor
}


void BackupController::AttachControllerData(ControllerData* controllerData)
{
    Data = controllerData;
}

void BackupController::AttachConnection(BackupControllerConnection* backupControllerConnection)
{
    HFLogger::logMessage("BackupController::AttachConnection");
    m_backupControllerConnection = backupControllerConnection;
    m_backupControllerConnection->AttachControllerData(Data);
}


unsigned int BackupController::GetHeartbeatCount()
{
    if (m_backupControllerConnection == nullptr)
    {
        return 0;
    }
    else
    {
        return m_backupControllerConnection->GetHeartbeatCount();
    }
}



bool BackupController::DataStreamValid()
{
    if (m_backupControllerConnection == nullptr)
    {
        return false;
    }
    else
    {
        return m_backupControllerConnection->DataStreamValid();
    }
}

bool BackupController::IsConnectionActive(int maxSeconds)
{
    if (m_backupControllerConnection == nullptr)
    {
        return false;
    }
    else
    {
        return m_backupControllerConnection->IsConnectionActive(maxSeconds);
    }
}



void BackupController::SendGpsState(bool craft, uint8_t fixType, uint8_t satellites, uint16_t hdop, uint16_t vdop)
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

        m_backupControllerConnection->SendGpsRawInt(device, (uint16_t) 0, fixType, (uint32_t) 0, (uint32_t) 0, (uint32_t) 0, hdop, vdop, (uint16_t) 0, (uint16_t) 0, satellites);

    }
}


void BackupController::SendGlobalPosition(bool craft, uint32_t lat, uint32_t lon, uint32_t alt, uint16_t hdg)
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

        m_backupControllerConnection->SendGlobalPositionInt(device, (uint32_t) 0, lat, lon, alt, alt, (uint16_t)0, (uint16_t)0, (uint16_t)0, hdg);

    }
}
