#include "SkyBox.h"

#include "HFLogger.h"


SkyBox::SkyBox()
{
    //ctor
    m_skyBoxConnection = nullptr;
    m_lockerMode = false;
}

SkyBox::~SkyBox()
{
    //dtor
}

void SkyBox::SetLockerMode(bool lockerMode)
{
    m_lockerMode = lockerMode;
}


void SkyBox::AttachConnection(SkyBoxConnection* skyBoxConnection)
{
    m_skyBoxConnection = skyBoxConnection;
    m_skyBoxConnection->AttachSkyBoxData(&Data);
//    m_skyBoxConnection->AttachReelData(&reelData);
}


unsigned int SkyBox::GetHeartbeatCount()
{
    if (m_skyBoxConnection == nullptr)
    {
        return 0;
    }
    else
    {
        return m_skyBoxConnection->GetHeartbeatCount();
    }
}



bool SkyBox::DataStreamValid()
{
    if (m_skyBoxConnection == nullptr)
    {
        return false;
    }
    else
    {
        return m_skyBoxConnection->DataStreamValid();
    }
}




bool SkyBox::IsConnectionActive(int maxSeconds)
{
    if (m_skyBoxConnection == nullptr)
    {
        return false;
    }
    else
    {
        return m_skyBoxConnection->IsConnectionActive(maxSeconds);
    }
}

bool SkyBox::IsInFlightPosition()
{

    // Return true if there is an active connection and it is in flight position
    if (m_lockerMode == false) {
        if (Data.IsSkyBoxInFlightPosition() == true) {
            if (IsConnectionActive(5) == true) {
                return true;
            }
        }
    }
    else
    {
        if (Data.IsLockerOpen() == true) {
            if (IsConnectionActive(5) == true) {
                return true;
            }
        }
    }

    return false;
}


bool SkyBox::IsInStoragePosition()
{
    // Return true if there is an active connection and it is in flight position
    if (m_lockerMode == false)
    {
        if (Data.IsSkyBoxInStoragePosition() == true) {
            if (IsConnectionActive(5) == true) {
                return true;
            }
        }
    }
    else
    {
        if (Data.IsLockerClosed() == true) {
            if (IsConnectionActive(5) == true) {
                return true;
            }
        }
    }
    return false;
}



void SkyBox::SendSetToFlightPosition()
{
    HFLogger::logMessage("SkyBox: Set To Flight Position");
    if (m_skyBoxConnection != nullptr)
    {
        m_skyBoxConnection->SendCommandLong(Source_FMS, 1, 250, 31013, 0, 1210, 31, 0, 0, 0, 0, 0);
    }
    else
    {
        HFLogger::logMessage("SkyBox: Not attached");
    }
}


void SkyBox::SendSetToStoragePosition()
{
    HFLogger::logMessage("SkyBox: Set To Storage Position");
    if (m_skyBoxConnection != nullptr)
    {
        m_skyBoxConnection->SendCommandLong(Source_FMS, 1, 250, 31013, 0, 1210, 32, 0, 0, 0, 0, 0);
    }
    else
    {
        HFLogger::logMessage("SkyBox: Not attached");
    }
}



void SkyBox::LockerOpen()
{
    HFLogger::logMessage("SkyBox: Open Locker");
    if (m_skyBoxConnection != nullptr)
    {
        m_skyBoxConnection->SendCommandLong(Source_FMS, 1, 250, 31013, 0, 1100, 10, 0, 0, 0, 0, 0);
    }
    else
    {
        HFLogger::logMessage("SkyBox: Not attached");
    }
}


void SkyBox::LockerClose()
{
    HFLogger::logMessage("SkyBox: Close Locker");
    if (m_skyBoxConnection != nullptr)
    {
        m_skyBoxConnection->SendCommandLong(Source_FMS, 1, 250, 31013, 0, 1100, 11, 0, 0, 0, 0, 0);
    }
    else
    {
        HFLogger::logMessage("SkyBox: Not attached");
    }
}

void SkyBox::SendEnableIPS(bool enable)
{
}


void SkyBox::SendEnableReel(bool enable)
{
}

void SkyBox::SendReelResetTetherPos()
{
        SendReelResetTetherPos(Data.m_reelData.resetPosition);
}

void SkyBox::SendReelResetTetherPos(int pos)
{
    HFLogger::logMessage("SkyBox: SendReelResetTetherPos");
    if (m_skyBoxConnection != nullptr)
    {
        m_skyBoxConnection->SendCommandLong(Source_FMS, 1, 250, 31013, 0, 1410, pos, 0, 0, 0, 0, 0);
    }
    else
    {
        HFLogger::logMessage("Skybox: Not attached");
    }
}

void SkyBox::SetReelResetPosition(int pos)
{
    Data.m_reelData.resetPosition = pos;
}

int SkyBox::GetReelResetPosition()
{
    return Data.m_reelData.resetPosition;
}



















