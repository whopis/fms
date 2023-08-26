#include "Craft.h"

#include "SourceData.h"

#include "HFLogger.h"

Craft::Craft()
{
    //ctor
    m_craftConnection = nullptr;

    m_gimbalTiltAngle = 0;
}

Craft::~Craft()
{
    //dtor
}


void Craft::AttachConnection(CraftConnection* craftConnection)
{
    m_craftConnection = craftConnection;
    m_craftConnection->AttachCraftData(&Data);

}


void Craft::SetGimbalTiltAngle(int angle)
{
    if (angle < 0)
        angle = 0;
    if (angle > 90000)
        angle = 90000;

    m_gimbalTiltAngle = angle;
}

void Craft::AdjustGimbalTiltAngle(int deltaAngle)
{
    int tempAngle = m_gimbalTiltAngle + deltaAngle;
    if (tempAngle < 0)
        tempAngle = 0;
    if (tempAngle > 90000)
        tempAngle = 90000;
    m_gimbalTiltAngle = tempAngle;
}

int  Craft::GetGimbalTiltAngle()
{
    return m_gimbalTiltAngle;
}

int  Craft::GetGimbalTiltOutput()
{
    int value;
    value = 1000 + (1000 * m_gimbalTiltAngle) / 90000;
    return value;
}


unsigned int Craft::GetHeartbeatCount()
{
    if (m_craftConnection == nullptr)
    {
        return 0;
    }
    else
    {
        return m_craftConnection->GetHeartbeatCount();
    }
}

bool Craft::DataStreamValid()
{
    if (m_craftConnection == nullptr)
    {
        return false;
    }
    else
    {
        return m_craftConnection->DataStreamValid();
    }
}


void Craft::RequestDataStream()
{
    if (m_craftConnection !=  nullptr)
    {
        m_craftConnection->RequestDataStream(Source_FMS, 0);
    }
}

void Craft::SendRcOverridesLow(uint16_t rc1, uint16_t rc2, uint16_t rc3, uint16_t rc4, uint16_t rc5, uint16_t rc6, uint16_t rc7, uint16_t rc8)
{
    if (m_craftConnection != nullptr)
    {
        m_craftConnection->OverrideRcChannelsLow(Source_FMS, rc1, rc2, rc3, rc4, rc5, rc6, rc7, rc8);
    }
}

void Craft::SendRcOverridesLow(uint16_t rc[])
{
    SendRcOverridesLow(rc[0], rc[1], rc[2], rc[3], rc[4], rc[5], rc[6], rc[7]);
}

bool Craft::VerifyRcOverridesLow(uint16_t rc[])
{
    if (m_craftConnection == nullptr)
    {
        return false;
    }
    else
    {
        bool success = true;
        for (int i = 0; i < 8; i++)
        {
            if ((rc[i] != 0) && (rc[i] != 65535))
            {
                if (rc[i] != Data.rcChannels[i])
                  success = false;
            }
            
        }
        return success;
    }
}


bool Craft::SendAndVerifyRcOverridesLow(uint16_t rc[], uint16_t timeout_ms)
{
    if (m_craftConnection == nullptr)
    {
        return false;
    }
    else
    {
        bool success = true;
        SendRcOverridesLow(rc[0], rc[1], rc[2], rc[3], rc[4], rc[5], rc[6], rc[7]);
        std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));

        success = VerifyRcOverridesLow(rc);

        return success;
    }
}




void Craft::SendRcOverridesHigh(uint16_t rc1, uint16_t rc2, uint16_t rc3, uint16_t rc4, uint16_t rc5, uint16_t rc6, uint16_t rc7, uint16_t rc8)
{
    if (m_craftConnection != nullptr)
    {
        m_craftConnection->OverrideRcChannelsHigh(Source_FMS, rc1, rc2, rc3, rc4, rc5, rc6, rc7, rc8);
    }
}

void Craft::SendRcOverridesHigh(uint16_t rc[])
{
    SendRcOverridesHigh(rc[0], rc[1], rc[2], rc[3], rc[4], rc[5], rc[6], rc[7]);
}

bool Craft::VerifyRcOverridesHigh(uint16_t rc[])
{
    if (m_craftConnection == nullptr)
    {
        return false;
    }
    else
    {
        bool success = true;
        for (int i = 0; i < 8; i++)
        {
            if (rc[i] != 0)
            {
                if (rc[i] != Data.rcChannels[i+8])
                  success = false;
            }
        }
        return success;
    }
}


bool Craft::SendAndVerifyRcOverridesHigh(uint16_t rc[], uint16_t timeout_ms)
{
    if (m_craftConnection == nullptr)
    {
        return false;
    }
    else
    {
        bool success = true;
        SendRcOverridesHigh(rc[0], rc[1], rc[2], rc[3], rc[4], rc[5], rc[6], rc[7]);
        std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));

        success = VerifyRcOverridesHigh(rc);

        return success;
    }
}




void Craft::SendArm()
{
    HFLogger::logMessage("ARMING");
    if (m_craftConnection != nullptr)
    {
        m_craftConnection->SendCommandLong(Source_FMS, 1, 250, 400, 0, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
}



void Craft::SendDisarm()
{
    HFLogger::logMessage("DISARMING");
    if (m_craftConnection != nullptr)
    {
        m_craftConnection->SendCommandLong(Source_FMS, 1, 250, 400, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
}



void Craft::SendTerminateFlight()
{
    HFLogger::logMessage("TERMINATING");
    if (m_craftConnection != nullptr)
    {
        m_craftConnection->SendCommandLong(Source_FMS, 1, 1, 185, 0, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
}



void Craft::SendSetPositionTargetGlobalInt(long latInt, long lonInt, float altitude)
{
    if (m_craftConnection != nullptr)
    {
        int coordinateFrame = 6;
        int typeMask = 0b0000110111111000;
        m_craftConnection->SendSetPositionTargetGlobalInt(Source_FMS, 0, 1, 1, coordinateFrame, typeMask, latInt, lonInt, altitude, 0, 0, 0, 0, 0, 0, 0, 0);
    }
}



void Craft::SendMissionItem(float latitude, float longitude, float altitude)
{
    if (m_craftConnection != nullptr)
    {
        m_craftConnection->ClearMissionAck();
        m_craftConnection->SendMissionItem(Source_FMS, 1, 1, 0, 3, 16, 2, 1, 0.0f, 0.0f, 0.0f, 0.0f, latitude, longitude, altitude);
    }
}

bool Craft::MissionItemAcknowledged()
{
    if (m_craftConnection == nullptr)
    {
        return false;
    }
    else
    {
        if ((m_craftConnection->MissionAckReceived() == true) && (m_craftConnection->MissionAckType() == 0))
        {
            m_craftConnection->ClearMissionAck();
            return true;
        }
        else
        {
            return false;
        }
    }
}

void Craft::SendTakeoff(float altitude)
{
    HFLogger::logMessage("TAKEOFF to altitude: %f", altitude);
    if (m_craftConnection != nullptr)
    {
        m_craftConnection->ClearMissionAck();
        m_craftConnection->SendCommandLong(Source_FMS, 1, 1, 22, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, altitude);
    }
}



bool Craft::TakeoffAcknowledged()
{
    if (m_craftConnection == nullptr)
    {
        return false;
    }
    else
    {
        if ((m_craftConnection->CommandAckRecevied() == true) && (m_craftConnection->CommandAckCommand() == 22))
        {
            m_craftConnection->ClearMissionAck();
            return true;
        }
        else
        {
            return false;
        }
    }
}


void Craft::SendLand()
{
    if (m_craftConnection != nullptr)
    {
        m_craftConnection->ClearCommandAck();
        m_craftConnection->SendCommandLong(Source_FMS, 1, 250, 21, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
}


bool Craft::LandAcknowledged()
{
    if (m_craftConnection == nullptr)
    {
        return false;
    }
    else
    {
        if ((m_craftConnection->CommandAckRecevied() == true) && (m_craftConnection->CommandAckCommand() == 21))
        {
            m_craftConnection->ClearMissionAck();
            return true;
        }
        else
        {
            return false;
        }
    }
}


void Craft::SendRegionOfInterest(float latitude, float longitude, float altitude)
{
    if (m_craftConnection != nullptr)
    {
        m_craftConnection->SendCommandLong(Source_FMS, 1, 1, 201, 0, 4.0f, 0, 0, 0, latitude, longitude, altitude);
    }
}


void Craft::SendAutopilotVersionRequest()
{
    if (m_craftConnection != nullptr)
    {
        m_craftConnection->SendAutopilotVersionRequest(Source_FMS, 1, 1);
    }
}


void Craft::SendVersionBannerRequest()
{
    if (m_craftConnection != nullptr)
    {
        m_craftConnection->SendCommandLong(Source_FMS, 1, 1, 42428, 0, 0, 0, 0, 0, 0, 0, 0);
    }
}

bool Craft::IsConnectionActive(int maxSeconds)
{
    if (m_craftConnection == nullptr)
    {
        return false;
    }
    else
    {
        return m_craftConnection->IsConnectionActive(maxSeconds);
    }
}


void Craft::UpdateBarometerBase(float pressAbs, int temperature)
{
    char paramId[16];
    if (m_craftConnection != nullptr)
    {
        float pressPascals = pressAbs * 100.0f;
        float tempDegrees = (float) temperature / 100.0f;

        memset(paramId, 0, 16);
        strncpy(paramId, "GND_ABS_PRESS", 16);
        m_craftConnection->SendParamSet(Source_FMS, 1, 1, paramId, pressPascals, MAV_PARAM_TYPE_REAL32);

        memset(paramId, 0, 16);
        strncpy(paramId, "GND_TEMP", 16);
        m_craftConnection->SendParamSet(Source_FMS, 1, 1, paramId, tempDegrees, MAV_PARAM_TYPE_REAL32);
    }
}


void Craft::UpdateBarometerBase2(float pressAbs, float pressAbs2, int temperature)
{
    char paramId[16];
    if (m_craftConnection != nullptr)
    {
        float pressPascals = pressAbs * 100.0f;
        float pressPascals2 = pressAbs2 * 100.0f;
        float tempDegrees = (float) temperature / 100.0f;

        memset(paramId, 0, 16);
        strncpy(paramId, "GND_ABS_PRESS", 16);
        m_craftConnection->SendParamSet(Source_FMS, 1, 1, paramId, pressPascals, MAV_PARAM_TYPE_REAL32);

        memset(paramId, 0, 16);
        strncpy(paramId, "GND_ABS_PRESS2", 16);
        m_craftConnection->SendParamSet(Source_FMS, 1, 1, paramId, pressPascals2, MAV_PARAM_TYPE_REAL32);


        memset(paramId, 0, 16);
        strncpy(paramId, "GND_TEMP", 16);
        m_craftConnection->SendParamSet(Source_FMS, 1, 1, paramId, tempDegrees, MAV_PARAM_TYPE_REAL32);
    }
}


void Craft::UpdateBarometerBase3(float pressAbs, float pressAbs2, float pressAbs3, int temperature)
{
    char paramId[16];
    if (m_craftConnection != nullptr)
    {
        float pressPascals = pressAbs * 100.0f;
        float pressPascals2 = pressAbs2 * 100.0f;
        float pressPascals3 = pressAbs3 * 100.0f;
        float tempDegrees = (float) temperature / 100.0f;

        memset(paramId, 0, 16);
        strncpy(paramId, "GND_ABS_PRESS", 16);
        m_craftConnection->SendParamSet(Source_FMS, 1, 1, paramId, pressPascals, MAV_PARAM_TYPE_REAL32);

        memset(paramId, 0, 16);
        strncpy(paramId, "GND_ABS_PRESS2", 16);
        m_craftConnection->SendParamSet(Source_FMS, 1, 1, paramId, pressPascals2, MAV_PARAM_TYPE_REAL32);

        memset(paramId, 0, 16);
        strncpy(paramId, "GND_ABS_PRESS3", 16);
        m_craftConnection->SendParamSet(Source_FMS, 1, 1, paramId, pressPascals3, MAV_PARAM_TYPE_REAL32);

        /*
        memset(paramId, 0, 16);
        strncpy(paramId, "GND_TEMP", 16);
        m_craftConnection->SendParamSet(Source_FMS, 1, 1, paramId, tempDegrees, MAV_PARAM_TYPE_REAL32);
        */
    }
}


void Craft::UpdateYawResetAltitude(float alt)
{
    char paramId[16];
    if (m_craftConnection != nullptr)
    {
        memset(paramId, 0, 16);
        strncpy(paramId, "EK2_MAG_RST_ALT", 16);
        m_craftConnection->SendParamSet(Source_FMS, 1, 1, paramId, alt, MAV_PARAM_TYPE_REAL32);

        memset(paramId, 0, 16);
        strncpy(paramId, "EK3_MAG_RST_ALT", 16);
        m_craftConnection->SendParamSet(Source_FMS, 1, 1, paramId, alt, MAV_PARAM_TYPE_REAL32);

    }
}


void Craft::UpdateWpNavSpeedUp(float wpNavSpeedUp)
{
    char paramId[16];
    if (m_craftConnection != nullptr)
    {
        Data.wpnav_speed_up_set = false;
        memset(paramId, 0, 16);
        strncpy(paramId, "WPNAV_SPEED_UP", 16);
        m_craftConnection->SendParamSet(Source_FMS, 1, 1, paramId, wpNavSpeedUp, MAV_PARAM_TYPE_REAL32);
    }
}




void Craft::SendParamRequestRead(char* param_id)
{
    char paramId[16];
    if (m_craftConnection != nullptr)
    {
        memset(paramId, 0, 16);
        strncpy(paramId, param_id, 16);
        m_craftConnection->SendParamRequestRead(Source_FMS, 1, 1, paramId);
    }
}



void Craft::SendResetFlightStats()
{
    char paramId[16];
    if (m_craftConnection != nullptr)
    {


        memset(paramId, 0, 16);
        strncpy(paramId, "STAT_RESET", 16);
        m_craftConnection->SendParamSet(Source_FMS, 1, 1, paramId, 0, MAV_PARAM_TYPE_REAL32);
    }
}


bool Craft::IsInLandMode()
{
    bool inLand = false;
    
    if (m_craftConnection != nullptr)
    {
        if (Data.custom_mode == 9)
        {
            inLand = true;
        }
    }

    return inLand;
}
