#include "SkyBoxConnection.h"
#include "HFLogger.h"

SkyBoxConnection::SkyBoxConnection()
{
    //ctor

    m_heartbeatCount = 0;
    m_skyBoxData = nullptr;
    //m_reelData = nullptr;

    strncpy(m_deviceId, "SkyBoxConnection", 25);

}

SkyBoxConnection::~SkyBoxConnection()
{
    //dtor
}

bool SkyBoxConnection::DataStreamValid()
{
    bool valid = false;
    if (m_heartbeatCount > 5)
    {
        HFLogger::logMessage("SkyBoxConnection::DataStreamValid == true");
        valid = true;
    }

    return valid;
}



void SkyBoxConnection::AttachSkyBoxData(SkyBoxData* skyBoxData)
{
    HFLogger::logMessage("SkyBoxConnection::AttachSkyBoxData");
    m_skyBoxData = skyBoxData;
    
}

/*
void SkyBoxConnection::AttachReelData(ReelData* reelData)
{
    HFLogger::logMessage("SkyBoxConnection::AttachReelData");
    m_reelData = reelData;
}
*/

void SkyBoxConnection::HandleHeartbeat(mavlink_message_t* msg)
{
    HFLogger::logMessage("SKYBOX: Received heartbeat");
    mavlink_msg_heartbeat_decode(msg, &m_heartbeat);
    m_heartbeatCount++;

}




void SkyBoxConnection::HandleCommandLong(mavlink_message_t* msg)
{
    mavlink_command_long_t commandLong;
    mavlink_msg_command_long_decode(msg, &commandLong);

    switch (commandLong.command)
    {
        case MAV_CMD_USER_4:
            HandleCmdUser4(commandLong);
            break;
    }

}


void SkyBoxConnection::HandleCmdUser4(mavlink_command_long_t commandLong)
{
    HFLogger::logMessage("SkyBox: CmdUser4");
    
    //if (commandLong.param1 == 1200.0f)
    if (commandLong.param1 == CUSTOM_STATUS_SKYBOX)
    {
        if (m_skyBoxData != nullptr)
        {
            m_skyBoxData->rollTopPosition = (int) commandLong.param2;
            m_skyBoxData->scissorLiftPosition = (int) commandLong.param3;
            m_skyBoxData->moveState = (int) commandLong.param6;
            m_skyBoxData->moveTimeRemaining = (int) commandLong.param7;

            HFLogger::logMessage("SkyBox: Status - rt: %d  sl: %d  ms: %d  mt: %d", (int)commandLong.param2, (int)commandLong.param3, (int)commandLong.param6, (int)commandLong.param7);

        }
        else
        {
            HFLogger::logMessage("SkyBox: Not connected - CmdUser4:1200 (CUSTOM_STATUS_SKYBOX)");
        }
    }


    //if (commandLong.param1 == 1101)
    if (commandLong.param1 == CUSTOM_STATUS_LOCKER)
    {
        if (m_skyBoxData != nullptr)
        {
            m_skyBoxData->lockerPosition = (int) commandLong.param4;
            HFLogger::logMessage("SkyBox: Status - state: %d", m_skyBoxData->lockerPosition);
        }
        else
        {
            HFLogger::logMessage("SkyBox: Not connected - CmdUser4:1101 (CUSTOM_STATUS_LOCKER)");
        }
    }
    
    //if (commandLong.param1 == 1401)
    if (commandLong.param1 == CUSTOM_STATUS_REEL)
    {
        HFLogger::logMessage("SkyBox: Reel Data0 : tetherPos=%d, temp=%d, volt12=%d, motorOn=%d, enc0=%d, enc1=%d",
                              (int)commandLong.param2, (int)commandLong.param3, (int)commandLong.param4,
                              (int)commandLong.param5, (int)commandLong.param6, (int)commandLong.param7);
        if (m_skyBoxData != nullptr)
        {
            m_skyBoxData->m_reelData.tetherPos = (int32_t)commandLong.param2;
            m_skyBoxData->m_reelData.temp = (int32_t)commandLong.param3;
            m_skyBoxData->m_reelData.volt12 = (int32_t)commandLong.param4;
            m_skyBoxData->m_reelData.motorOn = (int32_t)commandLong.param5;
            m_skyBoxData->m_reelData.enc0 = (int32_t)commandLong.param6;
            m_skyBoxData->m_reelData.enc1 = (int32_t)commandLong.param7;
        }
    }
    
    
    //if (commandLong.param1 == 1401)
    if (commandLong.param1 == CUSTOM_STATUS_REEL_1)
    {
        HFLogger::logMessage("SkyBox: Reel Data1 : state=%d, faultMask=%d, errorMask=%d, tetherPos=%d, tetherWarning=%d, motorEnabled=%d",
                              (int)commandLong.param2, (int)commandLong.param3, (int)commandLong.param4,
                              (int)commandLong.param5, (int)commandLong.param6, (int)commandLong.param7);
        if (m_skyBoxData != nullptr)
        {
            m_skyBoxData->m_reelData.state = (int32_t)commandLong.param2;
            m_skyBoxData->m_reelData.faultMask = (int32_t)commandLong.param3;
            m_skyBoxData->m_reelData.errorMask = (int32_t)commandLong.param4;
            m_skyBoxData->m_reelData.tetherPos = (int32_t)commandLong.param5;
            m_skyBoxData->m_reelData.tetherWarning = (int32_t)commandLong.param6;
            m_skyBoxData->m_reelData.motorOn = (int32_t)commandLong.param7;
        }
    }

    
    //if (commandLong.param1 == 1402)
    if (commandLong.param1 == CUSTOM_STATUS_REEL_2)
    {
        HFLogger::logMessage("SkyBox: Reel Data2 : voltTension=%d, volt12=%d, voltMotor=%d, currMotor=%d, temp=%d, tempIR=%d",
                              (int)commandLong.param2, (int)commandLong.param3, (int)commandLong.param4,
                              (int)commandLong.param5, (int)commandLong.param6, (int)commandLong.param7);
        if (m_skyBoxData != nullptr)
        {
            m_skyBoxData->m_reelData.voltTension = (int32_t)commandLong.param2;
            m_skyBoxData->m_reelData.volt12 = (int32_t)commandLong.param3;
            m_skyBoxData->m_reelData.voltMotor = (int32_t)commandLong.param4;
            m_skyBoxData->m_reelData.currMotor = (int32_t)commandLong.param5;
            m_skyBoxData->m_reelData.temp = (int32_t)commandLong.param6;
            m_skyBoxData->m_reelData.tempIr = (int32_t)commandLong.param7;
        }
    }
    

}







