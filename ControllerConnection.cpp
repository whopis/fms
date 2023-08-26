#include "ControllerConnection.h"

#include <iostream>
#include "HFLogger.h"

using namespace std;

ControllerConnection::ControllerConnection()
{
    //ctor
    m_heartbeatCount = 0;
    m_controllerData = nullptr;

    strncpy(m_deviceId, "ControllerConnection", 25);
}


ControllerConnection::~ControllerConnection()
{
    //dtor
}



bool ControllerConnection::DataStreamValid()
{
    bool valid = false;
    if (m_heartbeatCount > 5)
    {
        valid = true;
    }

    return valid;
}


void ControllerConnection::AttachControllerData(ControllerData* controllerData)
{
    m_controllerData = controllerData;
}

void ControllerConnection::HandleHeartbeat(mavlink_message_t* msg)
{
    m_heartbeatCount++;
    mavlink_msg_heartbeat_decode(msg, &m_heartbeat);
    if (m_heartbeat.type == MAV_TYPE_GCS)
    {
        Master = true;
        HFLogger::logMessage("MASTER HB");


    }
    else
    {
        Master = false;
        HFLogger::logMessage("SLAVE HB");
    }

    if (m_heartbeat.autopilot == 7)
    {
        NoGroundGps = true;
    }
}

void ControllerConnection::HandleManualControl(mavlink_message_t* msg)
{
    mavlink_msg_manual_control_decode(msg, &m_manualControl);

    HFLogger::logMessage("Controller - Manual Control");

    if (m_controllerData != nullptr)
    {
        if (m_manualControl.buttons >= 64)
        {
            m_controllerData->RemoteJoyX = m_manualControl.x;
            m_controllerData->RemoteJoyY = m_manualControl.y;
            m_controllerData->RemoteJoyZ = m_manualControl.z;
            if (m_manualControl.buttons == 65)
            {
                m_controllerData->RemoteJoyButton = true;
            }
            else
            {
                m_controllerData->RemoteJoyButton = false;
            }
        }
        else
        {
            m_controllerData->JoyX = m_manualControl.x;
            m_controllerData->JoyY = m_manualControl.y;
            m_controllerData->JoyZ = m_manualControl.z;
            if (m_manualControl.buttons == 1)
            {
                m_controllerData->JoyButton = true;
            }
            else
            {
                m_controllerData->JoyButton = false;
            }
        }

    }
}

void ControllerConnection::ClearManualCommands()
{
    if (m_controllerData != nullptr)
    {
        m_controllerData->JoyX = 0;
        m_controllerData->JoyY = 0;
        m_controllerData->JoyZ = 0;
    }
}


void ControllerConnection::HandleMissionItem(mavlink_message_t* msg)
{
    mavlink_msg_mission_item_decode(msg, &m_missionItem);

    HFCommand* cmd;
    cmd = new HFCommand(HFCMD_NAVIGATE_TO_POS);

    cmd->X = m_missionItem.x;
    cmd->Y = m_missionItem.y;
    cmd->Z = m_missionItem.z;

    if (m_controllerData != nullptr)
    {
        m_controllerData->QueueCommand(cmd);
        HFLogger::logMessage("Controller - NAVIGATE_TO_POS - %0.7f, %0.7f, %0.7f", m_missionItem.x, m_missionItem.y, m_missionItem.z);
    }
}


void ControllerConnection::HandleRcChannelsOverride(mavlink_message_t* msg)
{
    HFLogger::logMessage("Controller RC Override");

    mavlink_msg_rc_channels_override_decode(msg, &m_rcChannelsOverride);

    if (m_controllerData != nullptr)
    {
        m_controllerData->m_auxRcPort.channels[0] = m_rcChannelsOverride.chan1_raw;
        m_controllerData->m_auxRcPort.channels[1] = m_rcChannelsOverride.chan2_raw;
        m_controllerData->m_auxRcPort.channels[2] = m_rcChannelsOverride.chan3_raw;
        m_controllerData->m_auxRcPort.channels[3] = m_rcChannelsOverride.chan4_raw;
        m_controllerData->m_auxRcPort.channels[4] = m_rcChannelsOverride.chan5_raw;
        m_controllerData->m_auxRcPort.channels[5] = m_rcChannelsOverride.chan6_raw;
        m_controllerData->m_auxRcPort.channels[6] = m_rcChannelsOverride.chan7_raw;
        m_controllerData->m_auxRcPort.channels[7] = m_rcChannelsOverride.chan8_raw;
    }
}



void ControllerConnection::HandleCommandLong(mavlink_message_t* msg)
{
    mavlink_command_long_t commandLong;
    mavlink_msg_command_long_decode(msg, &commandLong);

    switch (commandLong.command)
    {
        case MAV_CMD_COMPONENT_ARM_DISARM:
            HandleCmdComponentArmDisarm(commandLong);
            break;
        case MAV_CMD_NAV_TAKEOFF:
            HandleCmdNavTakeoff(commandLong);
            break;
        case MAV_CMD_NAV_LAND:
            HandleCmdNavLand(commandLong);
            break;
        case MAV_CMD_DO_FOLLOW:
            HandleCmdDoFollow(commandLong);
            break;
        case MAV_CMD_CONDITION_CHANGE_ALT:
            HandleCmdConditionChangeAlt(commandLong);
            break;
        case MAV_CMD_USER_1:
            HandleCmdUser1(commandLong);
            break;
        case MAV_CMD_USER_2:
            HandleCmdUser2(commandLong);
            break;
        case MAV_CMD_USER_3:
            HandleCmdUser3(commandLong);
            break;
        case MAV_CMD_USER_4:
            HandleCmdUser4(commandLong);
            break;
        case MAV_CMD_USER_5:
            HandleCmdUser5(commandLong);
            break;
        case MAV_CMD_DO_MOUNT_CONTROL:
            HandleCmdDoMountControl(commandLong);
            break;
        case MAV_CMD_NAV_ROI:
            HandleCmdNavRoi(commandLong);
            break;
        default:
            break;
    }

}





void ControllerConnection::HandleCmdComponentArmDisarm(mavlink_command_long_t commandLong)
{
    HFCommand* cmd;
    if (commandLong.param1 == 1.0f)
    {
        cmd = new HFCommand(HFCMD_ARM);
        HFLogger::logMessage("Controller - ARM");
    }
    else
    {
        cmd = new HFCommand(HFCMD_DISARM);
        HFLogger::logMessage("Controller - DISARM");
    }

    if (m_controllerData != nullptr)
    {
        m_controllerData->QueueCommand(cmd);
    }
    else
    {
        HFLogger::logMessage("Controller - NO POINTER");
    }

}

void ControllerConnection::HandleCmdNavTakeoff(mavlink_command_long_t commandLong)
{
    HFCommand* cmd;
    cmd = new HFCommand(HFCMD_LAUNCH);

    if (m_controllerData != nullptr)
    {
        m_controllerData->QueueCommand(cmd);
        HFLogger::logMessage("Controller - LAUNCH");
    }
}

void ControllerConnection::HandleCmdNavLand(mavlink_command_long_t commandLong)
{
    HFCommand* cmd;
    cmd = new HFCommand(HFCMD_LAND);
    if (commandLong.param3 == 100)
    {
        cmd->UseLandHeading = true;
        cmd->LandHeading = (int) (commandLong.param4 * 100);
    }
    else
    {
        cmd->UseLandHeading = false;
        cmd->LandHeading = 0;
    }

    if (m_controllerData != nullptr)
    {
        m_controllerData->QueueCommand(cmd);
        HFLogger::logMessage("Controller - LAND");
    }
}

void ControllerConnection::HandleCmdDoFollow(mavlink_command_long_t commandLong)
{
    HFCommand* cmd;
    cmd = new HFCommand(HFCMD_FOLLOW);
    if (commandLong.param1 == 1.0f)
    {
        cmd->Enable = true;
        HFLogger::logMessage("Controller - FOLLOW:ENABLE");
    }
    else
    {
        cmd->Enable = false;
        HFLogger::logMessage("Controller - FOLLOW:DISABLE");
    }

    if (m_controllerData != nullptr)
    {
        m_controllerData->QueueCommand(cmd);
    }
}

void ControllerConnection::HandleCmdConditionChangeAlt(mavlink_command_long_t commandLong)
{
    HFCommand* cmd;
    cmd = new HFCommand(HFCMD_CHANGE_ALT);
    cmd->Z = commandLong.param7;
    if (commandLong.param2 == 0)
    {
        cmd->AbsoluteAltitude = false;
    }
    else
    {
        cmd->AbsoluteAltitude = true;
    }

    if (m_controllerData != nullptr)
    {
        m_controllerData->QueueCommand(cmd);
        HFLogger::logMessage("Controller - CHANGE_ALT: %0.2f", cmd->Z);
    }
}

void ControllerConnection::HandleCmdUser1(mavlink_command_long_t commandLong)
{
    HFCommand* cmd;
    cmd = new HFCommand(HFCMD_FLIGHTMODE);

    if (m_controllerData != nullptr)
    {
        m_controllerData->QueueCommand(cmd);
        HFLogger::logMessage("Controller - FLIGHTMODE_TOGGLE");
    }

}


void ControllerConnection::HandleCmdUser2(mavlink_command_long_t commandLong)
{
    HFCommand* cmd;
    cmd = new HFCommand(HFCMD_REPOSITION);

    cmd->Frame = HFFRAME_CRAFT;
    cmd->X = commandLong.param5;
    cmd->Y = commandLong.param6;
    cmd->Z = commandLong.param7;

    if (m_controllerData != nullptr)
    {
        m_controllerData->QueueCommand(cmd);
        HFLogger::logMessage("Controller - REPOSITION - %0.2f, %0.2f, %0.2f, %0.2f", commandLong.param1, commandLong.param5, commandLong.param6, commandLong.param7);
    }

}


void ControllerConnection::HandleCmdUser3(mavlink_command_long_t commandLong)
{
    HFCommand* cmd;
    cmd = new HFCommand(HFCMD_REMOTE_CTRL);

    if (commandLong.param1 == 1.0f)
    {
        cmd->Enable = true;
    }
    else
    {
        cmd->Enable = false;
    }

    if (m_controllerData != nullptr)
    {
        m_controllerData->QueueCommand(cmd);
        //HFLogger::logMessage("Controller - REMOTE CONTROL - %0.0f", commandLong.param1);
    }

}



void ControllerConnection::HandleCmdUser4(mavlink_command_long_t commandLong)
{
    HFCommand* cmd;
    
    HFLogger::logMessage("Controller - CMD: %d", (int)commandLong.param1);
    

    if (commandLong.param1 == 100.0f)
    {
        cmd = new HFCommand(HFCMD_COLOR_PALETTE);
        cmd->PaletteCommand = (int)commandLong.param2;


        if (m_controllerData != nullptr)
        {
            m_controllerData->QueueCommand(cmd);
            HFLogger::logMessage("Controller - COLOR PALETTE - %0.0f", commandLong.param2);
        }
    }


    if (commandLong.param1 == 200.0f)
    {
        cmd = new HFCommand(HFCMD_ZOOM);
        cmd->ZoomCommand = (int)commandLong.param2;


        if (m_controllerData != nullptr)
        {
            m_controllerData->QueueCommand(cmd);
            HFLogger::logMessage("Controller - ZOOM - %0.0f", commandLong.param2);
        }
    }

    if (commandLong.param1 == 250.0f)
    {
        cmd = new HFCommand(HFCMD_ZOOMSCALE);
        cmd->ZoomCommand = (int) commandLong.param2;

        if (m_controllerData != nullptr)
        {
            m_controllerData->QueueCommand(cmd);
            HFLogger::logMessage("Controller - ZOOMSCALE - %0.0f", commandLong.param2);
        }
    }

    // This is hardcoded for now
    if (commandLong.param1 == 300.0f)
    {
        HFLogger::logMessage("Controller - LED STATE");
        if (m_controllerData != nullptr)
        {
            int ledCommand = (int) commandLong.param2;
            switch (ledCommand)
            {
                case 0:
                case 1:
                case 2:
                case 3:
                    m_controllerData->LedState = (int) commandLong.param2;
                    break;

                default:
                    if (m_controllerData->LedState == 0)
                    {
                        m_controllerData->LedState = 1;
                    }
                    else
                    {
                        m_controllerData->LedState = 0;
                    }
                    break;
            }
            // now handled by BguMonitor thread
            //m_controllerData->WriteCommandFile();
        }
    }

    if (commandLong.param1 == 301.0f)
    {
        HFLogger::logMessage("Controller - LASER STATE");
        if (m_controllerData != nullptr)
        {
            m_controllerData->LaserState = (int) commandLong.param2;
            // now handled by BguMonitor thread
            //m_controllerData->WriteCommandFile();
        }
    }


    if (commandLong.param1 == 400.0f)
    {
        HFLogger::logMessage("Controller - GO TO HOME RECEIVED");
        cmd = new HFCommand(HFCMD_GO_TO_HOME);

        if (m_controllerData != nullptr)
        {
            m_controllerData->QueueCommand(cmd);
            HFLogger::logMessage("Controller - GO TO HOME ACCEPTED");
        }
    }
    
    
    ///////////////////////////////////
    //////  DJB - HACK FOR TARDEC /////


    /*
    if (commandLong.param1 == 1210)
    {
        HFLogger::logMessage("Controller - SKYBOX MOVE");

        if (m_controllerData != nullptr)
        {
            if (commandLong.param2 == 31)
            {
                HFLogger::logMessage("Controller - SKYBOX SET TO LAUNCH");
                cmd = new HFCommand(HFCMD_SKYBOX_OPEN);
                m_controllerData->QueueCommand(cmd);
            }

            if (commandLong.param2 == 32)
            {
                HFLogger::logMessage("Controller - SKYBOX SET TO STOW");
                cmd = new HFCommand(HFCMD_SKYBOX_CLOSE);
                m_controllerData->QueueCommand(cmd);
            }
        }
        else
        {
            if (commandLong.param2 == 31)
                HFLogger::logMessage("Controller - Not Connected - SKYBOX TO LAUNCH");
            if (commandLong.param2 == 32)
                HFLogger::logMessage("Controller - Not Connected - SKYBOX TO STOW");
        }
    }
    */


    if ((commandLong.param1 == 1220) || (commandLong.param1 == 1210))
    {
        HFLogger::logMessage("Controller - LOCKER MOVE");
        
        if (commandLong.param1 == 1210) 
        {
            HFLogger::logMessage("Controller - LOCKER ACCEPTING SKYBOX COMMAND");
        }

        if (m_controllerData != nullptr)
        {
            if (commandLong.param2 == 31)
            {
                HFLogger::logMessage("Controller - LOCKER OPEN");
                cmd = new HFCommand(HFCMD_LOCKER_OPEN);
                m_controllerData->QueueCommand(cmd);
            }

            if (commandLong.param2 == 32)
            {
                HFLogger::logMessage("Controller - LOCKER CLOSE");
                cmd = new HFCommand(HFCMD_LOCKER_CLOSE);
                m_controllerData->QueueCommand(cmd);
            }
        }
        else
        {
            if (commandLong.param2 == 31)
                HFLogger::logMessage("Controller - Not Connected - LOCKER OPEN");
            if (commandLong.param2 == 32)
                HFLogger::logMessage("Controller - Not Connected - LOCKER CLOSE");
        }
    }

    //////  DJB - END HACK FOR TARDEC /////
    ///////////////////////////////////////



    if (commandLong.param1 == 1410)
    {
        HFLogger::logMessage("Controller - REEL ResetTetherPos");

        if (m_controllerData != nullptr)
        {
            HFLogger::logMessage("Controller - REEL RESET TETHER POS");
            cmd = new HFCommand(HFCMD_REEL_RESETPOS);
            m_controllerData->QueueCommand(cmd);
        }
        else
        {
            HFLogger::logMessage("Controller - Not Connected - REEL RESET TETHER POS");
        }
    }
    
    
    if (commandLong.param1 == 2000)
    {
        HFLogger::logMessage("Controller - Precision Land Setting");
        if (m_controllerData != nullptr)
        {
            cmd = new HFCommand(HFCMD_PRECISION_LAND);

            if (commandLong.param2 == 1)            
            {
              HFLogger::logMessage("Controller - Precision Land Enabled");
              cmd->Enable = true;
            }
            
            if (commandLong.param2 == 0)
            {
              HFLogger::logMessage("Controller - Precision Land Disabled");
              cmd->Enable = false;
            }

            m_controllerData->QueueCommand(cmd);
            
        }
    }
    
    
    if (commandLong.param1 == 3000)
    {
        HFLogger::logMessage("Controller - GPS Enable Setting");
        if (m_controllerData != nullptr)
        {
            int gpsEnabled = (int) commandLong.param2;
            HFLogger::logMessage("Controller - GPS Enable: %d", gpsEnabled);
            if (gpsEnabled == 0)
            {
                m_controllerData->GpsOn = 0;
            }
            else
            {
                m_controllerData->GpsOn = 1;
            }
                                
        }        
    }
    
    
    if (commandLong.param1 == 3100)
    {
        HFLogger::logMessage("Controller - GPS Default Location");
        if (m_controllerData != nullptr)
        {
            int32_t lat = (int32_t) (commandLong.param2 * 10000000);
            int32_t lon = (int32_t) (commandLong.param3 * 10000000);
            
            HFLogger::logMessage("GPS-DENIED: default lat %3.3f, lon %3.3f / (%d, %d)", commandLong.param2, commandLong.param3, lat, lon);
            
            if ((lat > 1800000000) || (lon > 1800000000))
            {
                m_controllerData->gpsDeniedDefaultLatitude = 2000000000;
                m_controllerData->gpsDeniedDefaultLongitude = 2000000000;
            }
            else
            {
                m_controllerData->gpsDeniedDefaultLatitude = lat;
                m_controllerData->gpsDeniedDefaultLongitude = lon;
            }
        }
    }

}


void ControllerConnection::HandleCmdUser5(mavlink_command_long_t commandLong)
{
    HFLogger::logMessage("Controller - KILL CRAFT");
    bool validKillCommand = true;

    union {
        float floatValue;
        unsigned char byteValue[4];
    } converter;


    converter.floatValue = commandLong.param1;
    HFLogger::logMessage("P1: %2x %2x %2x %2x", (unsigned char)converter.byteValue[0], (unsigned char)converter.byteValue[1], (unsigned char)converter.byteValue[2], (unsigned char)converter.byteValue[3]);

    converter.floatValue = commandLong.param2;
    HFLogger::logMessage("P2: %2x %2x %2x %2x", (unsigned char)converter.byteValue[0], (unsigned char)converter.byteValue[1], (unsigned char)converter.byteValue[2], (unsigned char)converter.byteValue[3]);

    converter.floatValue = commandLong.param3;
    HFLogger::logMessage("P3: %2x %2x %2x %2x", (unsigned char)converter.byteValue[0], (unsigned char)converter.byteValue[1], (unsigned char)converter.byteValue[2], (unsigned char)converter.byteValue[3]);

    converter.floatValue = commandLong.param4;
    HFLogger::logMessage("P4: %2x %2x %2x %2x", (unsigned char)converter.byteValue[0], (unsigned char)converter.byteValue[1], (unsigned char)converter.byteValue[2], (unsigned char)converter.byteValue[3]);

    converter.floatValue = commandLong.param5;
    HFLogger::logMessage("P5: %2x %2x %2x %2x", (unsigned char)converter.byteValue[0], (unsigned char)converter.byteValue[1], (unsigned char)converter.byteValue[2], (unsigned char)converter.byteValue[3]);

    converter.floatValue = commandLong.param6;
    HFLogger::logMessage("P6: %2x %2x %2x %2x", (unsigned char)converter.byteValue[0], (unsigned char)converter.byteValue[1], (unsigned char)converter.byteValue[2], (unsigned char)converter.byteValue[3]);

    converter.floatValue = commandLong.param7;
    HFLogger::logMessage("P7: %2x %2x %2x %2x", (unsigned char)converter.byteValue[0], (unsigned char)converter.byteValue[1], (unsigned char)converter.byteValue[2], (unsigned char)converter.byteValue[3]);


    converter.floatValue = commandLong.param1;
    if ((unsigned char)converter.byteValue[1] != (unsigned char)0x11)
    {
      validKillCommand = false;
      HFLogger::logMessage("Expected %2x, got %2x", 0x11, converter.byteValue[1]);
    }
    else
        HFLogger::logMessage("check 1");

    if (converter.byteValue[0] != 0x1A)
    {
      validKillCommand = false;
      HFLogger::logMessage("Expected %2x, got %2x", 0x1A, converter.byteValue[0]);
    }
    else
        HFLogger::logMessage("check 2");


    converter.floatValue = commandLong.param2;
    if (converter.byteValue[2] != 0x21)
    {
      validKillCommand = false;
      HFLogger::logMessage("Expected %2x, got %2x", 0x21, converter.byteValue[2]);
    }
    else
        HFLogger::logMessage("check 3");

    if (converter.byteValue[1] != 0x1A)
    {
      validKillCommand = false;
      HFLogger::logMessage("Expected %2x, got %2x", 0x1A, converter.byteValue[1]);
    }
    else
        HFLogger::logMessage("check 4");

    if (converter.byteValue[0] != 0x2B)
    {
      validKillCommand = false;
      HFLogger::logMessage("Expected %2x, got %2x", 0x2b, converter.byteValue[0]);
    }
    else
        HFLogger::logMessage("check 5");

    converter.floatValue = commandLong.param3;
    if (converter.byteValue[2] != 0x31)
    {
      validKillCommand = false;
      HFLogger::logMessage("Expected %2x, got %2x", 0x31, converter.byteValue[2]);
    }
    else
        HFLogger::logMessage("check 6");

    if (converter.byteValue[1] != 0xB2)
    {
      validKillCommand = false;
      HFLogger::logMessage("Expected %2x, got %2x", 0xb2, converter.byteValue[1]);
    }
    else
        HFLogger::logMessage("check 7");

    if (converter.byteValue[0] != 0xB3)
    {
      validKillCommand = false;
      HFLogger::logMessage("Expected %2x, got %2x", 0xB3, converter.byteValue[0]);
    }
    else
        HFLogger::logMessage("check 8");

    converter.floatValue = commandLong.param4;
    if (converter.byteValue[0] != 0x00)
    {
      validKillCommand = false;
      HFLogger::logMessage("Expected %2x, got %2x", 0x00, converter.byteValue[0]);
    }
    else
        HFLogger::logMessage("check 9");

    converter.floatValue = commandLong.param5;
    if (converter.byteValue[0] != 0x00)
    {
      validKillCommand = false;
      HFLogger::logMessage("Expected %2x, got %2x", 0x00, converter.byteValue[0]);
    }
    else
        HFLogger::logMessage("check 10");

    converter.floatValue = commandLong.param6;
    if (converter.byteValue[0] != 0x00)
    {
      validKillCommand = false;
      HFLogger::logMessage("Expected %2x, got %2x", 0x00, converter.byteValue[0]);
    }
    else
        HFLogger::logMessage("check 11");

    converter.floatValue = commandLong.param7;
    if (converter.byteValue[0] != 0x01)
    {
      validKillCommand = false;
      HFLogger::logMessage("Expected %2x, got %2x", 0x01, converter.byteValue[0]);
    }
    else
        HFLogger::logMessage("check 12");


    if (validKillCommand == true)
    {
        HFCommand* cmd;
        cmd = new HFCommand(HFCMD_KILL_CRAFT);
        cmd->Enable = true;
        m_controllerData->QueueCommand(cmd);
        HFLogger::logMessage("Controller - KILL CRAFT");
    }


}


void ControllerConnection::HandleCmdDoMountControl(mavlink_command_long_t commandLong)
{
    HFCommand* cmd;
    cmd = new HFCommand(HFCMD_MOUNT_CTRL);

    cmd->DeltaTilt = (int) commandLong.param1;
    if (m_controllerData != nullptr)
    {
        m_controllerData->QueueCommand(cmd);
        HFLogger::logMessage("Controller - MOUNT_CONTROL - %0.2f", commandLong.param1);
    }
}


void ControllerConnection::HandleCmdNavRoi(mavlink_command_long_t commandLong)
{
    if (m_controllerData != nullptr)
    {
        m_controllerData->RoiLatitude = commandLong.param5;
        m_controllerData->RoiLongitude = commandLong.param6;
        m_controllerData->RoiAltitude = commandLong.param7;
        m_controllerData->NewRoi = true;
    }
}


