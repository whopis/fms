#include "BackupControllerConnection.h"

#include <iostream>
#include "HFLogger.h"

using namespace std;

BackupControllerConnection::BackupControllerConnection()
{
    //ctor
    m_heartbeatCount = 0;
    m_controllerData = nullptr;

    strncpy(m_deviceId, "BackupControllerConn", 25);
}


BackupControllerConnection::~BackupControllerConnection()
{
    //dtor
}



bool BackupControllerConnection::DataStreamValid()
{
    bool valid = false;
    if (m_heartbeatCount > 5)
    {
        valid = true;
    }

    return valid;
}


void BackupControllerConnection::AttachControllerData(ControllerData* controllerData)
{
    m_controllerData = controllerData;
}

void BackupControllerConnection::HandleHeartbeat(mavlink_message_t* msg)
{
    m_heartbeatCount++;
    mavlink_msg_heartbeat_decode(msg, &m_heartbeat);
    if (m_heartbeat.type == MAV_TYPE_GCS)
    {
        Master = true;
        HFLogger::logMessage("BACKUP MASTER HB");


    }
    else
    {
        Master = false;
        HFLogger::logMessage("BACKUP SLAVE HB");
    }

    if (m_heartbeat.autopilot == 7)
    {
        NoGroundGps = true;
    }
}


void BackupControllerConnection::HandleMissionItem(mavlink_message_t* msg)
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


void BackupControllerConnection::HandleRcChannelsOverride(mavlink_message_t* msg)
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



void BackupControllerConnection::HandleCommandLong(mavlink_message_t* msg)
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





void BackupControllerConnection::HandleCmdComponentArmDisarm(mavlink_command_long_t commandLong)
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

void BackupControllerConnection::HandleCmdNavTakeoff(mavlink_command_long_t commandLong)
{
    HFCommand* cmd;
    cmd = new HFCommand(HFCMD_LAUNCH);

    if (m_controllerData != nullptr)
    {
        m_controllerData->QueueCommand(cmd);
        HFLogger::logMessage("Controller - LAUNCH");
    }
}

void BackupControllerConnection::HandleCmdNavLand(mavlink_command_long_t commandLong)
{
    HFCommand* cmd;
    cmd = new HFCommand(HFCMD_LAND);

    if (m_controllerData != nullptr)
    {
        m_controllerData->QueueCommand(cmd);
        HFLogger::logMessage("Controller - LAND");
    }
}

void BackupControllerConnection::HandleCmdDoFollow(mavlink_command_long_t commandLong)
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

void BackupControllerConnection::HandleCmdConditionChangeAlt(mavlink_command_long_t commandLong)
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

void BackupControllerConnection::HandleCmdUser1(mavlink_command_long_t commandLong)
{
    HFCommand* cmd;
    cmd = new HFCommand(HFCMD_FLIGHTMODE);

    if (m_controllerData != nullptr)
    {
        m_controllerData->QueueCommand(cmd);
        HFLogger::logMessage("Controller - FLIGHTMODE_TOGGLE");
    }

}


void BackupControllerConnection::HandleCmdUser2(mavlink_command_long_t commandLong)
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


void BackupControllerConnection::HandleCmdUser3(mavlink_command_long_t commandLong)
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


void BackupControllerConnection::HandleCmdUser5(mavlink_command_long_t commandLong)
{
    HFLogger::logMessage("Controller - KILL CRAFT");
    bool validKillCommand = true;

    union {
        float floatValue;
        unsigned char byteValue[4];
    } converter;


    converter.floatValue = commandLong.param1;
    if ((unsigned char)converter.byteValue[1] != (unsigned char)0x11)
      validKillCommand = false;

    if (converter.byteValue[0] != 0x1A)
      validKillCommand = false;


    converter.floatValue = commandLong.param2;
    if (converter.byteValue[2] != 0x21)
      validKillCommand = false;

    if (converter.byteValue[1] != 0x1A)
      validKillCommand = false;

    if (converter.byteValue[0] != 0x2B)
      validKillCommand = false;

    converter.floatValue = commandLong.param3;
    if (converter.byteValue[2] != 0x31)
      validKillCommand = false;

    if (converter.byteValue[1] != 0xB2)
      validKillCommand = false;

    if (converter.byteValue[0] != 0xB3)
      validKillCommand = false;

    converter.floatValue = commandLong.param4;
    if (converter.byteValue[0] != 0x00)
      validKillCommand = false;

    converter.floatValue = commandLong.param5;
    if (converter.byteValue[0] != 0x00)
      validKillCommand = false;

    converter.floatValue = commandLong.param6;
    if (converter.byteValue[0] != 0x00)
      validKillCommand = false;

    converter.floatValue = commandLong.param7;
    if (converter.byteValue[0] != 0x01)
      validKillCommand = false;


    if (validKillCommand == true)
    {
        HFCommand* cmd;
        cmd = new HFCommand(HFCMD_KILL_CRAFT);
        cmd->Enable = true;
        m_controllerData->QueueCommand(cmd);
        HFLogger::logMessage("Controller - KILL CRAFT");
    }


}


void BackupControllerConnection::HandleCmdDoMountControl(mavlink_command_long_t commandLong)
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


void BackupControllerConnection::HandleCmdNavRoi(mavlink_command_long_t commandLong)
{
    if (m_controllerData != nullptr)
    {
        m_controllerData->RoiLatitude = commandLong.param5;
        m_controllerData->RoiLongitude = commandLong.param6;
        m_controllerData->RoiAltitude = commandLong.param7;
        m_controllerData->NewRoi = true;
    }
}


