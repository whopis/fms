#ifndef CONTROLLERCONNECTION_H
#define CONTROLLERCONNECTION_H

#include "DeviceConnection.h"
#include "ControllerData.h"


class ControllerConnection : public DeviceConnection
{
    public:
        ControllerConnection();
        virtual ~ControllerConnection();


        bool DataStreamValid();

        void AttachControllerData(ControllerData* controllerData);

        void ClearManualCommands();

    protected:
        void HandleHeartbeat(mavlink_message_t* msg);
        void HandleManualControl(mavlink_message_t* msg);
        void HandleCommandLong(mavlink_message_t* msg);
        void HandleMissionItem(mavlink_message_t* msg);
        void HandleRcChannelsOverride(mavlink_message_t* msg);

        void HandleCmdComponentArmDisarm(mavlink_command_long_t commandLong);
        void HandleCmdNavTakeoff(mavlink_command_long_t commandLong);
        void HandleCmdNavLand(mavlink_command_long_t commandLong);
        void HandleCmdDoFollow(mavlink_command_long_t commandLong);
        void HandleCmdConditionChangeAlt(mavlink_command_long_t commandLong);
        void HandleCmdUser1(mavlink_command_long_t commandLong);
        void HandleCmdUser2(mavlink_command_long_t commandLong);
        void HandleCmdUser3(mavlink_command_long_t commandLong);
        void HandleCmdUser4(mavlink_command_long_t commandLong);
        void HandleCmdUser5(mavlink_command_long_t commandLong);
        void HandleCmdDoMountControl(mavlink_command_long_t commandLong);
        void HandleCmdNavRoi(mavlink_command_long_t commandLong);




        int16_t m_joyX;
        int16_t m_joyY;
        int16_t m_joyZ;
        int16_t m_joyR;

        std::string m_statusText;

        mavlink_heartbeat_t         m_heartbeat;
        mavlink_manual_control_t    m_manualControl;
        //mavlink_command_long_t      m_commandLong;
        mavlink_mission_item_t      m_missionItem;

        mavlink_rc_channels_override_t  m_rcChannelsOverride;

        ControllerData* m_controllerData;

    private:













};

#endif // CONTROLLERCONNECTION_H
