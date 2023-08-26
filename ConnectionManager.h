#ifndef CONNECTIONMANAGER_H
#define CONNECTIONMANAGER_H

#include <vector>
#include <thread>
#include <netinet/in.h>

#include "DeviceConnectionFactory.h"

#include "SystemState.h"


typedef std::vector<DeviceConnection*> DeviceVector;


class ConnectionManager
{
    public:
        ConnectionManager();
        virtual ~ConnectionManager();

        void SetId(const char* id);

        void AttachConnectionFactory(DeviceConnectionFactory* factory);
        void AttachStatusTextQueue(ConcurrentQueue<StatusText*>* statusTextQueue);

        void MonitorConnections(in_addr_t ipAddress, int port);

        void StopMonitoring();

        int DeviceCount();
        DeviceConnection* GetDevice(int index);

        int ActiveCount(int maxSeconds);
        int StaleCount(int maxSeconds);

        DeviceConnection* GetFirstActiveConnection(int maxSeconds);
        DeviceConnection* GetLatestActiveConnection(int maxSeconds);
        DeviceConnection* GetFirstActiveMasterConnection(int maxSeconds);

        void SendHeartbeatAllConnections(systemState_t systemState, uint32_t craft_custom_mode);

        void SendCraftGlobalPositionAllConnections(int32_t latitude, int32_t longitude, int32_t relative_alt, uint16_t heading);
        void SendCraftGpsStatusAllConnections(uint8_t fixType, uint8_t satellites, uint16_t hdop, uint16_t vdop);
        void SendGroundGlobalPositionAllConnections(int32_t latitude, int32_t longitude, int32_t relative_alt, uint16_t heading);
        void SendGroundGpsStatusAllConnections(uint8_t fixType, uint8_t satellites, uint16_t hdop, uint16_t vdop);
        void SendTargetGlobalPositionAllConnections(int32_t latitude, int32_t longitude, int32_t relative_alt, uint16_t heading);
        void SendTetherLengthAllConnection(int32_t tetherLength);
        void SendRcChannelsAllConnections(uint16_t* rcChannels);
        void SendCraftSystemStatusAllConnections(uint16_t voltage_battery, uint16_t current_battery);
        void SendStatusTextAllConnection(uint8_t severity, char* text);
        void SendSystemTimeAllConnection(uint64_t unix_time_us, uint32_t boot_time_ms);
        void SendRangefinderAllConnection(float distance);
        void SendCameraZoomScaleAllConnections(int zoomScale);
        void SendIrColorPaletteAllConnections(int irColorPalette);
        void SendSkyBoxStatusAllConnections(int rollTopPosition, int scissorLiftPosition, int moveState, int moveTimeRemaining);
        void SendLockerStatusAllConnections(int moveState);
        void SendGimbalStatus001(float latitude, float longitude, float relative_alt, float heading, float tiltAngle);
        void SendGimbalStatus002(float hFoV_current, float vFoV_current, float hFoV_full, float vFoV_full, float magFactor);
        void SendWind(float windSpeed, float windDirection, float windBearing);
        void SendFlightLimits(float ceiling, float powerCeiling, bool precisionLandEnabled, bool precisionLandHealthy, int gpsOn);
        void SendFlightStatsAllConnections(int bootCnt, int flightTime, int runTime, int lastReset);
        void SendBguStatusAllConnections(float precisionLandStatus, int controllerGpsOn, int bguGpsOn);
        void SendFlightStatusAllConnections(bool craftInitiatedLanding);
        void SendReelStatus1AllConnections(int state, int faultMask, int errorMask, int tetherPos, int tetherWarning, int motorOn);
        void SendReelStatus2AllConnections(int voltTension, int volt12, int voltMotor, int currentMotor, int temp, int tempIr);
        

    protected:
    private:
        void ListenerLoop();

        DeviceConnectionFactory* m_factory;
        DeviceVector m_devices;
        in_addr_t m_ipAddress;
        int m_port;

        ConcurrentQueue<StatusText*>* m_statusTextQueue;

        std::thread m_listenerThread;

        bool m_runListenerThread;

        char m_connectionId[20];


};

#endif // CONNECTIONMANAGER_H



