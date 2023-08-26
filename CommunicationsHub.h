#ifndef COMMUNICATIONSHUB_H
#define COMMUNICATIONSHUB_H

#include "ConnectionManager.h"
#include "GPSFactory.h"
#include "ControllerFactory.h"
#include "BackupControllerFactory.h"
#include "CraftFactory.h"
#include "ObserverFactory.h"
#include "BaroFactory.h"
#include "SkyBoxFactory.h"

#include "Craft.h"
#include "Controller.h"
#include "BackupController.h"
#include "GroundGPS.h"
#include "GroundBaro.h"
#include "SkyBox.h"

#include "SystemState.h"

#include <thread>


class CommunicationsHub
{
    public:
        CommunicationsHub();
        virtual ~CommunicationsHub();

        void Start();
        void Stop();

        ConnectionManager ControllerManager;
        ConnectionManager GroundGPSManager;
        ConnectionManager CraftManager;
        ConnectionManager ObserverManager;
        ConnectionManager GroundBaroManager;
        ConnectionManager BackupControllerManager;
        ConnectionManager SkyBoxManager;

        void SendHeartbeat(systemState_t systemState, uint32_t craft_custom_mode);
        void SendCraftGlobalPositionInt(int32_t latitude, int32_t longitude, int32_t relative_alt, uint16_t heading);
        void SendCraftGpsStatus(uint8_t fixType, uint8_t satellites, uint16_t hdop, uint16_t vdop);
        void SendGroundGlobalPositionInt(int32_t latitude, int32_t longitude, int32_t relative_alt, uint16_t heading);
        void SendGroundGpsStatus(uint8_t fixType, uint8_t satellites, uint16_t hdop, uint16_t vdop);
        void SendTargetGlobalPositionInt(int32_t latitude, int32_t longitude, int32_t relative_alt, uint16_t heading);
        void SendRcChannels(uint16_t* rcChannels);
        void SendCraftSystemStatus(uint16_t voltage_battery, uint16_t current_battery);
        void SendSystemTime(uint64_t time_unix_usec, uint32_t time_boot_ms);
        void SendRangefinder(float distance);
        void SendTetherLength(int32_t tetherLength);
        void SendCameraZoomScale(int zoomScale);
        void SendIrColorPalette(int currentColorPalette);
        void SendSkyBoxStatus(int rollTopPosition, int scissorLiftPosition, int moveState, int moveTimeRemaining);
        void SendLockerStatus(int moveState);
        void SendGimbalStatus001(float latitude, float longitude, float relative_alt, float heading, float tiltAngle);
        void SendGimbalStatus002(float hFoV_current, float vFoV_current, float hFoV_full, float vFoV_full, float magFactor);
        void SendWind(float windSpeed, float windDirection, float windBearing);
        void SendFlightLimits(float ceiling, float powerCeiling, bool precisionLandEnabled, bool precisionLandHealthy, int gpsOn);
        void SendBguStatus(float precisionLandStatus, int controllerGpsOn, int bguGpsOn);
        void SendFlightStatus(bool craftInitiatedLanding);

        void SendFlightStats(int bootCnt, int flightTime, int runTime, int lastReset);

        void SendReelStatus1(int state, int faultMask, int errorMask, int tetherPos, int tetherWarning, int motorOn);
        void SendReelStatus2(int voltTension, int volt12, int voltMotor, int currentMotor, int temp, int tempIr);


        void AttachCraft(Craft* craft);
        void AttachController(Controller* controller);
        void AttachBackupController(BackupController* backupController);
        void AttachGroundGps(GroundGPS* groundGps);
        void AttachGroundBaro(GroundBaro* groundBaro);
        void AttachSkyBox(SkyBox* skyBox);


        bool AttemptConnectToCraft();
        bool AttemptConnectToController();
        bool AttemptConnectToBackupController();
        bool AttemptConnectToGroundGps();
        bool AttemptConnectToGroundBaro();
        bool AttemptConnectToSkyBox();
        void SendStatusText(StatusText* statusText);

        bool IsControllerActive(int seconds);

        //bool IsEmergencyLandingRequested();
        bool IsEmergencyDescendRequested();
        int GetTetherLength();
        
        
        bool m_useControllerProxy;


    protected:

        ControllerFactory m_ControllerFactory;
        BackupControllerFactory m_BackupControllerFactory;
        GPSFactory m_GpsFactory;
        CraftFactory m_CraftFactory;
        ObserverFactory m_ObserverFactory;
        BaroFactory m_GroundBaroFactory;
        SkyBoxFactory m_SkyBoxFactory;

        ConcurrentQueue<StatusText*> m_statusTextQueue;

        Craft* m_craft;
        Controller* m_controller;
        BackupController* m_backupController;
        GroundGPS* m_groundGps;
        GroundBaro* m_groundBaro;
        SkyBox* m_skyBox;

        std::thread m_statusTextHandlerThread;
        bool m_runStatusTextHandler;

        void StatusTextHandler();

    private:
};

#endif // COMMUNICATIONSHUB_H
