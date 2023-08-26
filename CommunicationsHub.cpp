#include "CommunicationsHub.h"

#include <sys/socket.h>
#include <arpa/inet.h>

#include "HFLogger.h"


CommunicationsHub::CommunicationsHub()
{
    //ctor
    ControllerManager.SetId("ControllerManager");
    BackupControllerManager.SetId("BackupCtrlMgr");
    GroundGPSManager.SetId("GroundGPSManager");
    CraftManager.SetId("CraftManager");
    ObserverManager.SetId("ObserverManager");
    GroundBaroManager.SetId("GroundBaroManager");
    SkyBoxManager.SetId("SkyBoxManager");

    m_runStatusTextHandler = false;
    
    m_useControllerProxy = false;
}

CommunicationsHub::~CommunicationsHub()
{
    //dtor
}

bool CommunicationsHub::IsEmergencyDescendRequested()
{
    bool descendRequested = false;
    try
    {
        int observerCount = ObserverManager.DeviceCount();
        for (int i = 0; i < observerCount; i++)
        {
            ObserverConnection* oc = (ObserverConnection*)ObserverManager.GetDevice(i);
            if (oc->IsLowerAltitudeRequested() == true)
            {
                descendRequested = true;
            }
        }
    }
    catch (const std::exception& e)
    {
        descendRequested = false;
    }

    return descendRequested;
}

int CommunicationsHub::GetTetherLength()
{
    int tetherLength = 0;
    try
    {
        int observerCount = ObserverManager.DeviceCount();
        for (int i = 0; i < observerCount; i++)
        {
            ObserverConnection* oc = (ObserverConnection*)ObserverManager.GetDevice(i);
            if (oc->GetTetherLength() > 0)
            {
                tetherLength = oc->GetTetherLength();
            }
        }
    }
    catch (const std::exception& e)
    {
        tetherLength = 0;
    }

    return tetherLength;
}

/*
bool CommunicationsHub::IsEmergencyLandingRequested()
{
    bool landingRequested = false;
    try
    {
        int observerCount = ObserverManager.DeviceCount();
        for (int i = 0; i < observerCount; i++)
        {
            ObserverConnection* oc = (ObserverConnection*)ObserverManager.GetDevice(i);
            if (oc->IsLandingRequested() == true)
            {
                landingRequested = true;
            }
        }
    }
    catch (const std::exception& e)
    {
        landingRequested = false;
    }

    return landingRequested;
}
*/


bool CommunicationsHub::IsControllerActive(int seconds)
{
    bool active = false;
    if (ControllerManager.ActiveCount(seconds) > 0)
    {
        active = true;
    }

    return active;
}

void CommunicationsHub::AttachCraft(Craft* craft)
{
    m_craft = craft;
}


void CommunicationsHub::AttachController(Controller* controller)
{
    m_controller = controller;
}


void CommunicationsHub::AttachBackupController(BackupController* backupController)
{
    m_backupController = backupController;
}

void CommunicationsHub::AttachGroundGps(GroundGPS* groundGps)
{
    m_groundGps = groundGps;
}


void CommunicationsHub::AttachGroundBaro(GroundBaro* groundBaro)
{
    m_groundBaro = groundBaro;
}


void CommunicationsHub::AttachSkyBox(SkyBox* skyBox)
{
    m_skyBox = skyBox;
}


void CommunicationsHub::Start()
{

    HFLogger::logMessage("CommunicationsHub::Start");

    in_addr_t ipAddress = inet_addr("10.20.30.200");

    HFLogger::logMessage("start ControllerManager");
    ControllerManager.AttachConnectionFactory(&m_ControllerFactory);
    ControllerManager.AttachStatusTextQueue(&m_statusTextQueue);
    if (m_useControllerProxy == false) 
    {
        HFLogger::logMessage("ControllerManager using port 10150");
        ControllerManager.MonitorConnections(ipAddress, 10150);
    }
    else
    {
        HFLogger::logMessage("ControllerManager using port 10152");
        ControllerManager.MonitorConnections(ipAddress, 10152);
    }



    HFLogger::logMessage("start BackupControllerManager");
    BackupControllerManager.AttachConnectionFactory(&m_BackupControllerFactory);
    BackupControllerManager.AttachStatusTextQueue(&m_statusTextQueue);
    BackupControllerManager.MonitorConnections(ipAddress, 10151);


    HFLogger::logMessage("start GroundGPSManager");
    GroundGPSManager.AttachConnectionFactory(&m_GpsFactory);
    GroundGPSManager.AttachStatusTextQueue(&m_statusTextQueue);
    GroundGPSManager.MonitorConnections(ipAddress, 10110);


    HFLogger::logMessage("start CraftManager");
    CraftManager.AttachConnectionFactory(&m_CraftFactory);
    CraftManager.AttachStatusTextQueue(&m_statusTextQueue);
    CraftManager.MonitorConnections(ipAddress, 10100);


    HFLogger::logMessage("start ObserverManager");
    ObserverManager.AttachConnectionFactory(&m_ObserverFactory);
    ObserverManager.AttachStatusTextQueue(&m_statusTextQueue);
    ObserverManager.MonitorConnections(ipAddress, 10050);

    HFLogger::logMessage("start GroundBaroManager");
    GroundBaroManager.AttachConnectionFactory(&m_GroundBaroFactory);
    GroundBaroManager.AttachStatusTextQueue(&m_statusTextQueue);
    GroundBaroManager.MonitorConnections(ipAddress, 10120);


    HFLogger::logMessage("start SkyBoxManager");
    SkyBoxManager.AttachConnectionFactory(&m_SkyBoxFactory);
    SkyBoxManager.AttachStatusTextQueue(&m_statusTextQueue);
    SkyBoxManager.MonitorConnections(ipAddress, 10201);

    m_runStatusTextHandler = true;
    m_statusTextHandlerThread = std::thread(&CommunicationsHub::StatusTextHandler, this);

}


void CommunicationsHub::Stop()
{
    ControllerManager.StopMonitoring();
    GroundGPSManager.StopMonitoring();
    CraftManager.StopMonitoring();
    ObserverManager.StopMonitoring();
    GroundBaroManager.StopMonitoring();
    SkyBoxManager.StopMonitoring();
}


void CommunicationsHub::SendHeartbeat(systemState_t systemState, uint32_t craft_custom_mode)
{
    // Send heartbeats to all devices
    HFLogger::logMessage("Sending heartbeats");
    ControllerManager.SendHeartbeatAllConnections(systemState, craft_custom_mode);
    BackupControllerManager.SendHeartbeatAllConnections(systemState, craft_custom_mode);
    GroundGPSManager.SendHeartbeatAllConnections(systemState, craft_custom_mode);
    CraftManager.SendHeartbeatAllConnections(systemState, craft_custom_mode);
    ObserverManager.SendHeartbeatAllConnections(systemState, craft_custom_mode);
    GroundBaroManager.SendHeartbeatAllConnections(systemState, craft_custom_mode);
    SkyBoxManager.SendHeartbeatAllConnections(systemState, craft_custom_mode);
}


void CommunicationsHub::SendTetherLength(int32_t tetherLength)
{
    ControllerManager.SendTetherLengthAllConnection(tetherLength);
}


void CommunicationsHub::SendCraftGlobalPositionInt(int32_t latitude, int32_t longitude, int32_t relative_alt, uint16_t heading)
{
    ControllerManager.SendCraftGlobalPositionAllConnections(latitude, longitude, relative_alt, heading);
    BackupControllerManager.SendCraftGlobalPositionAllConnections(latitude, longitude, relative_alt, heading);
    ObserverManager.SendCraftGlobalPositionAllConnections(latitude, longitude, relative_alt, heading);
}

void CommunicationsHub::SendCraftGpsStatus(uint8_t fixType, uint8_t satellites, uint16_t hdop, uint16_t vdop)
{
    ControllerManager.SendCraftGpsStatusAllConnections(fixType, satellites, hdop, vdop);
    BackupControllerManager.SendCraftGpsStatusAllConnections(fixType, satellites, hdop, vdop);
    ObserverManager.SendCraftGpsStatusAllConnections(fixType, satellites, hdop, vdop);
}

void CommunicationsHub::SendGroundGlobalPositionInt(int32_t latitude, int32_t longitude, int32_t relative_alt, uint16_t heading)
{
    ControllerManager.SendGroundGlobalPositionAllConnections(latitude, longitude, relative_alt, heading);
    BackupControllerManager.SendGroundGlobalPositionAllConnections(latitude, longitude, relative_alt, heading);
    ObserverManager.SendGroundGlobalPositionAllConnections(latitude, longitude, relative_alt, heading);
}

void CommunicationsHub::SendGroundGpsStatus(uint8_t fixType, uint8_t satellites, uint16_t hdop, uint16_t vdop)
{
    ControllerManager.SendGroundGpsStatusAllConnections(fixType, satellites, hdop, vdop);
    BackupControllerManager.SendGroundGpsStatusAllConnections(fixType, satellites, hdop, vdop);
    ObserverManager.SendGroundGpsStatusAllConnections(fixType, satellites, hdop, vdop);
}

void CommunicationsHub::SendTargetGlobalPositionInt(int32_t latitude, int32_t longitude, int32_t relative_alt, uint16_t heading)
{
    ControllerManager.SendTargetGlobalPositionAllConnections(latitude, longitude, relative_alt, heading);
    BackupControllerManager.SendTargetGlobalPositionAllConnections(latitude, longitude, relative_alt, heading);
    ObserverManager.SendTargetGlobalPositionAllConnections(latitude, longitude, relative_alt, heading);
}

void CommunicationsHub::SendRcChannels(uint16_t* rcChannels)
{
    ControllerManager.SendRcChannelsAllConnections(rcChannels);
    BackupControllerManager.SendRcChannelsAllConnections(rcChannels);
    ObserverManager.SendRcChannelsAllConnections(rcChannels);
}

void CommunicationsHub::SendCraftSystemStatus(uint16_t voltage_battery, uint16_t current_battery)
{
    ControllerManager.SendCraftSystemStatusAllConnections(voltage_battery, current_battery);
    BackupControllerManager.SendCraftSystemStatusAllConnections(voltage_battery, current_battery);
    ObserverManager.SendCraftSystemStatusAllConnections(voltage_battery, current_battery);
}


void CommunicationsHub::SendSystemTime(uint64_t time_unix_usec, uint32_t time_boot_ms)
{
    ControllerManager.SendSystemTimeAllConnection(time_unix_usec, time_boot_ms);
    BackupControllerManager.SendSystemTimeAllConnection(time_unix_usec, time_boot_ms);
    ObserverManager.SendSystemTimeAllConnection(time_unix_usec, time_boot_ms);
}

void CommunicationsHub::SendRangefinder(float distance)
{
    ControllerManager.SendRangefinderAllConnection(distance);
    BackupControllerManager.SendRangefinderAllConnection(distance);
    ObserverManager.SendRangefinderAllConnection(distance);
}


void CommunicationsHub::SendCameraZoomScale(int zoomScale)
{
    ControllerManager.SendCameraZoomScaleAllConnections(zoomScale);
    BackupControllerManager.SendCameraZoomScaleAllConnections(zoomScale);
    ObserverManager.SendCameraZoomScaleAllConnections(zoomScale);
}

void CommunicationsHub::SendIrColorPalette(int currentColorPalette)
{
    ControllerManager.SendIrColorPaletteAllConnections(currentColorPalette);
    BackupControllerManager.SendIrColorPaletteAllConnections(currentColorPalette);
    ObserverManager.SendIrColorPaletteAllConnections(currentColorPalette);
}


void CommunicationsHub::SendSkyBoxStatus(int rollTopPosition, int scissorLiftPosition, int moveState, int moveTimeRemaining)
{
    ControllerManager.SendSkyBoxStatusAllConnections(rollTopPosition, scissorLiftPosition, moveState, moveTimeRemaining);
    BackupControllerManager.SendSkyBoxStatusAllConnections(rollTopPosition, scissorLiftPosition, moveState, moveTimeRemaining);
    ObserverManager.SendSkyBoxStatusAllConnections(rollTopPosition, scissorLiftPosition, moveState, moveTimeRemaining);
}

void CommunicationsHub::SendLockerStatus(int moveState)
{
    ControllerManager.SendLockerStatusAllConnections(moveState);
    BackupControllerManager.SendLockerStatusAllConnections(moveState);
    ObserverManager.SendLockerStatusAllConnections(moveState);
}

void CommunicationsHub::SendGimbalStatus001(float latitude, float longitude, float relative_alt, float heading, float tiltAngle)
{
    ControllerManager.SendGimbalStatus001(latitude, longitude, relative_alt, heading, tiltAngle);
    BackupControllerManager.SendGimbalStatus001(latitude, longitude, relative_alt, heading, tiltAngle);
    ObserverManager.SendGimbalStatus001(latitude, longitude, relative_alt, heading, tiltAngle);
}

void CommunicationsHub::SendGimbalStatus002(float hFoV_current, float vFoV_current, float hFoV_full, float vFoV_full, float magFactor)
{
    ControllerManager.SendGimbalStatus002(hFoV_current, vFoV_current, hFoV_full, vFoV_full, magFactor);
    BackupControllerManager.SendGimbalStatus002(hFoV_current, vFoV_current, hFoV_full, vFoV_full, magFactor);
    ObserverManager.SendGimbalStatus002(hFoV_current, vFoV_current, hFoV_full, vFoV_full, magFactor);
}


void CommunicationsHub::SendWind(float windSpeed, float windDirection, float windBearing)
{
    ControllerManager.SendWind(windSpeed, windDirection, windBearing);
    BackupControllerManager.SendWind(windSpeed, windDirection, windBearing);
    ObserverManager.SendWind(windSpeed, windDirection, windBearing);
}


void CommunicationsHub::SendFlightLimits(float ceiling, float powerCeiling, bool precisionLandEnabled, bool precisionLandHealthy, int gpsOn)
{
    ControllerManager.SendFlightLimits(ceiling, powerCeiling, precisionLandEnabled, precisionLandHealthy, gpsOn);
    BackupControllerManager.SendFlightLimits(ceiling, powerCeiling, precisionLandEnabled, precisionLandHealthy, gpsOn);
    ObserverManager.SendFlightLimits(ceiling, powerCeiling, precisionLandEnabled, precisionLandHealthy, gpsOn);
}

void CommunicationsHub::SendFlightStats(int bootCnt, int flightTime, int runTime, int lastReset)
{
    ControllerManager.SendFlightStatsAllConnections(bootCnt, flightTime, runTime, lastReset);
    BackupControllerManager.SendFlightStatsAllConnections(bootCnt, flightTime, runTime, lastReset);
    ObserverManager.SendFlightStatsAllConnections(bootCnt, flightTime, runTime, lastReset);
}


void CommunicationsHub::SendReelStatus1(int state, int faultMask, int errorMask, int tetherPos, int tetherWarning, int motorOn)
{
    ControllerManager.SendReelStatus1AllConnections(state, faultMask, errorMask, tetherPos, tetherWarning, motorOn);
    BackupControllerManager.SendReelStatus1AllConnections(state, faultMask, errorMask, tetherPos, tetherWarning, motorOn);
    ObserverManager.SendReelStatus1AllConnections(state, faultMask, errorMask, tetherPos, tetherWarning, motorOn);
}



void CommunicationsHub::SendReelStatus2(int voltTension, int volt12, int voltMotor, int currentMotor, int temp, int tempIr)
{
    ControllerManager.SendReelStatus2AllConnections(voltTension, volt12, voltMotor, currentMotor, temp, tempIr);
    BackupControllerManager.SendReelStatus2AllConnections(voltTension, volt12, voltMotor, currentMotor, temp, tempIr);
    ObserverManager.SendReelStatus2AllConnections(voltTension, volt12, voltMotor, currentMotor, temp, tempIr);
}


        void SendReelStatus1(int state, int faultMask, int errorMask, int tetherPos, int tetherWarning, int motorOn);
        void SendReelStatus2(int voltTension, int volt12, int voltMotor, int currentMotor, int temp, int tempIr);


void CommunicationsHub::SendBguStatus(float precisionLandStatus, int controllerGpsOn, int bguGpsOn)
{
    ControllerManager.SendBguStatusAllConnections(precisionLandStatus, controllerGpsOn, bguGpsOn);
    BackupControllerManager.SendBguStatusAllConnections(precisionLandStatus, controllerGpsOn, bguGpsOn);
    ObserverManager.SendBguStatusAllConnections(precisionLandStatus, controllerGpsOn, bguGpsOn);
}

void CommunicationsHub::SendFlightStatus(bool craftInitiatedLanding)
{
    ControllerManager.SendFlightStatusAllConnections(craftInitiatedLanding);
    BackupControllerManager.SendFlightStatusAllConnections(craftInitiatedLanding);
    ObserverManager.SendFlightStatusAllConnections(craftInitiatedLanding);    
}

bool CommunicationsHub::AttemptConnectToCraft()
{
    bool success = false;

//    DeviceConnection* device = CraftManager.GetFirstActiveConnection(5);
    DeviceConnection* device = CraftManager.GetLatestActiveConnection(5);

    if (device != nullptr)
    {
        m_craft->AttachConnection((CraftConnection*)device);
        success = true;
    }

    return success;
}


bool CommunicationsHub::AttemptConnectToController()
{
    bool success = false;

    // Need to make sure this is a true controller - not just a slave
//    DeviceConnection* device = ControllerManager.GetFirstActiveConnection(5);
    DeviceConnection* device = ControllerManager.GetFirstActiveMasterConnection(5);

    if (device != nullptr)
    {
        HFLogger::logMessage("Connecting to controller");
        m_controller->AttachConnection((ControllerConnection*)device);
        success = true;
    }

    return success;

}


bool CommunicationsHub::AttemptConnectToBackupController()
{
    bool success = false;

    // Need to make sure this is a true controller - not just a slave
    DeviceConnection* device = BackupControllerManager.GetFirstActiveMasterConnection(5);

    if (device != nullptr)
    {
        m_backupController->AttachConnection((BackupControllerConnection*)device);
        success = true;
    }

    return success;

}


bool CommunicationsHub::AttemptConnectToGroundGps()
{
    bool success = false;

    DeviceConnection* device = GroundGPSManager.GetFirstActiveConnection(5);

    if (device != nullptr)
    {
        m_groundGps->AttachConnection((GPSConnection*)device);
        success = true;
    }

    return success;
}



bool CommunicationsHub::AttemptConnectToGroundBaro()
{
    bool success = false;

    DeviceConnection* device = GroundBaroManager.GetFirstActiveConnection(5);

    if (device != nullptr)
    {
        m_groundBaro->AttachConnection((BaroConnection*)device);
        success = true;
    }

    return success;
}



bool CommunicationsHub::AttemptConnectToSkyBox()
{
    bool success = false;

    DeviceConnection* device = SkyBoxManager.GetFirstActiveConnection(5);

    if (device != nullptr)
    {
        m_skyBox->AttachConnection((SkyBoxConnection*)device);
        success = true;
    }

    return success;
}

void CommunicationsHub::StatusTextHandler()
{
    while (m_runStatusTextHandler == true)
    {
        StatusText* statusText;
        if (m_statusTextQueue.Dequeue(statusText) == true)
        {
            HFLogger::logMessage("repeating status text");
            // Send this status text to all controllers
            ControllerManager.SendStatusTextAllConnection(statusText->severity, statusText->text);
            ObserverManager.SendStatusTextAllConnection(statusText->severity, statusText->text);
            delete statusText;
        }
    }
}

void CommunicationsHub::SendStatusText(StatusText *statusText)
{
    m_statusTextQueue.Enqueue(statusText);
}


