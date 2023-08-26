#include "ConnectionManager.h"
#include "TcpDataHandler.h"
#include "MAVLinkHandler.h"

#include <string.h>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include "HFLogger.h"

#include "ControllerFactory.h"



ConnectionManager::ConnectionManager()
{
    //ctor
    strncpy(m_connectionId, "id not set", 20);

    m_statusTextQueue = nullptr;
}

ConnectionManager::~ConnectionManager()
{
    //dtor
}       


void ConnectionManager::SetId(const char* id)
{
    strncpy(m_connectionId, id, 20);
}

void ConnectionManager::AttachConnectionFactory(DeviceConnectionFactory* factory)
{
    m_factory = factory;
}

void ConnectionManager::AttachStatusTextQueue(ConcurrentQueue<StatusText*>* statusTextQueue)
{
    m_statusTextQueue = statusTextQueue;
}


void ConnectionManager::MonitorConnections(in_addr_t ipAddress, int port)
{
    m_ipAddress = ipAddress;
    m_port = port;

    // Start a thread that will listen for connections
    // and spawn new Controller objects as needed
    m_listenerThread = std::thread(&ConnectionManager::ListenerLoop, this);

}

void ConnectionManager::StopMonitoring()
{
    m_runListenerThread = false;
    m_listenerThread.join();
}


int ConnectionManager::DeviceCount()
{
    return m_devices.size();
}

int ConnectionManager::ActiveCount(int maxSeconds)
{
    int count = 0;
    for (unsigned int i = 0; i < m_devices.size(); i++)
    {
        if (m_devices[i]->IsConnectionActive(maxSeconds) == true)
        {
            count++;
        }
    }
    return count;
}

int ConnectionManager::StaleCount(int maxSeconds)
{
    int count = 0;
    for (unsigned int i = 0; i < m_devices.size(); i++)
    {
        if (m_devices[i]->IsConnectionActive(maxSeconds) == false)
        {
            count++;
        }
    }
    return count;
}

DeviceConnection* ConnectionManager::GetFirstActiveConnection(int maxSeconds)
{
    DeviceConnection* device = nullptr;

    unsigned int i = 0;
    while ((i < m_devices.size()) && (device == nullptr))
    {
        if (m_devices[i]->IsConnectionActive(maxSeconds) == true)
        {
            device = m_devices[i];
        }

        i++;
    }

    return device;
}


DeviceConnection* ConnectionManager::GetLatestActiveConnection(int maxSeconds)
{
    DeviceConnection* device = nullptr;

    // Go backwards through list
    unsigned int i = m_devices.size();
    while ((i > 0) && (device == nullptr))
    {
        i--;
        
        if (m_devices[i]->IsConnectionActive(maxSeconds) == true)
        {
            device = m_devices[i];
        }

    }

    return device;
}



DeviceConnection* ConnectionManager::GetFirstActiveMasterConnection(int maxSeconds)
{
    DeviceConnection* device = nullptr;

    unsigned int i = 0;
    while ((i < m_devices.size()) && (device == nullptr))
    {
        if (m_devices[i]->IsConnectionActive(maxSeconds) == true)
        {
            if (m_devices[i]->Master == true)
            {
                device = m_devices[i];
            }
        }

        i++;
    }

    return device;
}



DeviceConnection* ConnectionManager::GetDevice(int index)
{
    return m_devices[index];
}


void ConnectionManager::ListenerLoop()
{
    int listenerfd = 0;
    int connfd = 0;
    struct sockaddr_in serv_addr;

    listenerfd = socket(AF_INET, SOCK_STREAM, 0);
    fcntl(listenerfd, F_SETFL, O_NONBLOCK);
    memset(&serv_addr, 0, sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = m_ipAddress;
    serv_addr.sin_port = htons(m_port);

    bind(listenerfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));

    listen(listenerfd, 10);

    HFLogger::logMessage("%s: listening at %s:%d", m_connectionId, inet_ntoa(serv_addr.sin_addr), m_port);
    m_runListenerThread = true;
    while (m_runListenerThread == true)
    {
        // Block until a connection is accepted
        connfd = accept(listenerfd, (struct sockaddr*)NULL, NULL);
        if (connfd != -1)
        {
            struct sockaddr_in inSock;
            socklen_t inSockLen = sizeof(struct sockaddr_in);

            int res = getpeername(connfd, (sockaddr*)&inSock, &inSockLen);
            if (res == 0)
            {
                HFLogger::logMessage("%s: connection from %s:%d on port %d", m_connectionId, inet_ntoa(inSock.sin_addr), inSock.sin_port, m_port);
            }
            else
            {
                HFLogger::logMessage("%s: unable to determine source address", m_connectionId);
            }


            // Create a TCP data handler with this connection
            TcpDataHandler* tcpDataHandler = new TcpDataHandler();
            tcpDataHandler->AttachSocket(connfd);

            // Create a MAVLinkHandler that uses this TCPDataHandler
            MAVLinkHandler* mavLinkHandler = new MAVLinkHandler(tcpDataHandler);
            HFLogger::logMessage("ConnectionManager: Create MAVLinkHandler id %s, parent %08x , handler %08x", m_connectionId, this, mavLinkHandler);

            // Create a controller object with this MAVLinkHandler
            DeviceConnection* device = m_factory->CreateDeviceConnection();
            device->AttachMAVLinkHandler(mavLinkHandler);
            device->AttachStatusTextQueue(m_statusTextQueue);

            device->Start();

            // Add the controller the vector
            m_devices.push_back(device);


        }
        else
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

    }

    while (m_devices.size() > 0)
    {
        DeviceConnection* device = m_devices.back();
        device->Stop();
        m_devices.pop_back();
        delete device;
    }
}


void ConnectionManager::SendHeartbeatAllConnections(systemState_t systemState, uint32_t craft_custom_mode)
{
    // Attempt in a try-catch block in case device disconnects
    uint8_t flightMode = (uint8_t) craft_custom_mode;
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendHeartbeat(Source_FMS, (uint32_t)systemState, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, flightMode, MAV_STATE_ACTIVE);
        }
    }
    catch (const std::exception& e)
    {
    }
}


void ConnectionManager::SendTetherLengthAllConnection(int32_t tetherLength)
{
    // Attempt in a try-catch block in case device disconnects
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendGlobalPositionInt(Source_TK, 0, 0, 0, tetherLength, tetherLength, 0, 0, 0, 0);
        }
    }
    catch (const std::exception& e)
    {
    }
}



void ConnectionManager::SendCraftGlobalPositionAllConnections(int32_t latitude, int32_t longitude, int32_t relative_alt, uint16_t heading)
{
    // Attempt in a try-catch block in case device disconnects
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendGlobalPositionInt(Source_Craft, 0, latitude, longitude, relative_alt, relative_alt, 0, 0, 0, heading);
        }
    }
    catch (const std::exception& e)
    {
    }
}


void ConnectionManager::SendCraftGpsStatusAllConnections(uint8_t fixType, uint8_t satellites, uint16_t hdop, uint16_t vdop)
{
    // Attempt in a try-catch block in case device disconnects
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendGpsRawInt(Source_Craft, 0, fixType, 0, 0, 0, hdop, vdop, 0, 0, satellites);
        }
    }
    catch (const std::exception& e)
    {
    }
}

void ConnectionManager::SendGroundGlobalPositionAllConnections(int32_t latitude, int32_t longitude, int32_t relative_alt, uint16_t heading)
{
    // Attempt in a try-catch block in case device disconnects
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendGlobalPositionInt(Source_GroundGps, 0, latitude, longitude, relative_alt, relative_alt, 0, 0, 0, heading);
        }
    }
    catch (const std::exception& e)
    {
    }
}

void ConnectionManager::SendGroundGpsStatusAllConnections(uint8_t fixType, uint8_t satellites, uint16_t hdop, uint16_t vdop)
{
    // Attempt in a try-catch block in case device disconnects
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendGpsRawInt(Source_GroundGps, 0, fixType, 0, 0, 0, hdop, vdop, 0, 0, satellites);
        }
    }
    catch (const std::exception& e)
    {
    }
}

void ConnectionManager::SendTargetGlobalPositionAllConnections(int32_t latitude, int32_t longitude, int32_t relative_alt, uint16_t heading)
{
    // Attempt in a try-catch block in case device disconnects
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendGlobalPositionInt(Source_Target, 0, latitude, longitude, relative_alt, relative_alt, 0, 0, 0, heading);
        }
    }
    catch (const std::exception& e)
    {
    }
}

void ConnectionManager::SendRcChannelsAllConnections(uint16_t* rcChannels)
{
    // Attempt in a try-catch block in case device disconnects
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendRCChannelsRaw(Source_FMS, 0, 0, rcChannels[0], rcChannels[1], rcChannels[2], rcChannels[3], rcChannels[4], rcChannels[5], rcChannels[6], rcChannels[7], 0);
        }
    }
    catch (const std::exception& e)
    {
    }
}

void ConnectionManager::SendCraftSystemStatusAllConnections(uint16_t voltage_battery, uint16_t current_battery)
{
    // Attempt in a try-catch block in case device disconnects
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendSysStatus(Source_FMS, 0, 0, 0, 0, voltage_battery, current_battery, 0, 0, 0, 0, 0, 0, 0);
        }
    }
    catch (const std::exception& e)
    {
    }
}


void ConnectionManager::SendStatusTextAllConnection(uint8_t severity, char* text)
{
    // Attempt in a try-catch block in case device disconnects
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendStatusText(Source_FMS, severity, text);
        }
    }
    catch (const std::exception& e)
    {
    }
}


void ConnectionManager::SendSystemTimeAllConnection(uint64_t unix_time_us, uint32_t boot_time_ms)
{
    // Attempt in a try-catch block in case device disconnects
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendSystemTime(Source_FMS, unix_time_us, boot_time_ms);
        }
    }
    catch (const std::exception& e)
    {
    }
}


void ConnectionManager::SendRangefinderAllConnection(float distance)
{
    // Attempt in a try-catch block in case device disconnects
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendRangefinder(Source_FMS, distance);
        }
    }
    catch (const std::exception& e)
    {
    }
}



void ConnectionManager::SendCameraZoomScaleAllConnections(int zoomScale)
{
    // Attempt in a try-catch block in case device disconnects
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendZoomScale(Source_FMS, zoomScale);
        }
    }
    catch (const std::exception& e)
    {
    }
}


void ConnectionManager::SendIrColorPaletteAllConnections(int irColorPalette)
{
    // Attempt in a try-catch block in case device disconnects
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendIrColorPalette(Source_FMS, irColorPalette);
        }
    }
    catch (const std::exception& e)
    {
    }
}

void ConnectionManager::SendSkyBoxStatusAllConnections(int rollTopPosition, int scissorLiftPosition, int moveState, int moveTimeRemaining)
{
    // Attempt in a try-catch block in case device disconnects
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendSkyBoxStatusAllConnections(Source_FMS, rollTopPosition, scissorLiftPosition, moveState, moveTimeRemaining);
        }
    }
    catch (const std::exception& e)
    {
    }
}


void ConnectionManager::SendLockerStatusAllConnections(int moveState)
{
    // Attempt in a try-catch block in case device disconnects
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendLockerStatusAllConnections(Source_FMS, moveState);
        }
    }
    catch (const std::exception& e)
    {
    }
}

void ConnectionManager::SendGimbalStatus001(float latitude, float longitude, float relative_alt, float heading, float tiltAngle)
{
    // Attempt in a try-catch block in case device disconnects
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendGimbalStatus001(Source_FMS, latitude, longitude, relative_alt, heading, tiltAngle);
        }
    }
    catch (const std::exception& e)
    {
    }
}

void ConnectionManager::SendGimbalStatus002(float hFoV_current, float vFoV_current, float hFoV_full, float vFoV_full, float magFactor)
{
    // Attempt in a try-catch block in case device disconnects
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendGimbalStatus002(Source_FMS, hFoV_current, vFoV_current, hFoV_full, vFoV_full, magFactor);
        }
    }
    catch (const std::exception& e)
    {
    }
}


void ConnectionManager::SendWind(float windSpeed, float windDirection, float windBearing)
{
    // Attempt in a try-catch block in case device disconnects
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendWind(Source_FMS, windSpeed, windDirection, windBearing);
        }
    }
    catch (const std::exception& e)
    {
    }    
}


void ConnectionManager::SendFlightLimits(float ceiling, float powerCeiling, bool precisionLandEnabled, bool precisionLandHealthy, int gpsOn)
{
    // Attempt in a try-catch block in case device disconnects
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendFlightLimits(Source_FMS, ceiling, powerCeiling, precisionLandEnabled, precisionLandHealthy, gpsOn);
        }
    }
    catch (const std::exception& e)
    {
    }    
}



void ConnectionManager::SendFlightStatsAllConnections(int bootCnt, int flightTime, int runTime, int lastReset)
{
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
            {
                char paramId[16];

                strncpy(paramId, "STAT_BOOTCNT", 16);
                m_devices[i]->SendParamValue(Source_FMS, paramId, bootCnt, MAV_PARAM_TYPE_UINT32);

                strncpy(paramId, "STAT_FLTTIME", 16);
                m_devices[i]->SendParamValue(Source_FMS, paramId, flightTime, MAV_PARAM_TYPE_UINT32);

                strncpy(paramId, "STAT_RUNTIME", 16);
                m_devices[i]->SendParamValue(Source_FMS, paramId, runTime, MAV_PARAM_TYPE_UINT32);

                strncpy(paramId, "STAT_RESET", 16);
                m_devices[i]->SendParamValue(Source_FMS, paramId, lastReset, MAV_PARAM_TYPE_UINT32);
            }
        }
    }
    catch (const std::exception& e)
    {
    }
}


void ConnectionManager::SendBguStatusAllConnections(float precisionLandStatus, int controllerGpsOn, int bguGpsOn)
{
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendBguStatus(Source_FMS, precisionLandStatus, controllerGpsOn, bguGpsOn);
        }
    }
    catch (const std::exception& e)
    {
    }
}


void ConnectionManager::SendFlightStatusAllConnections(bool craftInitiatedLanding)
{
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendFlightStatus(Source_FMS, craftInitiatedLanding);
        }
    }
    catch (const std::exception& e)
    {
    }    
}



void ConnectionManager::SendReelStatus1AllConnections(int state, int faultMask, int errorMask, int tetherPos, int tetherWarning, int motorOn)
{
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendReelStatus1(Source_FMS, state, faultMask, errorMask, tetherPos, tetherWarning, motorOn);
        }
    }
    catch (const std::exception& e)
    {
    }   
}


void ConnectionManager::SendReelStatus2AllConnections(int voltTension, int volt12, int voltMotor, int currentMotor, int temp, int tempIr)
{
    try
    {
        for (unsigned int i = 0; i < m_devices.size(); i++)
        {
            if (m_devices[i]->IsConnectionActive(5) == true)
                m_devices[i]->SendReelStatus2(Source_FMS, voltTension, volt12, voltMotor, currentMotor, temp, tempIr);
        }
    }
    catch (const std::exception& e)
    {
    }       
}
        
        
        
