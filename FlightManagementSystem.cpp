#include "FlightManagementSystem.h"

#include <math.h>

#include "HFLogger.h"

#include <python2.7/Python.h>

#include "Version.h"

//// 20180516 001 PLS
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
//// 20180516 001 END

#include <curl/curl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/writer.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"

using namespace rapidjson;

using namespace std::chrono;

FlightManagementSystem::FlightManagementSystem()
{
    HFLogger::logMessage("FlightManagementSystem: %08x", this);
    //ctor
    m_systemState = notSet;

    m_craftConnectionState = noConnection;
    m_groundGpsConnectionState = noConnection;
    m_controllerConnectionState = noConnection;
    m_groundBaroConnectionState = noConnection;
    m_skyBoxConnectionState = noConnection;

    // Connect the backup controller data to the actual controller data

    m_backupController.Data = &(m_controller.Data);


    m_communicationsHub.AttachController(&m_controller);
    m_communicationsHub.AttachBackupController(&m_backupController);
    m_communicationsHub.AttachCraft(&m_craft);
    m_communicationsHub.AttachGroundGps(&m_groundGPS);
    m_communicationsHub.AttachGroundBaro(&m_groundBaro);
    m_communicationsHub.AttachSkyBox(&m_skyBox);

    m_controllerConnectionMade = false;
    m_craftConnectionMade = false;
    m_groundGpsConnectionMade = false;
    m_groundBaroConnectionMade = false;
    m_skyBoxConnectionMade = false;

    m_precisionLanding = false;
    m_noGroundGps = false;
    m_httpZoom = false;

    m_baroCompensate = false;

    m_yawFollowsPan = false;

    m_zoomState = 0;


    m_useRemoteJoystick = false;

    m_flightStarted = false;

    m_maxAltitude = 6096;

    m_demoMode = false;

    m_useV35Plus = false;

    m_useExtendedChannels = false;

    m_requireBackupController = false;

    m_disableVoltageCheck = false;
    m_bigSky = false;

    m_abortPL = false;

    m_useSkyBox = false;
    m_useLocker = false;

    m_noLocationLogging = false;
    
    m_usePowerLimitCeiling = false;
    m_powerLimitFloor = 10.0f;
    m_powerLimitDelta = 10.0f;
    m_powerLimitCountMax = 1;

	m_zoomLevel = 0;
	m_zoomTarget = 0;
	memset(curl_buffer, 0x00, sizeof(curl_buffer));

    m_stepZoomOpticalTarget1 = 0;
    m_stepZoomOpticalLevel1 = 0;
    m_stepZoomDigitalTarget1 = 0;
    m_stepZoomDigitalLevel1 = 0;
    m_stepZoomOpticalTarget2 = 0;
    m_stepZoomOpticalLevel2 = 0;

	m_runCameraFinderLoop = false;
	m_cameraFinderStart = false;
	// m_availableCameras[] initialize in finder thread

	m_emergencyLand = false;

	currentColorPalette = 101;
	lastColorPalette = 101;

    currentZoomLevel = 0;
	//currentVisibleZoomLevel = 0;
	//currentIRZoomLevel = 0;

	InitGimbalStatus();

	m_slewToLandHeadingFirstTime = true;
    m_slewToTkHeadingFirstTime = true;

	//m_useLandingHeading = false;
	m_useLandingHeading = true;
	m_landingHeadingTarget = 0;

	m_lostCommTimeout = 5;

	m_rpiTemperature = -100.0;
    
    sawtoothStepCount = 0;
    
    
    // Set the power loss limit max count
    powerLossMonitor.SetCountLimit(m_powerLimitCountMax);
    
    // Set the power loss descent counter to 5 seconds
    powerLossMonitor.SetDelayTimeout(5);
    
    
    // Set the allowable vibration threshold
    m_vibrationThresholdXY = 15.0;
    m_vibrationThresholdZ = 50.0;
    m_vibrationFilterLength = 20;
    
    // Set the min altitude to 5m (500cm)
    m_minAltitude = 500;

    m_yawRealignmentAltitude = 4.5f;
    
    m_landingRetryEnabled = false;
    
    craftInitiatedLanding = false;
    
    m_useControllerProxy = false;
    
    
    m_useAlignTkHeading = false;
    m_tkHeadingTarget = 0;
    m_tkHeadingYawSlewRate = 25;
    m_tkHeadingPanSlewRate = 25;

    landHeadingCraftInitial = 0;
    landHeadingCraftAfterRealign = 0;
    landHeadingCraftDelta = 0;
    
    
    m_gpsDeniedSupport = false;
    m_gpsDeniedYawSlewMaxRate = 25;
    m_gpsDeniedPanSlewMaxRate = 100;
    m_gpsDeniedMaxAltitude = 4000;
    
    m_launchClimbRate = 70;
    m_flightClimbRate = 70;
    m_commLossLandAbort = false;

    m_gpsDeniedLandOnLoss = true;
    
    
    commReconnectCheckComplete = false;
    

}// end constructor

FlightManagementSystem::~FlightManagementSystem()
{
    //dtor
}// end destructor


int FlightManagementSystem::VerifyFlightVoltage()
{

    HFLogger::logMessage("VerifyFlightVoltage: Start");

 
    int minVoltage;
    int maxVoltage;

    if (m_bigSky == false) {
        minVoltage = 22000;
        maxVoltage = 28000;
    } else {
        minVoltage = 44000;
        maxVoltage = 56000;
    }



    int voltageRange = 0;


    HFLogger::logMessage("VerifyFlightVoltage: voltage = %d", m_craft.Data.batteryVoltage);
    
    if (m_craft.Data.batteryVoltage < minVoltage)
    {
        voltageRange = -1;        
        HFLogger::logMessage("VerifyFlightVoltage: Low Flight Voltage - %d", m_craft.Data.batteryVoltage);
    }

    if (m_craft.Data.batteryVoltage > maxVoltage)
    {
        //  voltageRange = 1;
        // This check did not work well - made system too sensitive to high voltage
        HFLogger::logMessage("VerifyFlightVoltage: High Flight Voltage - %d  (check disabled)", m_craft.Data.batteryVoltage);
    }

    return voltageRange;
}// end VerifyFlightVoltage


bool FlightManagementSystem::IsBadBatteryVoltage()
{
    int minVoltage;
    int maxVoltage;

    if (m_bigSky == false) {
        minVoltage = 8000;
        maxVoltage = 12000;
    } else {
        minVoltage = 16000;
        maxVoltage = 24000;
    }

    if ((m_craft.Data.batteryVoltage < maxVoltage) && (m_craft.Data.batteryVoltage > minVoltage))
    {
        return true;
    }
    else
    {
        return false;
    }
}



bool FlightManagementSystem::IsVoltageGoodToArm()
{
    int minVoltage;
    int maxVoltage;

    if (m_bigSky == false) {
        minVoltage = 20000;
        maxVoltage = 33000;
    } else {
        minVoltage = 40000;
        maxVoltage = 66000;
    }

    bool goodVoltage = true;


    if (m_craft.Data.batteryVoltage < minVoltage)
    {
        goodVoltage = false;
    }

    if (m_craft.Data.batteryVoltage > maxVoltage)
    {
        goodVoltage = false;
    }

    if (goodVoltage == false)
    {
        HFLogger::logMessage("Bad Voltage - %d", m_craft.Data.batteryVoltage);
    }

    return goodVoltage;
}


bool FlightManagementSystem::IsVoltageRangeGood()
{
    if (m_disableVoltageCheck == true) {
        return true;
    }

    int minVoltage;
    int maxVoltage;

    if (m_bigSky == false) {
        minVoltage = 20000;
        maxVoltage = 33000;
    } else {
        minVoltage = 40000;
        maxVoltage = 66000;
    }

    bool goodVoltage = true;


    if (m_craft.Data.batteryVoltage < minVoltage)
    {
        goodVoltage = false;
    }

    if (m_craft.Data.batteryVoltage > maxVoltage)
    {
        goodVoltage = false;
    }

    if (goodVoltage == false)
    {
        HFLogger::logMessage("Bad Voltage - %d", m_craft.Data.batteryVoltage);
    }

    return goodVoltage;
}// end IsVOltageRangeGood


bool FlightManagementSystem::CriticalVoltageCheck()
{
      bool goodVoltage = true;
      
      HFLogger::logMessage("Critical voltage check = %d", m_craft.Data.batteryVoltage);
      
      if (m_craft.Data.batteryVoltage < 20000)
      {
          goodVoltage = false;
      }
      
      HFLogger::logMessage("Critical voltage check result - %d", goodVoltage);
      
      return goodVoltage;
}


bool FlightManagementSystem::CriticalTetherVoltageCheck()
{
      bool goodVoltage = true;
      
      HFLogger::logMessage("Critical tether voltage check = %d", m_craft.Data.batteryVoltage);
      
      if (m_craft.Data.batteryVoltage < 9000)
      {
          goodVoltage = false;
      }
      
      HFLogger::logMessage("Critical tether voltage check result - %d", goodVoltage);
      
      return goodVoltage;}



void FlightManagementSystem::Start()
{
    // Set the communications hub to use (or not use) the controller proxy
    m_communicationsHub.m_useControllerProxy = m_useControllerProxy;
    
    // later we can change this to a parameter or set in vivo
    m_skyBox.SetReelResetPosition(0);
    
    m_holdPoint.m_noLocationLogging = m_noLocationLogging;

    // Verify min altitude in acceptable range
    if (m_minAltitude < 500)
    {
        m_minAltitude = 500;
        HFLogger::logMessage("MIN_ALTITUDE: Out of range - Raising minimum altitude to 5.0m");
    }
    if (m_minAltitude > 2000)
    {
        m_minAltitude = 2000;
        HFLogger::logMessage("MIN_ALTITUDE: Out of range - Lowering minimum altitude to 20.0m");
    }
    
    m_holdPoint.SetFloorCeiling((float)m_minAltitude / 100.0f, (float)m_maxAltitude / 100.0f);
    
    m_holdPoint.SetGpsDeniedCeiling((float) m_gpsDeniedMaxAltitude / 100.0f);
    m_holdPoint.SetGpsDeniedMode(false);
    
    m_craft.Data.SetRequirements(m_minCraftSat, m_maxCraftHdop, m_debugSkipGps);
    m_groundGPS.Data.SetRequirements(m_minGroundSat, m_maxGroundHdop, m_debugSkipGps, m_baroCompensate);

    // Start the state machine
    UpdateState(startup);
    m_runStateMachineLoop = true;
    m_stateMachineThread = std::thread(&FlightManagementSystem::StepStateMachine, this);

    // Start the display
    m_runDisplayLoop = true;
    m_displayThread = std::thread(&FlightManagementSystem::DisplayLoop, this);

	// For zoom on Ionodes Atomas Micro board with FCB-MA120 camera
	m_runContinuousZoomLoop = true;
	m_continuousZoomThread = std::thread(&FlightManagementSystem::ContinuousZoomLoop, this);

    // Separate thread to stop craft from lagging in Follow Mode during Step Zooms
    m_runStepZoomLoop = true;
    m_stepZoomThread = std::thread(&FlightManagementSystem::StepZoomLoop, this);

	// Search for cameras in separate thread to avoid blocking due to length of bootup time of Atomas Mini/Micro boards
	m_runCameraFinderLoop = true;
	m_cameraFinderStart = false;	// don't start looking right away.  wait intil craft connection
	m_cameraFinderThread = std::thread(&FlightManagementSystem::CameraFinderLoop, this);

	if (m_useLocker == true)
	{
        m_skyBox.SetLockerMode(true);
	}
    
    // Set the power loss limit max count
    powerLossMonitor.SetCountLimit(m_powerLimitCountMax);
    HFLogger::logMessage("PowerLimitCountMax: %d", m_powerLimitCountMax);
    
    // Set the power limit ceiling min
    m_holdPoint.SetPowerLimitCeilingMin(m_powerLimitFloor);
    HFLogger::logMessage("PowerLimitCeilingMin: %f", m_powerLimitFloor);
    
    
    // Set vibration info
    m_craft.Data.setVibrationFilterLength(m_vibrationFilterLength);
    HFLogger::logMessage("Vibration Filter Length: %d", m_vibrationFilterLength);


}// end Start


void FlightManagementSystem::Stop()
{
    //// ******  THREAD SAFE ***** ////
	//// ****** TO avoid deadlocks, ALWAYS stop threads in the reverse order
	//// ****** to that in which they were created.

	// Stop the camera finder thread
	m_runCameraFinderLoop = false;
	m_cameraFinderThread.join();

    // Stop the step zoom thread
    m_runStepZoomLoop = false;
    m_stepZoomThread.join();

	// Stop the continuous zoom threads
	m_runContinuousZoomLoop = false;
	m_continuousZoomThread.join();

    // Stop the display
    m_runDisplayLoop = false;
    m_displayThread.join();

    // Stop sending the controller data stream
    m_runControllerDataStreamLoop = false;
    m_controllerDataStreamThread.join();

    // Stop sending heartbeats
    m_runHeartbeatLoop = false;
    m_heartbeatThread.join();

    // Stop checking BGU report
    m_runBguMonitorLoop = false;
    m_bguMonitorThread.join();

    // Stop checking temperature
    m_runThermalLoop = false;
    m_thermalThread.join();

    // Stop the communications hub
    m_communicationsHub.Stop();

    // Stop the state machine thread
    m_runStateMachineLoop = false;
    m_stateMachineThread.join();


}// end Stop





void FlightManagementSystem::StepStateMachine()
{
    while (m_runStateMachineLoop == true)
    {
        HFLogger::logMessage("STATE: %x", m_systemState);
        switch (m_systemState)
        {
            case notSet:
                m_systemState = startup;
                break;
            case startup:
                HandleStartup();
                break;
            case connectToController:
                HandleConnectToController();
                break;
            case connectToSkyBox:
                HandleConnectToSkyBox();
                break;
            case connectToCraft:
                HandleConnectToCraft();
                break;
            case demoMode:
                HandleDemoMode();
                break;
            case connectToGroundGPS:
                HandleConnectToGroundGPS();
                break;
            case connectToGroundBaro:
                HandleConnectToGroundBaro();
                break;
            case acquiringCraftGPS:
                HandleAcquiringCraftGPS();
                break;
            case acquiringControllerGPS:
                HandleAcquiringControllerGPS();
                break;
            case readyToArm:
                HandleReadyToArm();
                break;
            case readySkyBoxClosed:
                HandleReadySkyBoxClosed();
                break;
            case closingSkyBoxFromReady:
                HandleClosingSkyBoxFromReady();
                break;
            case arming:
                HandleArming();
                break;
            case armed:
                HandleArmed();
                break;
            case launch:
                HandleLaunching();
                break;
            case flightHold:
                HandleFlight();
                break;
            case flightTranslate:
                HandleFlight();
                break;
            case flightFollow:
                HandleFlight();
                break;
            case land:
                HandleLanding();
                break;
            case failsafe:
                m_systemState = safetyStart;
                break;
            case safetyStart:
                m_systemState = lowVoltage;
                break;
            case lowVoltage:
                m_systemState = error;
                break;
            case error:
                HandleError();
                break;
            case killCraft:
                HandleKillCraft();
                break;
            default:
                m_systemState = notSet;

        }
    }
    HFLogger::logMessage("FMS : STATE WHILE LOOP EXIT");
}// end StepStateMaching


void FlightManagementSystem::UpdateState(systemState_t newState)
{
    landingFromFlightHold = false;
    if (m_systemState == flightHold)
    {
        if (newState == land)
        {
            landingFromFlightHold = true;
        }
    }
    
    m_systemState = newState;
    HFLogger::logMessage("State: %s", SystemStateToString(m_systemState));

}// end UpdateState

void FlightManagementSystem::HandleDemoMode()
{
    // Clear any commands
    m_controller.Data.ClearCommandQueue();

    bool done = false;

    while(!done)
    {
        for (int i = 0; i < 8; i++)
        {
            m_demoCraftData.rcChannels[i] = m_craft.Data.rcChannels[i];
        }

        // Look for a command
        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            m_controller.Data.ClearCommandQueue();

            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                return;
            }

            if (cmd->CommandType() == HFCMD_MOUNT_CTRL)
            {
                int deltaTilt = (int)cmd->DeltaTilt;

                m_craft.AdjustGimbalTiltAngle(deltaTilt);
            }

            if (cmd->CommandType() == HFCMD_COLOR_PALETTE)
            {
                ColorPaletteControl(cmd->PaletteCommand);
            }

            if (cmd->CommandType() == HFCMD_ZOOM)
            {
                ZoomControl(cmd->ZoomCommand);
                //ZoomControl(1, cmd->ZoomCommand);
                //ZoomControl(2, cmd->ZoomCommand);
            }
            if (cmd->CommandType() == HFCMD_ZOOMSCALE)
            {
                ZoomScaleControl(cmd->ZoomCommand);
            }


           // Look for an altitude change
            if (cmd->CommandType() == HFCMD_CHANGE_ALT)
            {
                if (m_systemState != flightTranslate)
                {
                    if (cmd->AbsoluteAltitude == false)
                    {
                        int newAltitude = m_demoCraftData.relative_alt + (int) (1000 * cmd->Z);
                        if (newAltitude < 5000)
                            newAltitude = 5000;
                        if (newAltitude > 67000)
                          newAltitude = 67000;
                        m_demoCraftData.relative_alt = newAltitude;
                    }
                }
            }



            delete cmd;
        }

        // Send joystick / zoom commands
        UpdateCameraCraftPosition(m_controller.Data.JoyX, m_controller.Data.JoyY, m_controller.Data.JoyZ, m_controller.Data.JoyButton, m_controller.Data.RoiLatitude, m_controller.Data.RoiLongitude, m_controller.Data.RoiAltitude, m_controller.Data.NewRoi);
        m_controller.Data.NewRoi = false;

        VerifyConnections(true);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}// end HandleDemoMode




void FlightManagementSystem::HandleKillCraft()
{
    HFLogger::logMessage("KILL CRAFT - KILL CRAFT - KILL CRAFT");

    bool motorsStopped = false;
    int motorsLowCount = 0;

    while (!motorsStopped)
    {
        m_craft.SendTerminateFlight();
        m_craft.SendDisarm();
        std::this_thread::sleep_for(std::chrono::milliseconds(250));


        bool motorsLow = true;
        if (m_craft.Data.servoOutputs[0] > 1200)
          motorsLow = false;
        if (m_craft.Data.servoOutputs[1] > 1200)
          motorsLow = false;
        if (m_craft.Data.servoOutputs[2] > 1200)
          motorsLow = false;
        if (m_craft.Data.servoOutputs[3] > 1200)
          motorsLow = false;

        if (motorsLow == true)
        {
            motorsLowCount++;
        }
        else
        {
            motorsLowCount = 0;
        }

        if (motorsLowCount > 20)
        {
            motorsStopped = true;
            HFLogger::logMessage("Motors low - disarming");
        }
    }



    bool craftDisarmed = false;
    int craftDisarmedCount = 0;
    int minHeartbeatCount = m_craft.Data.heartbeatCount + 5;

    while (craftDisarmed == false)
    {
        if (m_craft.Data.armed == false)
        {
            HFLogger::logMessage("Disarmed - count: %d", craftDisarmedCount);
            craftDisarmedCount++;
            if (craftDisarmedCount > 5)
            {
                if (m_craft.Data.heartbeatCount > minHeartbeatCount)
                {
                    HFLogger::logMessage("Disarmed count is good");
                    craftDisarmed = true;
                }
            }
        }
        else
        {
            craftDisarmedCount = 0;
            minHeartbeatCount = m_craft.Data.heartbeatCount + 5;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    }

    // mark flight as ended
    m_flightStarted = false;

    // Force craft into land mode
    HFLogger::logMessage("Forcing mode to land");
    while (m_craft.Data.rcChannels[RC_MODE] != 1000)
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        if ((m_precisionLanding == true) && (m_emergencyLand == false))
        {
            m_craft.SendRcOverridesLow(0, 0, 65535, 65535, 1000, 65535, 0, 65535);
        }
        else
        {
            m_craft.SendRcOverridesLow(1500, 1500, 65535, 65535, 1000, 65535, 1100, 65535);
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Wait up to 5 seconds for craft to enter land mode
    int timeout = 0;
    while ((m_craft.Data.custom_mode != flightmode_land) && (timeout < 5))
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        timeout++;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Put the craft in stabilize
    HFLogger::logMessage("Forcing mode to stabilize");
    while (m_craft.Data.rcChannels[RC_MODE] != 1420)
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        if ((m_precisionLanding == true) && (m_emergencyLand == false))
        {
            m_craft.SendRcOverridesLow(0, 0, 65535, 65535, 1420, 65535, 0, 65535);
        }
        else
        {
            m_craft.SendRcOverridesLow(1500, 1500, 65535, 65535, 1420, 65535, 1100, 65535);
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

     // Wait up to 5 seconds for craft to enter stabilize mode
    timeout = 0;
    while ((m_craft.Data.custom_mode != flightmode_stabilize) && (timeout < 5))
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        timeout++;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Put the craft in guided
    HFLogger::logMessage("Forcing mode to guided");
    while (m_craft.Data.rcChannels[RC_MODE] != 2000)
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        if ((m_precisionLanding == true) && (m_emergencyLand == false))
        {
            m_craft.SendRcOverridesLow(0, 0, 65535, 65535, 2000, 65535, 0, 65535);
        }
        else
        {
            m_craft.SendRcOverridesLow(1500, 1500, 65535, 65535, 2000, 65535, 1100, 65535);
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

     // Wait up to 5 seconds for craft to enter guided mode
    timeout = 0;
    while ((m_craft.Data.custom_mode != flightmode_guided) && (timeout < 5))
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        timeout++;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    UpdateState(readyToArm);


}// end HandleKillCraft


void FlightManagementSystem::HandleStartup()
{
    // Start the communications hub
    m_communicationsHub.Start();


    // Start the temperature monitor loop
    m_runThermalLoop = true;
    m_thermalThread = std::thread(&FlightManagementSystem::ThermalLoop, this);

    // Start the BGU monitor loop
    m_runBguMonitorLoop = true;
    m_bguMonitorThread = std::thread(&FlightManagementSystem::BguMonitorLoop, this);

    // Start the heartbeat loop
    m_runHeartbeatLoop = true;
    m_heartbeatThread = std::thread(&FlightManagementSystem::HeartbeatLoop, this);

    // Start the controller data stream thread
    m_runControllerDataStreamLoop = true;
    m_controllerDataStreamThread = std::thread(&FlightManagementSystem::ControllerDataStreamLoop, this);

//    // Start the craft update thread
//    m_runCraftUpdateThread = true;
//    m_craftUpdateThread = std::thread(&FlightManagementSystem::CraftUpdateLoop, this);


//    m_craft.Data.SetRequirements(8, 250);
//    m_groundGPS.Data.SetRequirements(8, 250);

    m_craft.Data.SetRequirements(m_minCraftSat, m_maxCraftHdop, m_debugSkipGps);
    m_groundGPS.Data.SetRequirements(m_minGroundSat, m_maxGroundHdop, m_debugSkipGps, m_baroCompensate);

    // Start looking for the controller
    UpdateState(connectToController);

}// end HandleStartup





void FlightManagementSystem::HandleConnectToController()
{
    // Look for initial controller conntection
    while (m_communicationsHub.AttemptConnectToController() == false)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        if (m_runStateMachineLoop == false)
            return;

    }

    if (m_requireBackupController == true)
    {
        while (m_communicationsHub.AttemptConnectToBackupController() == false)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));

            if (m_runStateMachineLoop == false)
                return;
        }
    }


    m_controllerConnectionState = socketConnected;

    // Look for 5 heartbeats from controller
    while (m_controller.GetHeartbeatCount() < 5)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (m_controller.m_controllerConnection->NoGroundGps == true)
    {
        m_noGroundGps = true;
    }

    m_controllerConnectionState = connected;

    m_controllerConnectionMade = true;

    // Send messages to controller detailing configuration
    char msg[50];

    // Precision landing
    strncpy(msg, "Precision Land: ", 50);
    if ((m_precisionLanding == true) && (m_emergencyLand == false))
    {
        strcat(msg, "TRUE");
    }
    else
    {
        strcat(msg, "FALSE");
    }
    SendStatusMessage(msg, 50);

    // No Ground GPS
    strncpy(msg, "No Ground GPS: ", 50);
    if (m_noGroundGps == true)
    {
        strcat(msg, "TRUE");
    }
    else
    {
        strcat(msg, "FALSE");
    }
    SendStatusMessage(msg, 50);

    // Min Craft Sat
    snprintf(msg, 50, "Min Craft Sat: %d", m_minCraftSat);
    SendStatusMessage(msg, 50);

     // Min Craft HDoP
    snprintf(msg, 50, "Max Craft HDoP: %d", m_maxCraftHdop);
    SendStatusMessage(msg, 50);

    // Min Ground Sat
    snprintf(msg, 50, "Min Ground Sat: %d", m_minGroundSat);
    SendStatusMessage(msg, 50);

    // Min Ground HDoP
    snprintf(msg, 50, "Min Ground HDoP: %d", m_maxGroundHdop);
    SendStatusMessage(msg, 50);


    // HTTP Zoom
    strncpy(msg, "HTTP Zoom: ", 50);
    if (m_httpZoom == true)
    {
        strcat(msg, "TRUE");
    }
    else
    {
        strcat(msg, "FALSE");
    }
    SendStatusMessage(msg, 50);


    // Baro compensate
    strncpy(msg, "Baro Compensate: ", 50);
    if (m_baroCompensate == true)
    {
        strcat(msg, "TRUE");
    }
    else
    {
        strcat(msg, "FALSE");
    }
    SendStatusMessage(msg, 50);

    // Debug Skip GPS
    strncpy(msg, "Debug Skip GPS: ", 50);
    if (m_debugSkipGps == true)
    {
        strcat(msg, "TRUE");
    }
    else
    {
        strcat(msg, "FALSE");
    }
    SendStatusMessage(msg, 50);


    // Yaw follows pan
    strncpy(msg, "Yaw follows pan: ", 50);
    if (m_yawFollowsPan == true)
    {
        strcat(msg, "ON");
    }
    else
    {
        strcat(msg, "OFF");
    }
    SendStatusMessage(msg, 50);

    // Max Altitude
    snprintf(msg, 50, "Max Altitude: %d", m_maxAltitude);
    SendStatusMessage(msg, 50);


    // Demo Mode
    strncpy(msg, "Demo mode: ", 50);
    if (m_demoMode == true)
    {
        strcat(msg, "ON");
    }
    else
    {
        strcat(msg, "OFF");
    }
    SendStatusMessage(msg, 50);


    // Version 3.5+
    strncpy(msg, "Version 3.5+ : ", 50);
    if (m_useV35Plus == true)
    {
        strcat(msg, "ON");
    }
    else
    {
        strcat(msg, "OFF");
    }
    SendStatusMessage(msg, 50);





    // Send a message to the controller with the version
    strncpy(msg, "FMS v", 50);
    strncat(msg, FMS_VERSION, 49);
    SendStatusMessage(msg, 50);
/*
    StatusText* statusText = new StatusText();
    statusText->severity = 0;
    strncpy(statusText->text, msg, 50);
    m_communicationsHub.SendStatusText(statusText);
*/


/*
    if ((m_useSkyBox == true) || (m_useLocker == true))
    {
        UpdateState(connectToSkyBox);
    }
    else
    {
        UpdateState(connectToCraft);
    }
*/
    UpdateState(connectToSkyBox);

}// end HandleConnectToController


void FlightManagementSystem::SendStatusMessage(char* msg, int len)
{
    StatusText* statusText = new StatusText();
    statusText->severity = 0;
    strncpy(statusText->text, msg, len);
    m_communicationsHub.SendStatusText(statusText);
}// end  SendStatusMessage

void FlightManagementSystem::HandleConnectToCraft()
{

    HFLogger::logMessage("Start of HandleConnectToCraft");

    // Look for initial craft connection
    while (m_communicationsHub.AttemptConnectToCraft() == false)
    {
        HFLogger::logMessage("In HandleConnectToCraft  loop");
        std::this_thread::sleep_for(std::chrono::seconds(1));

        if (m_runStateMachineLoop == false)
            return;

        HFLogger::logMessage("In HandleConnectToCraft - check commands");

        // Check for kill craft command
        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            m_controller.Data.ClearCommandQueue();

            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                return;
            }


            if (cmd->CommandType() == HFCMD_SKYBOX_OPEN)
            {
                m_skyBox.SendSetToFlightPosition();
            }

            if (cmd->CommandType() == HFCMD_SKYBOX_CLOSE)
            {
                m_skyBox.SendSetToStoragePosition();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_OPEN)
            {
                m_skyBox.LockerOpen();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_CLOSE)
            {
                HFLogger::logMessage("SkyBox: Locker close command execute");
                m_skyBox.LockerClose();
            }

        }

        HFLogger::logMessage("In HandleConnectToCraft verify");

        VerifyControllerConnection(true);


    }

    m_craftConnectionState = socketConnected;

    // Look for 5 heartbeats from craft
    HFLogger::logMessage("Looking for craft heartbeats");
    while (m_craft.GetHeartbeatCount() < 5)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));


        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            m_controller.Data.ClearCommandQueue();

            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                return;
            }


            if (cmd->CommandType() == HFCMD_SKYBOX_OPEN)
            {
                m_skyBox.SendSetToFlightPosition();
            }

            if (cmd->CommandType() == HFCMD_SKYBOX_CLOSE)
            {
                m_skyBox.SendSetToStoragePosition();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_OPEN)
            {
                m_skyBox.LockerOpen();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_CLOSE)
            {
                HFLogger::logMessage("SkyBox: Locker close command execute");
                m_skyBox.LockerClose();
            }

        }

    }

    m_craftConnectionState = requestingDataStream;

    // Request data stream from craft
    HFLogger::logMessage("Requesting data stream from craft");
    while (m_craft.DataStreamValid() == false)
    {
        // Send a stream request
        m_craft.RequestDataStream();

        std::this_thread::sleep_for(std::chrono::seconds(1));


        // Check for kill craft command
        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            m_controller.Data.ClearCommandQueue();

            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                return;
            }


            if (cmd->CommandType() == HFCMD_SKYBOX_OPEN)
            {
                m_skyBox.SendSetToFlightPosition();
            }

            if (cmd->CommandType() == HFCMD_SKYBOX_CLOSE)
            {
                m_skyBox.SendSetToStoragePosition();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_OPEN)
            {
                m_skyBox.LockerOpen();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_CLOSE)
            {
                m_skyBox.LockerClose();
            }

        }


    }

    m_craftConnectionState = waitingForValidData;

    // Wait to make sure we have good data
    HFLogger::logMessage("Waiting for valid data");
    for (int i = 0; i < 5; i++)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));


        // Check for kill craft command
        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            m_controller.Data.ClearCommandQueue();

            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                return;
            }


            if (cmd->CommandType() == HFCMD_SKYBOX_OPEN)
            {
                m_skyBox.SendSetToFlightPosition();
            }

            if (cmd->CommandType() == HFCMD_SKYBOX_CLOSE)
            {
                m_skyBox.SendSetToStoragePosition();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_OPEN)
            {
                m_skyBox.LockerOpen();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_CLOSE)
            {
                m_skyBox.LockerClose();
            }

        }

    }

    // Check the servos to see if we are in flight
    HFLogger::logMessage("Verifying craft is not in flight");
    bool inFlight = false;
    for (int i = 0; i < 4; i++)
    {
        if (m_craft.Data.servoOutputs[i] > 1300)
            inFlight = true;
    }
    
    // Check to see if craft is armed
    if (m_craft.Data.armed == true)
    {
        inFlight = true;
    }

    if (inFlight == true)
    {
        UpdateState(error);
        return;
    }




    // Release the overrides
    uint16_t rcChannelsGimbalDetect[8] = {0, 0, 1001, 0, 0, 0, 0, 0};

    // Verify the updates
    HFLogger::logMessage("GIMBAL_AUTODETECT: Releasing RC Channels");
    while (m_craft.SendAndVerifyRcOverridesLow(rcChannelsGimbalDetect, 500) == false)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

    }




    char gimbalDetectMsg[50];
    
    if (m_yawFollowsPan == true)
    {
        if (m_craft.Data.gimbalHeartbeatCount == 0)
        {
            HFLogger::logMessage("GIMBAL: No heartbeats detected - disabling yaw follow pan");
            m_yawFollowsPan = false;
            strncpy(gimbalDetectMsg, "Gimbal autodetect fail - Setting no gimbal", 50);
            StatusText* statusText = new StatusText();
            statusText->severity = 0;
            strncpy(statusText->text, gimbalDetectMsg, 50);
            m_communicationsHub.SendStatusText(statusText);
        }
        else
        {
            HFLogger::logMessage("GIMBAL: Confirming heartbeats detected");
            m_yawFollowsPan = true;
        }        
    }
    else
    {
        HFLogger::logMessage("GIMBAL_AUTODETECT: Yaw Follows Pawn is disabled - skipping autodetect");
        strncpy(gimbalDetectMsg, "Yaw follows pan disabled - no autodetect", 50);
        StatusText* statusText = new StatusText();
        statusText->severity = 0;
        strncpy(statusText->text, gimbalDetectMsg, 50);
        m_communicationsHub.SendStatusText(statusText);    
    }
    


/*
    if (m_yawFollowsPan == true)
    {
        // Attempting auto-detect of gimbal system
        HFLogger::logMessage("GIMBAL_AUTODETECT: start");
        bool gimbalDetectionComplete = false;

        int gimbalDetectCount = 0;
        while ((gimbalDetectionComplete == false) && (gimbalDetectCount < 10))
        {
            HFLogger::logMessage("GIMBAL_AUTODETECT: attempt %d", gimbalDetectCount + 1);
            switch (detectGimbal())
            {
                case GIMBAL_FOUND:
                    m_yawFollowsPan = true;
                    gimbalDetectionComplete = true;
                    HFLogger::logMessage("GIMBAL_AUTODETECT: gimbal found");
                break;

                case GIMBAL_NOT_FOUND:
                    m_yawFollowsPan = false;
                    gimbalDetectionComplete = true;
                    HFLogger::logMessage("GIMBAL_AUTODETECT: gimbal not found");
                break;

                default:
                    HFLogger::logMessage("GIMBAL_AUTODETECT: not set");
                break;
            }

            gimbalDetectCount++;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }

        if (gimbalDetectionComplete == false)
        {
            HFLogger::logMessage("GIMBAL_AUTODETECT: Autodetect failed, setting to no gimbal present");
            m_yawFollowsPan = false;
            strncpy(gimbalDetectMsg, "Gimbal autodetect fail - Setting no gimbal", 50);
            StatusText* statusText = new StatusText();
            statusText->severity = 0;
            strncpy(statusText->text, gimbalDetectMsg, 50);
            m_communicationsHub.SendStatusText(statusText);
        }
        else
        {
            if (m_yawFollowsPan == true)
            {
                strncpy(gimbalDetectMsg, "Gimbal detected - yaw follows pan set", 50);
            }
            else
            {
                strncpy(gimbalDetectMsg, "No gimbal detected - yaw follows pan off", 50);
            }
            StatusText* statusText = new StatusText();
            statusText->severity = 0;
            strncpy(statusText->text, gimbalDetectMsg, 50);
            m_communicationsHub.SendStatusText(statusText);
        }
    }
    else
    {
        HFLogger::logMessage("GIMBAL_AUTODETECT: Yaw Follows Pawn is disabled - skipping autodetect");
        strncpy(gimbalDetectMsg, "Yaw follows pan disabled - no autodetect", 50);
        StatusText* statusText = new StatusText();
        statusText->severity = 0;
        strncpy(statusText->text, gimbalDetectMsg, 50);
        m_communicationsHub.SendStatusText(statusText);
    }
    */

    m_craftConnectionState = initializingChannels;

    // Set the RC overrides
    uint16_t rcChannels[8] = {1500, 1500, 1000, 1500, 2000, 1500, 1100, 2000};

    if ((m_precisionLanding == true) && (m_emergencyLand == false))
    {
        rcChannels[RC_PITCH] = 0;
        rcChannels[RC_ROLL] = 0;
        rcChannels[RC_ZOOM] = 0;
    }

    // Verify the updates
    HFLogger::logMessage("Overriding RC channels");
    while (m_craft.SendAndVerifyRcOverridesLow(rcChannels, 500) == false)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

    }

    // Force craft into land mode
    HFLogger::logMessage("Forcing mode to land");
    while (m_craft.Data.rcChannels[RC_MODE] != 1000)
    {
        if ((m_precisionLanding == true) && (m_emergencyLand == false))
        {
            m_craft.SendRcOverridesLow(0, 0, 65535, 65535, 1000, 65535, 0, 65535);
        }
        else
        {
            m_craft.SendRcOverridesLow(65535, 65535, 65535, 65535, 1000, 65535, 1100, 65535);
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }
    }

    // Wait up to 5 seconds for craft to enter land mode
    int timeout = 0;
    while ((m_craft.Data.custom_mode != flightmode_land) && (timeout < 5))
    {
        timeout++;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }
    }

    // Put the craft in guided
    HFLogger::logMessage("Forcing mode to guided");
    while (m_craft.Data.rcChannels[RC_MODE] != 2000)
    {
        if ((m_precisionLanding == true) && (m_emergencyLand == false))
        {
            m_craft.SendRcOverridesLow(0, 0, 65535, 65535, 2000, 65535, 0, 65535);
        }
        else
        {
            m_craft.SendRcOverridesLow(65535, 65535, 65535, 65535, 2000, 65535, 1100, 65535);
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }
    }

    // Send the camera to the 0 position
    m_craft.SetGimbalTiltAngle(90000);

    m_craftConnectionState = connected;

    m_craftConnectionMade = true;

    m_craft.SendAutopilotVersionRequest();
    m_craft.SendVersionBannerRequest();

	// Start looking for cameras
	m_cameraFinderStart = true;


    // Set the yaw reset altitude
    SetYawResetAltitude();


	// Check for kill craft command
	if (CheckForKillCommand() == true)
	{
		UpdateState(killCraft);
		return;
	}

    if (m_demoMode == true)
    {
        HFLogger::logMessage("Entering demo mode");
            // Mark flight as started and grab time

    }
    else
    {
        HFLogger::logMessage("Not entering demo mode");
    }

    if (m_demoMode == true)
    {
        InitializeDemoMode();
        UpdateState(demoMode);
    }
    else
    {
        if (m_noGroundGps == true)
        {
            if (m_baroCompensate == false)
            {
                UpdateState(acquiringCraftGPS);
            }
            else
            {
                UpdateState(connectToGroundBaro);
            }
        }
        else
        {
            UpdateState(connectToGroundGPS);
        }
    }
}// end HandleConnectToCraft





bool FlightManagementSystem::SetYawResetAltitude()
{
    // Calculate and limit the realignment altitude
    float minAlt = (float) m_minAltitude / 100.0f;
    m_yawRealignmentAltitude = minAlt - 1.0f;
    if (m_yawRealignmentAltitude < 4.5f)
    {
        m_yawRealignmentAltitude = 4.5f;
    }
    if (m_yawRealignmentAltitude > 9.0f)
    {
        m_yawRealignmentAltitude = 9.0f;
    }
    
    HFLogger::logMessage("YAW_RESET: Yaw realignment set to %f", m_yawRealignmentAltitude);
    
    
    // First read existing values
    char paramId[16];
    bool readYawAltReset = false;
    bool yawAltResetFail = false;
    int yawAltResetCounter = 0;
    while ((readYawAltReset == false) && (yawAltResetFail == false))
    {
        // Send message to read current values
        m_craft.Data.ek2_mag_rst_alt_set = false;
        m_craft.Data.ek3_mag_rst_alt_set = false;        
        strncpy(paramId, "EK2_MAG_RST_ALT", 16);
        m_craft.SendParamRequestRead(paramId);
        strncpy(paramId, "EK3_MAG_RST_ALT", 16);
        m_craft.SendParamRequestRead(paramId);

        // Delay for response
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Check for response
        if ((m_craft.Data.ek2_mag_rst_alt_set == true) || (m_craft.Data.ek3_mag_rst_alt_set == true))
        {
            HFLogger::logMessage("YAW_RESET: Current values  EK2 = %f   EK3 = %f", m_craft.Data.ek2_mag_rst_alt, m_craft.Data.ek3_mag_rst_alt);
            readYawAltReset = true;
        }
        else
        {
            yawAltResetCounter++;
            if (yawAltResetCounter > 10) 
            {
                yawAltResetFail = true;
            }
        }
        
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return false;
        }
        
    }

    // Check for failure to read values
    if (yawAltResetFail == true) 
    {
        HFLogger::logMessage("YAW_RESET: Failed to read values.  Possibly older version of FW");
        return false;
    }

    
    // Set the desired values
    yawAltResetFail = false;
    readYawAltReset = false;
    yawAltResetCounter = 0;
    
    while ((readYawAltReset == false) && (yawAltResetFail == false))
    {
        // Update the values
        HFLogger::logMessage("YAW_RESET: Setting altitude to %f", m_yawRealignmentAltitude);
        m_craft.UpdateYawResetAltitude(m_yawRealignmentAltitude);
        
        // Request the values
        m_craft.Data.ek2_mag_rst_alt_set = false;
        m_craft.Data.ek3_mag_rst_alt_set = false;        
        strncpy(paramId, "EK2_MAG_RST_ALT", 16);
        m_craft.SendParamRequestRead(paramId);
        strncpy(paramId, "EK3_MAG_RST_ALT", 16);
        m_craft.SendParamRequestRead(paramId);

        
        // Delay for response
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        
        // Verify the values
        if ((m_craft.Data.ek2_mag_rst_alt == m_yawRealignmentAltitude) && (m_craft.Data.ek3_mag_rst_alt == m_yawRealignmentAltitude))
        {
            readYawAltReset = true;
            HFLogger::logMessage("YAW_RESET: Read values of EK2: %f   EK3: %f", m_craft.Data.ek2_mag_rst_alt, m_craft.Data.ek3_mag_rst_alt);
        }
        else
        {
            yawAltResetCounter++;
            if (yawAltResetCounter > 10) 
            {
                yawAltResetFail = true;
            }
        }        
        
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return false;
        }
    
    }
    
    if (yawAltResetFail == false)
    {
        HFLogger::logMessage("YAW_RESET: Complete");
        return true;
    }
    else    
    {
        HFLogger::logMessage("YAW_RESET: Failed");
        return false;
    }
    
}







void FlightManagementSystem::InitializeDemoMode()
{
    m_flightStarted = true;
    time(&m_flightStartTime);
    /* ADDED LOGIC TO READ LATITUDES & LONGITUDES FROM coordinates.TXT FILE */

    FILE *coordFile;
    char instruction[5];
    double error = 30.0;
    if((coordFile = fopen("coordinates.txt", "r"))) {
        int i;
        for(i = 0; i < 3; i++) {
            fgets(instruction, 4, coordFile);
            if(strcmp(instruction, "lat") == 0) {
                fscanf(coordFile, "%d", &m_demoCraftData.latitude);
                error /= 3.0;
            } else if(strcmp(instruction, "lon") == 0) {
                fscanf(coordFile, "%d", &m_demoCraftData.longitude);
                error /= 2.0;
            } else if(strcmp(instruction, "alt") == 0) {
                fscanf(coordFile, "%d", &m_demoCraftData.relative_alt);
                error /= 5.0;
            }
            fgets(instruction, 2, coordFile); // newline
        }
        if(error != 1.0) {
            m_demoCraftData.latitude  =   361215220;
            m_demoCraftData.longitude = -1151663060;
            m_demoCraftData.relative_alt = 5000;

        }
        fclose(coordFile);
    } else {
        m_demoCraftData.latitude  =   361215220;
        m_demoCraftData.longitude = -1151663060;
        m_demoCraftData.relative_alt = 5000;
    }

    m_demoCraftData.heading = m_craft.Data.heading;

    m_demoCraftData.gpsHdop = 175;
    m_demoCraftData.gpsVdop = 187;
    m_demoCraftData.gpsNumberSatellites = 14;
    m_demoCraftData.gpsFixType = 3;

}// end InitializeDemoMode


void FlightManagementSystem::HandleConnectToGroundGPS()
{
    // Look for initial ground GPS connection
    while (m_communicationsHub.AttemptConnectToGroundGps() == false)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        if (m_runStateMachineLoop == false)
            return;

        // Check for kill craft command
        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            m_controller.Data.ClearCommandQueue();

            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                return;
            }


            if (cmd->CommandType() == HFCMD_SKYBOX_OPEN)
            {
                m_skyBox.SendSetToFlightPosition();
            }

            if (cmd->CommandType() == HFCMD_SKYBOX_CLOSE)
            {
                m_skyBox.SendSetToStoragePosition();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_OPEN)
            {
                m_skyBox.LockerOpen();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_CLOSE)
            {
                m_skyBox.LockerClose();
            }

        }
    }

    m_groundGpsConnectionState = socketConnected;


    // Look for 5 heartbeats from ground GPS
    while (m_groundGPS.GetHeartbeatCount() < 5)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Check for kill craft command
        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            m_controller.Data.ClearCommandQueue();

            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                return;
            }


            if (cmd->CommandType() == HFCMD_SKYBOX_OPEN)
            {
                m_skyBox.SendSetToFlightPosition();
            }

            if (cmd->CommandType() == HFCMD_SKYBOX_CLOSE)
            {
                m_skyBox.SendSetToStoragePosition();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_OPEN)
            {
                m_skyBox.LockerOpen();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_CLOSE)
            {
                m_skyBox.LockerClose();
            }

        }
    }

    m_groundGpsConnectionState = waitingForValidData;

    bool groundGpsValid = false;
    while (groundGpsValid == false)
    {
        m_groundGPS.RequestDataStream();
        std::this_thread::sleep_for(std::chrono::seconds(1));

        if (m_baroCompensate == true)
        {
            groundGpsValid = m_groundGPS.DataStreamValid();// && m_groundGPS.PressureReceived();
        }
        else
        {
            groundGpsValid = m_groundGPS.DataStreamValid();
        }

        // Check for kill craft command
        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            m_controller.Data.ClearCommandQueue();

            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                return;
            }


            if (cmd->CommandType() == HFCMD_SKYBOX_OPEN)
            {
                m_skyBox.SendSetToFlightPosition();
            }

            if (cmd->CommandType() == HFCMD_SKYBOX_CLOSE)
            {
                m_skyBox.SendSetToStoragePosition();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_OPEN)
            {
                m_skyBox.LockerOpen();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_CLOSE)
            {
                m_skyBox.LockerClose();
            }

        }
    }

    // Wait to make sure we have good data
    for (int i = 0; i < 5; i++)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }
    }

    m_groundGpsConnectionState = connected;

    m_groundGpsConnectionMade = true;

    if (m_baroCompensate == false)
    {
        UpdateState(acquiringCraftGPS);
    }
    else
    {
        UpdateState(connectToGroundBaro);
    }

}// end HandleConnectToGroundGPS

void FlightManagementSystem::HandleConnectToGroundBaro()
{
   // Look for initial ground baro connection
    while (m_communicationsHub.AttemptConnectToGroundBaro() == false)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        if (m_runStateMachineLoop == false)
            return;

        // Check for kill craft command
        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            m_controller.Data.ClearCommandQueue();

            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                return;
            }


            if (cmd->CommandType() == HFCMD_SKYBOX_OPEN)
            {
                m_skyBox.SendSetToFlightPosition();
            }

            if (cmd->CommandType() == HFCMD_SKYBOX_CLOSE)
            {
                m_skyBox.SendSetToStoragePosition();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_OPEN)
            {
                m_skyBox.LockerOpen();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_CLOSE)
            {
                m_skyBox.LockerClose();
            }

        }
    }

    HFLogger::logMessage("ground baro socketConnected");
    m_groundBaroConnectionState = socketConnected;


    // Look for 5 heartbeats from ground GPS
    while (m_groundBaro.GetHeartbeatCount() < 5)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Check for kill craft command
        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            m_controller.Data.ClearCommandQueue();

            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                return;
            }


            if (cmd->CommandType() == HFCMD_SKYBOX_OPEN)
            {
                m_skyBox.SendSetToFlightPosition();
            }

            if (cmd->CommandType() == HFCMD_SKYBOX_CLOSE)
            {
                m_skyBox.SendSetToStoragePosition();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_OPEN)
            {
                m_skyBox.LockerOpen();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_CLOSE)
            {
                m_skyBox.LockerClose();
            }
        }
    }

    HFLogger::logMessage("ground baro waitingForValidData");
    m_groundBaroConnectionState = waitingForValidData;

    bool groundBaroValid = false;
    while (groundBaroValid == false)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        groundBaroValid = m_groundBaro.DataStreamValid();

        // Check for kill craft command
        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            m_controller.Data.ClearCommandQueue();

            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                return;
            }


            if (cmd->CommandType() == HFCMD_SKYBOX_OPEN)
            {
                m_skyBox.SendSetToFlightPosition();
            }

            if (cmd->CommandType() == HFCMD_SKYBOX_CLOSE)
            {
                m_skyBox.SendSetToStoragePosition();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_OPEN)
            {
                m_skyBox.LockerOpen();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_CLOSE)
            {
                m_skyBox.LockerClose();
            }
        }
    }

    // Wait to make sure we have good data
    for (int i = 0; i < 5; i++)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Check for kill craft command
        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            m_controller.Data.ClearCommandQueue();

            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                return;
            }


            if (cmd->CommandType() == HFCMD_SKYBOX_OPEN)
            {
                m_skyBox.SendSetToFlightPosition();
            }

            if (cmd->CommandType() == HFCMD_SKYBOX_CLOSE)
            {
                m_skyBox.SendSetToStoragePosition();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_OPEN)
            {
                m_skyBox.LockerOpen();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_CLOSE)
            {
                m_skyBox.LockerClose();
            }
        }
    }

    HFLogger::logMessage("ground baro connected");
    m_groundBaroConnectionState = connected;

    m_groundBaroConnectionMade = true;

//    UpdateState(connectToReel);
    UpdateState(acquiringCraftGPS);

}// end HandleConnectToGroundBaro


void FlightManagementSystem::HandleConnectToSkyBox()
{
   // Look for initial ground baro connection
    while (m_communicationsHub.AttemptConnectToSkyBox() == false)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        if (m_runStateMachineLoop == false)
            return;

        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }
    }

    HFLogger::logMessage("SkyBox socketConnected");
    m_skyBoxConnectionState = socketConnected;


    // Look for 5 heartbeats from ground GPS
    while (m_skyBox.GetHeartbeatCount() < 5)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }
    }

    HFLogger::logMessage("SkyBox waitingForValidData");


    // Wait to make sure we have good data
    for (int i = 0; i < 5; i++)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }
    }

    HFLogger::logMessage("SkyBox connected");
    m_skyBoxConnectionState = connected;

    m_skyBoxConnectionMade = true;


    UpdateState(connectToCraft);

}





void FlightManagementSystem::HandleAcquiringCraftGPS()
{
    bool done = false;
    bool goodGps = false;
    int goodGpsCount = 0;
    countHandleAquireCraftGPS = 0;

     // Read the GPS_AUTO_SWITCH parameter
    while (m_craft.Data.gps_auto_switch_set == false)
    {
        HFLogger::logMessage("Attempting to read GPS_AUTO_SWITCH");
        
        // Send message to read current value
        char paramId[16];
        strncpy(paramId, "GPS_AUTO_SWITCH", 16);
        m_craft.SendParamRequestRead(paramId);
        
        // Delay for response
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }
    }
    
    HFLogger::logMessage("HandleAcquireCraftGPS - GPS_AUTO_SWITCH: %d", m_craft.Data.gps_auto_switch);
    
    while (!done)
    {
        HFLogger::logMessage("HandleAquireCraftGPS count: %d", countHandleAquireCraftGPS);
        countHandleAquireCraftGPS++;
        
        if (m_craft.Data.IsGpsGood() == true)
        {
            if (goodGps == false)
            {
                goodGps = true;
            }
            else
            {
                if (goodGpsCount > 50)
                {
                    done = true;
                }
                goodGpsCount++;
            }
        }
        else
        {
            goodGps = false;
            goodGpsCount = 0;
        }

        if (m_runStateMachineLoop == false)
        {
          done = true;
          return;
        }

        VerifyConnections(true);

        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            m_controller.Data.ClearCommandQueue();

            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                return;
            }


            if (cmd->CommandType() == HFCMD_SKYBOX_OPEN)
            {
                m_skyBox.SendSetToFlightPosition();
            }

            if (cmd->CommandType() == HFCMD_SKYBOX_CLOSE)
            {
                m_skyBox.SendSetToStoragePosition();
            }


            if (cmd->CommandType() == HFCMD_LOCKER_OPEN)
            {
                m_skyBox.LockerOpen();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_CLOSE)
            {
                m_skyBox.LockerClose();
            }




            if (cmd->CommandType() == HFCMD_MOUNT_CTRL)
            {
                int deltaTilt = (int)cmd->DeltaTilt;

                m_craft.AdjustGimbalTiltAngle(deltaTilt);
            }

            if (cmd->CommandType() == HFCMD_REMOTE_CTRL)
            {
                m_useRemoteJoystick = cmd->Enable;
            }

            if (cmd->CommandType() == HFCMD_COLOR_PALETTE)
            {
                HFLogger::logMessage("====> Palette in acquire craft gps");
                ColorPaletteControl(cmd->PaletteCommand);
            }

            if (cmd->CommandType() == HFCMD_ZOOM)
            {
                ZoomControl(cmd->ZoomCommand);
                //ZoomControl(1, cmd->ZoomCommand);
                //ZoomControl(2, cmd->ZoomCommand);
            }
            if (cmd->CommandType() == HFCMD_ZOOMSCALE)
            {
                ZoomScaleControl(cmd->ZoomCommand);
            }

            delete cmd;
        }


        UpdateCameraCraftPosition(m_controller.Data.JoyX, m_controller.Data.JoyY, m_controller.Data.JoyZ, m_controller.Data.JoyButton);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));


    }

    if (m_noGroundGps == true)
    {
        UpdateState(readyToArm);
    }
    else
    {
        UpdateState(acquiringControllerGPS);
    }
}// end AcquireCraftGPS


void FlightManagementSystem::HandleAcquiringControllerGPS()
{
    bool done = false;
    bool goodGps = false;
    int goodGpsCount = 0;
    countHandleAquireControllerGPS = 0;

    while (!done)
    {
        HFLogger::logMessage("HandleAcquiringControllerGPS count: %d", countHandleAquireControllerGPS);
        countHandleAquireControllerGPS++;
        
        if (m_groundGPS.Data.IsGpsGood() == true)
        {
            if (goodGps == false)
            {
                goodGps = true;
            }
            else
            {
                if (goodGpsCount > 50)
                {
                    done = true;
                }
                goodGpsCount++;
            }
        }
        else
        {
            goodGps = false;
            goodGpsCount = 0;
        }

        if (m_runStateMachineLoop == false)
        {
          done = true;
          return;
        }

        VerifyConnections(true);


        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            m_controller.Data.ClearCommandQueue();

            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                return;
            }


            if (cmd->CommandType() == HFCMD_SKYBOX_OPEN)
            {
                m_skyBox.SendSetToFlightPosition();
            }

            if (cmd->CommandType() == HFCMD_SKYBOX_CLOSE)
            {
                m_skyBox.SendSetToStoragePosition();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_OPEN)
            {
                m_skyBox.LockerOpen();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_CLOSE)
            {
                m_skyBox.LockerClose();
            }



            if (cmd->CommandType() == HFCMD_MOUNT_CTRL)
            {
                int deltaTilt = (int)cmd->DeltaTilt;

                m_craft.AdjustGimbalTiltAngle(deltaTilt);
            }

            if (cmd->CommandType() == HFCMD_REMOTE_CTRL)
            {
                m_useRemoteJoystick = cmd->Enable;
            }

            if (cmd->CommandType() == HFCMD_COLOR_PALETTE)
            {
                ColorPaletteControl(cmd->PaletteCommand);
            }

            if (cmd->CommandType() == HFCMD_ZOOM)
            {
                ZoomControl(cmd->ZoomCommand);
                //ZoomControl(1, cmd->ZoomCommand);
                //ZoomControl(2, cmd->ZoomCommand);
            }
            if (cmd->CommandType() == HFCMD_ZOOMSCALE)
            {
                ZoomScaleControl(cmd->ZoomCommand);
            }

            delete cmd;
        }

        UpdateCameraCraftPosition(m_controller.Data.JoyX, m_controller.Data.JoyY, m_controller.Data.JoyZ, m_controller.Data.JoyButton);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));


    }

    UpdateState(readyToArm);
}// end HandleAcquireControllerGPS


bool FlightManagementSystem::IsAllGpsGood()
{
    if (m_noGroundGps == true)
    {
        // Verify we have a recent connection
        if (m_craft.IsConnectionActive(5) == true)
        {
            return m_craft.Data.IsGpsGood();
        }
        else
        {
            HFLogger::logMessage("IsAllGpsGood:  No connection to craft");
            return false;
        }
    }
    else
    {
        // Verify we have a recent connection
        if ((m_craft.IsConnectionActive(5) == true) && (m_groundGPS.IsConnectionActive(5) == true))
        {
            return (m_craft.Data.IsGpsGood() && m_groundGPS.Data.IsGpsGood());
        }
        else 
        {
            HFLogger::logMessage("IsAllGpsGood:  No connection to craft or ground GPS");
            return false;
        }
    }
}// end IsAllGpsGood




void FlightManagementSystem::HandleReadySkyBoxClosed()
{
    // Clear the command queue to remove bad commands
    m_controller.Data.ClearCommandQueue();

    

    bool done = false;
    while (!done)
    {


        // Verify GPS is still good
        if (IsAllGpsGood() == false)
        {
            UpdateState(acquiringCraftGPS);
            return;
        }

        // Look for an open skybox command
        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                delete cmd;
                return;
            }



            m_controller.Data.ClearCommandQueue();

            if (cmd->CommandType() == HFCMD_SKYBOX_OPEN)
            {
                m_skyBox.SendSetToFlightPosition();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_OPEN)
            {
                m_skyBox.LockerOpen();
            }


            if (cmd->CommandType() == HFCMD_MOUNT_CTRL)
            {
                int deltaTilt = (int)cmd->DeltaTilt;

                m_craft.AdjustGimbalTiltAngle(deltaTilt);
            }

            if (cmd->CommandType() == HFCMD_REMOTE_CTRL)
            {
                m_useRemoteJoystick = cmd->Enable;
            }

            if (cmd->CommandType() == HFCMD_COLOR_PALETTE)
            {
                ColorPaletteControl(cmd->PaletteCommand);
            }

            if (cmd->CommandType() == HFCMD_ZOOM)
            {
                ZoomControl(cmd->ZoomCommand);
                //ZoomControl(1, cmd->ZoomCommand);
                //ZoomControl(2, cmd->ZoomCommand);
            }
            if (cmd->CommandType() == HFCMD_ZOOMSCALE)
            {
                ZoomScaleControl(cmd->ZoomCommand);
            }


            delete cmd;
        }

        // Check to see if SkyBox is open
        if (m_skyBox.IsInFlightPosition() == true) {
            UpdateState(readyToArm);
            done = true;
            return;
        }

        if (m_runStateMachineLoop == false)
        {
          done = true;
          return;
        }

        VerifyConnections(true);

        UpdateCameraCraftPosition(m_controller.Data.JoyX, m_controller.Data.JoyY, m_controller.Data.JoyZ, m_controller.Data.JoyButton);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }
}



void FlightManagementSystem::HandleClosingSkyBoxFromReady()
{
    // Clear the command queue to remove bad commands
    m_controller.Data.ClearCommandQueue();


    bool done = false;
    while (!done)
    {
        // Look for an open skybox command
        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                delete cmd;
                return;
            }


            if (cmd->CommandType() == HFCMD_SKYBOX_CLOSE)
            {
                m_skyBox.SendSetToStoragePosition();
            }

            if (cmd->CommandType() == HFCMD_LOCKER_CLOSE)
            {
                m_skyBox.LockerClose();
            }


            m_controller.Data.ClearCommandQueue();

            if (cmd->CommandType() == HFCMD_MOUNT_CTRL)
            {
                int deltaTilt = (int)cmd->DeltaTilt;

                m_craft.AdjustGimbalTiltAngle(deltaTilt);
            }

            if (cmd->CommandType() == HFCMD_REMOTE_CTRL)
            {
                m_useRemoteJoystick = cmd->Enable;
            }

            if (cmd->CommandType() == HFCMD_COLOR_PALETTE)
            {
                ColorPaletteControl(cmd->PaletteCommand);
            }

            if (cmd->CommandType() == HFCMD_ZOOM)
            {
                ZoomControl(cmd->ZoomCommand);
                //ZoomControl(1, cmd->ZoomCommand);
                //ZoomControl(2, cmd->ZoomCommand);
            }
            if (cmd->CommandType() == HFCMD_ZOOMSCALE)
            {
                ZoomScaleControl(cmd->ZoomCommand);
            }


            delete cmd;
        }

        // Check to see if SkyBox is closed
        if (m_skyBox.IsInStoragePosition() == true) {
            UpdateState(readySkyBoxClosed);
            done = true;
            return;
        }

        if (m_runStateMachineLoop == false)
        {
          done = true;
          return;
        }

        VerifyConnections(true);

        UpdateCameraCraftPosition(m_controller.Data.JoyX, m_controller.Data.JoyY, m_controller.Data.JoyZ, m_controller.Data.JoyButton);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }
}




bool FlightManagementSystem::IsBguRcGood()
{
    bool rcGood = true;

    // Check roll
    if (m_craft.Data.rcChannels[0] < 1200)
    {
        rcGood = false;
    }

    // Check pitch
    if (m_craft.Data.rcChannels[1] < 1200)
    {
        rcGood = false;
    }

    // Check yaw
    if (m_craft.Data.rcChannels[3] < 1200)
    {
        rcGood = false;
    }


    return rcGood;
}

void FlightManagementSystem::HandleReadyToArm()
{
    countHandleReadyToArm = 0;
    
    landingAutoAborted = false;
    craftInitiatedLanding  = false;
    
    // Clear the command queue to remove bad commands
    m_controller.Data.ClearCommandQueue();


    // Capture the ground pressures if we are using baro compensate
    if (m_baroCompensate == true)
    {
        HFLogger::logMessage("Resetting baro compensation");

        m_groundBaroDataValid = false;
        // Update the starting pressures
        m_craftStartPressure = m_craft.Data.pressAbs;
        m_craftStartPressure2 = m_craft.Data.pressAbs2;
        m_craftStartPressure3 = m_craft.Data.pressAbs3;
        m_groundStartPressure = m_groundBaro.Data.pressAbs;

        m_lastDeltaPressure = 0.0f;
        m_groundBaroDataValid = true;

        HFLogger::logMessage("C1Base: %f  C2Base: %f  C3Base: %f  GndBase: %f", m_craftStartPressure, m_craftStartPressure2, m_craftStartPressure3, m_groundStartPressure);
    }




    // Set and verify the launch climb rate
    m_craft.Data.wpnav_speed_up_set = false;
    while ((m_craft.Data.wpnav_speed_up != m_launchClimbRate) || (m_craft.Data.wpnav_speed_up_set == false))
    {
        // Set the speed
        m_craft.UpdateWpNavSpeedUp(m_launchClimbRate);            
        HFLogger::logMessage("ReadytoArm - Setting WPNAV_SPEED_UP to %d", m_launchClimbRate);

        // Delay for response
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }           
       
    }
    HFLogger::logMessage("ReadyToArm - WPNAV_SPEED_UP: %d", m_craft.Data.wpnav_speed_up);
    
        


    bool done = false;
    while (!done)
    {
        HFLogger::logMessage("HandleReadyToArm count: %d", countHandleReadyToArm);
        countHandleReadyToArm++;

        // Verify SkyBox is in position
        if ((m_useSkyBox == true) || (m_useLocker == true))
        {
            if (m_skyBox.IsInFlightPosition() == false)
            {
                UpdateState(readySkyBoxClosed);
                return;
            }
        }

        


        // Verify BGU RC Signals are good
        if (IsBguRcGood() == false)
        {
            UpdateState(error);
            return;
        }

        // Verify GPS is still good
        if (IsAllGpsGood() == false)
        {
            UpdateState(acquiringCraftGPS);
            return;
        }
        
        
        // Send GPS Denied state to Hold Point
        if (m_gpsDeniedSupport == true)
        {
            m_holdPoint.SetGpsDeniedMode(IsGpsOn());
        }

        // Look for an arm command
        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                delete cmd;
                return;
            }


            if (cmd->CommandType() == HFCMD_SKYBOX_CLOSE)
            {
                m_skyBox.SendSetToStoragePosition();
                UpdateState(closingSkyBoxFromReady);
                delete cmd;
                return;
            }


            if (cmd->CommandType() == HFCMD_LOCKER_CLOSE)
            {
                m_skyBox.LockerClose();
                UpdateState(closingSkyBoxFromReady);
                delete cmd;
                return;
            }

            m_controller.Data.ClearCommandQueue();
            if (cmd->CommandType() == HFCMD_ARM)
            {
                HFLogger::logMessage("Arm Command");
                UpdateState(arming);
                done = true;
                delete cmd;
                return;
            }

            if (cmd->CommandType() == HFCMD_MOUNT_CTRL)
            {
                int deltaTilt = (int)cmd->DeltaTilt;

                m_craft.AdjustGimbalTiltAngle(deltaTilt);
            }

            if (cmd->CommandType() == HFCMD_REMOTE_CTRL)
            {
                m_useRemoteJoystick = cmd->Enable;
            }

            if (cmd->CommandType() == HFCMD_COLOR_PALETTE)
            {
                ColorPaletteControl(cmd->PaletteCommand);
            }

            if (cmd->CommandType() == HFCMD_ZOOM)
            {
                ZoomControl(cmd->ZoomCommand);
                //ZoomControl(1, cmd->ZoomCommand);
                //ZoomControl(2, cmd->ZoomCommand);
            }
            if (cmd->CommandType() == HFCMD_ZOOMSCALE)
            {
                ZoomScaleControl(cmd->ZoomCommand);
            }


            delete cmd;
        }

        // Check to see if craft is in armed state
        if (m_craft.Data.armed == true)
        {
            UpdateState(armed);
            done = true;
            return;
        }
        if (m_runStateMachineLoop == false)
        {
          done = true;
          return;
        }

        VerifyConnections(true);

        UpdateCameraCraftPosition(m_controller.Data.JoyX, m_controller.Data.JoyY, m_controller.Data.JoyZ, m_controller.Data.JoyButton);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }
}// end HandleReadyToArm

void FlightManagementSystem::HandleArming()
{
    if (IsVoltageGoodToArm() == false)
    {
        // Send a message to the controller
        char msg[50];
        strncpy(msg, "Bad Voltage", 50);
        strncat(msg, FMS_VERSION, 49);
        StatusText* statusText = new StatusText();
        statusText->severity = 0;
        strncpy(statusText->text, msg, 50);
        m_communicationsHub.SendStatusText(statusText);

        if (IsBadBatteryVoltage() == true)
        {
            strncpy(msg, "Backup Battery Low", 50);
            strncat(msg, FMS_VERSION, 49);
            StatusText* statusText = new StatusText();
            statusText->severity = 0;
            strncpy(statusText->text, msg, 50);
            m_communicationsHub.SendStatusText(statusText);
        }

        UpdateState(readyToArm);
        return;
    }


    // Clear any commands
    m_controller.Data.ClearCommandQueue();

    m_skyBox.SendReelResetTetherPos();
    
    // Capture craft initial heading
    landHeadingCraftInitial = (float) m_craft.Data.heading / 100.0f;
    
    
    m_craft.SendArm();

/* MOVED
    // Capture the ground pressures if we are using baro compensate
    if (m_baroCompensate == true)
    {
        // Update the starting pressures
        m_craftStartPressure = m_craft.Data.pressAbs;
        m_craftStartPressure2 = m_craft.Data.pressAbs2;
        m_groundStartPressure = m_groundBaro.Data.pressAbs;

        m_lastDeltaPressure = 0.0f;
        m_groundBaroDataValid = true;
    }
*/

    bool done = false;
    int counter = 0;
    while (!done)
    {
        if (m_craft.Data.armed == true)
        {
            done = true;
        }
        else
        {
            counter++;
            if (counter > 5)
            {
                done = true;
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }

        VerifyConnections(true);

        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

    }

    if (m_craft.Data.armed == true)
    {
        UpdateState(armed);
    }
    else
    {
        UpdateState(readyToArm);
    }
}// end HandleArming



void FlightManagementSystem::HandleArmed()
{
    // Clear any commands
    m_controller.Data.ClearCommandQueue();

    // Override yaw and pan
    m_craft.SendRcOverridesLow(65535, 65535, 65535, 1500, 65535, 1500, 65535, 65535);


    m_slewToTkHeadingFirstTime = true;
    

    bool done = false;

    while(!done)
    {
        // Check for automatic disarm
        if (m_craft.Data.armed == false)
        {

            // Release yaw and pan
            if (m_yawFollowsPan == true)
            {
                m_craft.SendRcOverridesLow(65535, 65535, 65535, 0, 65535, 1500, 65535, 65535);
            }

            
            UpdateState(readyToArm);
            return;
        }

        // Check that GPS is still good
        if (IsAllGpsGood() == false)
        {
            // Disarm the craft
            while (m_craft.Data.armed == true)
            {
                m_craft.SendDisarm();
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
            }

            // Go back to acquiring GPS

            // Release yaw and pan
            if (m_yawFollowsPan == true)
            {
                m_craft.SendRcOverridesLow(65535, 65535, 65535, 0, 65535, 1500, 65535, 65535);
            }
            
            UpdateState(acquiringCraftGPS);
            return;
        }

        // Look to see if craft is flying (takeoff command sync issue)
        if (IsCraftFlying() == true)
        {
            // Release yaw and pan
            if (m_yawFollowsPan == true)
            {
                m_craft.SendRcOverridesLow(65535, 65535, 65535, 0, 65535, 1500, 65535, 65535);
            }

            HFLogger::logMessage("Craft appears to be in flight");
            UpdateState(flightHold);
            return;
        }

        // Look for commands
        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            // Clear any other queued commands
            m_controller.Data.ClearCommandQueue();


            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                HFLogger::logMessage("Kill Command");
                UpdateState(killCraft);
                delete cmd;
                return;
            }

            // Look for a launch command
            if (cmd->CommandType() == HFCMD_LAUNCH)
            {
                HFLogger::logMessage("Launch command");
                UpdateState(launch);
                done = true;
            }

            // Look for a disarm command
            if (cmd->CommandType() == HFCMD_DISARM)
            {
                HFLogger::logMessage("Disarm command");

                // Disarm the craft
                while (m_craft.Data.armed == true)
                {
                    m_craft.SendDisarm();
                    std::this_thread::sleep_for(std::chrono::milliseconds(250));
                }
                UpdateState(readyToArm);
                done = true;
            }

            // Delete this command
            delete cmd;

        }

        VerifyConnections(true);


        std::this_thread::sleep_for(std::chrono::milliseconds(250));

    }
}// end HandleArmed

bool FlightManagementSystem::IsCraftFlying()
{
    bool inFlight = false;
    for (int i = 0; i < 4; i++)
    {
        if (m_craft.Data.servoOutputs[i] > 1300)
        {
            inFlight = true;
        }
    }

    return inFlight;
}// end IsCraftFlying

bool FlightManagementSystem::CheckForKillCommand()
{
    HFCommand *cmd;

    bool validKillCommand = false;

    // Check for kill craft command
    cmd = m_controller.Data.GetFirstCommand();
    if (cmd != nullptr)
    {
        if (cmd->CommandType() == HFCMD_KILL_CRAFT)
        {
            validKillCommand = true;
        }

        m_controller.Data.ClearCommandQueue();
    }

    return validKillCommand;
}// end CheckForKillCommand




void FlightManagementSystem::HandleLaunching()
{

    // Clear any commands
    m_controller.Data.ClearCommandQueue();
    
    
    landingSetHomeDisabledAfterHalt = false;


    // Check the voltage prior to allowing launch
    if (VerifyFlightVoltage() != 0)
    {
        // Send a message to the controller
        char msg[50];
        strncpy(msg, "BAD VOLTAGE", 50);
        StatusText* statusText = new StatusText();
        statusText->severity = 0;
        strncpy(statusText->text, msg, 50);
        m_communicationsHub.SendStatusText(statusText);

        UpdateState(armed);
        return;
    }

    // Reset out hold point to this location
    m_holdPoint.ResetHoldPoint(m_craft.Data.latitude, m_craft.Data.longitude, m_groundGPS.Data.latitude, m_groundGPS.Data.longitude);

    // Set the hold altitude to minimum
    m_holdPoint.SetAltitude((float)m_minAltitude/100.0f);
    m_holdPoint.SetFollowMode(false);


    // Send takeoff to minimum altitude
    int retryCount = 0;
    bool validAck = false;

    while (validAck == false)
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        m_craft.SendTakeoff(m_holdPoint.CraftTargetAltitude());
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        if (m_craft.TakeoffAcknowledged() == true)
        {
            validAck = true;
        }
        else
        {
            retryCount++;
            if (retryCount > 10)
            {
                UpdateState(armed);
                return;
            }
        }
        
        VerifyConnections(true);
    }

    // Mark flight as started and grab time
    m_flightStarted = true;
    time(&m_flightStartTime);

    // Wait 10 seconds if using original min altitude, or 15 otherwise
    int takeoffTimeout = 100;
    if (m_minAltitude == 500)
    {
        takeoffTimeout = 100;
    }
    else
    {
        takeoffTimeout = 150;
    }

    for (int i = 0; i < takeoffTimeout; i++)
    {
        // Look for a command
        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            m_controller.Data.ClearCommandQueue();

            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                return;
            }

            // Look for a land command
            if (cmd->CommandType() == HFCMD_LAND)
            {                    
                m_useLandingHeading = cmd->UseLandHeading;
                m_useLandingHeading = true;
                m_landingHeadingTarget = cmd->LandHeading;
                
                // Disable the slew to TK heading
                m_slewToTkHeadingFirstTime = false;

                HFLogger::logMessage("Land command");

                UpdateState(land);
                delete cmd;
                return;
            }
        }
        
        // Look for craft to read disarmed 
        // This may occur if the launch command was issues as the craft disarms
        if (m_craft.Data.armed == false)
        {
            // Send the system to land mode
            // This will recycle the system, but also ensure a landing in case of a false state
            HFLogger::logMessage("Craft appears to have disarmed - forcing land mode");
            UpdateState(land);
            return;
        }

        VerifyConnections(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Capture craft realigned heading
    landHeadingCraftAfterRealign = (float) m_craft.Data.heading / 100.0f;
    // Calculate Craft Delta
    landHeadingCraftDelta = landHeadingCraftAfterRealign - landHeadingCraftInitial;
    
    
    

    if (m_usePowerLimitCeiling == true)
    {
        // Reset the power loss monitor count
        powerLossMonitor.Reset(bguData.powerLossCount);
    }
    
    
    uint16_t rcChannels[8];
    rcChannels[RC_PITCH]    = 1500;     // Center pitch
    rcChannels[RC_ROLL]     = 1500;     // Center roll
    rcChannels[RC_THROTTLE] = 1000;     // Low throttle
    rcChannels[RC_YAW]      = 65535;    // Ignore yaw
    rcChannels[RC_MODE]     = 2000;     // Mode guided
    rcChannels[RC_PAN]      = 65535;    // Ignore pan
    rcChannels[RC_ZOOM]     = 1100;     // Set precision land off
    rcChannels[RC_TILT]     = 65535;    // Ignore tilt
    
    // Release Roll and Pitch if we are using precision land
    if ((m_precisionLanding == true) && (m_emergencyLand == false))
    {
        rcChannels[RC_PITCH] = 0;       
        rcChannels[RC_ROLL] = 0;    
        rcChannels[RC_ZOOM] = 0;    
        
        // Release throttle and mode if we are using landing retry
        if (m_landingRetryEnabled == true)
        {
            rcChannels[RC_THROTTLE] = 0;
            rcChannels[RC_MODE] = 0;
        }
    }
    
    // Release yaw if we are using yaw follows pan
    if (m_yawFollowsPan == true)
    {
        rcChannels[RC_YAW] = 0;
        rcChannels[RC_PAN] = 1500;
    }
    
    // Send the overrides
    m_craft.SendRcOverridesLow(rcChannels);
    
    
    
    // Issue a locker open command to reset the locking mechanism to nominal open position
    if (m_useLocker == true)
    {
        m_skyBox.LockerOpen();
    }
    
    /*
    //// Landing Retry Redux
    if ((m_precisionLanding == true) && (m_emergencyLand == false))
    {
        //m_craft.SendRcOverridesLow(0, 0, 1000, 65535, 65535, 65535, 65535, 65535);
        m_craft.SendRcOverridesLow(0, 0, 0, 65535, 0, 65535, 65535, 65535);            
    }
    else
    {
        //m_craft.SendRcOverridesLow(65535, 65535, 1000, 65535, 65535, 65535, 65535, 65535);
        m_craft.SendRcOverridesLow(65535, 65535, 0, 65535, 0, 65535, 65535, 65535);
    }
    //// Landing Retry Redux - end


    // Release yaw and pan
    if (m_yawFollowsPan == true)
    {
        m_craft.SendRcOverridesLow(65535, 65535, 65535, 0, 65535, 1500, 65535, 65535);
    }
    */

    UpdateState(flightHold);

}// end HandleLaunching

void FlightManagementSystem::HandleFlight()
{
    // Clear craft initiated landing
    craftInitiatedLanding = false;
    
    // Clear any commands
    m_controller.Data.ClearCommandQueue();

    bool done = false;

    int loopCounter = 0;

    float emergencyDescentAltitude = 0.0f;
    int emergencyDescentCounter = 0;
    bool inEmergencyDescent = false;

    while(!done)
    {
        // Verify that there is an active controller connected
//        if (m_communicationsHub.IsControllerActive(5) == false)
        if (m_communicationsHub.IsControllerActive(m_lostCommTimeout) == false)
        {
            HFLogger::logMessage("No controller present - landing");
            UpdateState(land);
            return;
        }


        // Look for a bad voltage
        if (IsVoltageRangeGood() == false)
        {
            // Send a message to the controller
            char msg[50];
            strncpy(msg, "Bad Voltage - landing", 50);
            StatusText* statusText = new StatusText();
            statusText->severity = 0;
            strncpy(statusText->text, msg, 50);
            m_communicationsHub.SendStatusText(statusText);

            UpdateState(land);
            return;
        }
        
        if (landingAutoAborted == true) 
        {
            if (CriticalTetherVoltageCheck() == false)
            {
                // Send a message to the controller
                char msg[50];
                strncpy(msg, "Critical Tether Voltage - landing", 50);
                StatusText* statusText = new StatusText();
                statusText->severity = 0;
                strncpy(statusText->text, msg, 50);
                m_communicationsHub.SendStatusText(statusText);

                UpdateState(land);
                return;
            }
        }
        


        // Look for bad vibration
        if (IsVibrationAllowable() == false)
        {
            // Send a message to the controller
            char msg[50];
            strncpy(msg, "Bad Vibration - landing", 50);
            StatusText* statusText = new StatusText();
            statusText->severity = 0;
            strncpy(statusText->text, msg, 50);
            m_communicationsHub.SendStatusText(statusText);

            UpdateState(land);
            return;        
        }

        if (IsATSHealthy() == false)
        {
            //!STUB 
            // Add code here to respond to ats failures
        }
        

        // Look for bad ground baro
        if (m_baroCompensate == true)
        {
            if (m_groundBaroDataValid == false)
            {
                // Send a message to the controller
                char msg[50];
                strncpy(msg, "Bad Ground Barometer - landing", 50);
                StatusText* statusText = new StatusText();
                statusText->severity = 0;
                strncpy(statusText->text, msg, 50);
                m_communicationsHub.SendStatusText(statusText);

                HFLogger::logMessage("Bad Ground Barometer - landing");

                UpdateState(land);
                return;
            }
        }

        /*
        // Look for an emergency land request
        if (m_communicationsHub.IsEmergencyLandingRequested() == true)
        {
            HFLogger::logMessage("Emergency land requested");
            UpdateState(land);
            return;
        }
        */

        // Look for a lower altitude request
        if (m_communicationsHub.IsEmergencyDescendRequested() == true)
        {
            if (inEmergencyDescent == false)
            {
                inEmergencyDescent = true;
                HFLogger::logMessage("Emergency descend requested");
                emergencyDescentAltitude = m_holdPoint.CraftTargetAltitude() - 4.0f;
                m_holdPoint.SetAltitude(emergencyDescentAltitude);
                HFLogger::logMessage("New hold pos set");
                if (m_useV35Plus == true)
                {
                    m_craft.SendSetPositionTargetGlobalInt(m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                    if (m_noLocationLogging == false)
                    {
                        HFLogger::logMessage("New hold pos: %d, %d, %d", m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                    }
                }
                else
                {
                    m_craft.SendMissionItem(m_holdPoint.CraftHoldLatitude(), m_holdPoint.CraftHoldLongitude(), m_holdPoint.CraftTargetAltitude());
                }

                // Send a message to the controller
                char msg[50];
                strncpy(msg, "Approaching tether limit - descending", 50);
                strncat(msg, FMS_VERSION, 49);
                StatusText* statusText = new StatusText();
                statusText->severity = 0;
                strncpy(statusText->text, msg, 50);
                m_communicationsHub.SendStatusText(statusText);
            }
            else
            {
                HFLogger::logMessage("In emergency descent %d", emergencyDescentCounter);
                emergencyDescentCounter++;
                if (emergencyDescentCounter > 50)
                {
                  inEmergencyDescent = false;
                  emergencyDescentCounter = 0;
                  HFLogger::logMessage("Emergency descent complete");
                }
            }
        }
        else
        {
            inEmergencyDescent = false;
            emergencyDescentCounter = 0;
        }


        if (m_usePowerLimitCeiling == true)
        {
            // Check to see if we need to descend due to power loss
            if (powerLossMonitor.DescentRequired(bguData.powerLossCount) == true)
            {
                HFLogger::logMessage("Power loss descent required");

                // Adjust the hold point power limit ceiling delta meters lower than current position            
                float craftAltitude = m_craft.Data.relative_alt / 1000.0f;
                float powerLimitCeiling = craftAltitude - m_powerLimitDelta;
                m_holdPoint.SetPowerLimitCeiling(powerLimitCeiling);
                HFLogger::logMessage("Power limit ceiling %0.2f", powerLimitCeiling);
                
                // Set altitude to current altitude (will be limited to new ceiling)
                m_holdPoint.SetAltitude(craftAltitude);
                
                // Tell the craft to go to current position
                m_craft.SendSetPositionTargetGlobalInt(m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                if (m_noLocationLogging == false)
                {
                    HFLogger::logMessage("New hold pos: %d, %d, %d", m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                }
                
            }
        }


        // Look for a craft initiated (failsafe) landing
        if (m_craft.IsInLandMode() == true) 
        {
            HFLogger::logMessage("Craft unexpectedly in land mode");
            // A craft initiated landing has been detected
            craftInitiatedLanding = true;
            commReconnectCheckComplete = false;
            // Clear the voltage set flag to ensure we get a new voltage readin
            m_craft.Data.voltage_set = false;
            UpdateState(land);
            return;
        }
        
                
        
        // Look for GPS going away
        // Will not change flight limits until craft lands
        if ( m_gpsDeniedSupport == true)
        {
            if (IsGpsOn() == false)
            {
                if (m_gpsDeniedLandOnLoss == true) 
                {
                    HFLogger::logMessage("GPS-DENIED: Loss of GPS - LANDING");

                    UpdateState(land);
                    return;
                }
                else
                {
                    if (m_holdPoint.GetGpsDeniedMode() == false)
                    {
                        HFLogger::logMessage("GPS-DENIED: Limiting max altitude - first time");

                        // Enable GPS Denied Mode
                        m_holdPoint.SetGpsDeniedMode(true);                    
                        
                        // This will limit the altitude to the lesser of current or gps denied limit
                        float craftAltitude = m_craft.Data.relative_alt / 1000.0f;
                        m_holdPoint.SetAltitude(craftAltitude);
                    
                        // Tell the craft to go to current position
                        m_craft.SendSetPositionTargetGlobalInt(m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                        HFLogger::logMessage("New hold pos: %d, %d, %d", m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                    }   
                }             
            }
        }
        
        
        // Verify flight climb rate and set if needed
        if (m_craft.Data.wpnav_speed_up != m_flightClimbRate) 
        {
            HFLogger::logMessage("HandleFlight - setting WPNAV_SPEED_UP to %d", m_flightClimbRate);
            m_craft.UpdateWpNavSpeedUp(m_flightClimbRate);            
        }



        // Look for a command
        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            m_controller.Data.ClearCommandQueue();

            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                return;
            }

            // Look for a land command
            if (cmd->CommandType() == HFCMD_LAND)
            {
                m_useLandingHeading = cmd->UseLandHeading;
                m_useLandingHeading = true;
                m_landingHeadingTarget = cmd->LandHeading;

                HFLogger::logMessage("Land command");
                m_holdPoint.SetFollowMode(false);

                UpdateState(land);
                done = true;
                delete cmd;
                return;
            }

            // Look for an altitude change
            if (cmd->CommandType() == HFCMD_CHANGE_ALT)
            {
                if (m_systemState != flightTranslate)
                {
                    if (cmd->AbsoluteAltitude == false)
                    {
                        HFLogger::logMessage("Change altitude delta : %f", cmd->Z);
                        m_holdPoint.AdjustAltitude(cmd->Z);
                    }
                    else
                    {
                        HFLogger::logMessage("Change altitude absolute : %f", cmd->Z);
                        m_holdPoint.SetAltitude(cmd->Z);
                    }
                    HFLogger::logMessage("New hold pos set");
                    if (m_useV35Plus == true)
                    {
                        m_craft.SendSetPositionTargetGlobalInt(m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                        if (m_noLocationLogging == false)
                        {
                            HFLogger::logMessage("New hold pos: %d, %d, %d", m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                        }
                    }
                    else
                    {
                        m_craft.SendMissionItem(m_holdPoint.CraftHoldLatitude(), m_holdPoint.CraftHoldLongitude(), m_holdPoint.CraftTargetAltitude());
                    }
                }
            }

            // Look for a follow mode command
            if (cmd->CommandType() == HFCMD_FOLLOW)
            {
                if (m_noGroundGps == false)
                {
                    if (cmd->Enable == true)
                    {

                        m_holdPoint.ResetHoldPoint(m_craft.Data.latitude, m_craft.Data.longitude, m_groundGPS.Data.latitude, m_groundGPS.Data.longitude);
                        m_holdPoint.SetFollowMode(true);
                        UpdateState(flightFollow);
                        HFLogger::logMessage("Follow mode enabled");
                    }
                    else
                    {
                        m_holdPoint.ResetHoldPoint(m_craft.Data.latitude, m_craft.Data.longitude, m_groundGPS.Data.latitude, m_groundGPS.Data.longitude);
                        m_holdPoint.SetFollowMode(false);
                        UpdateState(flightHold);
                        HFLogger::logMessage("Set hold point");
                        if (m_useV35Plus == true)
                        {
                            m_craft.SendSetPositionTargetGlobalInt(m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                            if (m_noLocationLogging == false)
                            {
                                HFLogger::logMessage("New hold pos: %d, %d, %d", m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                            }
                        }
                        else
                        {
                            m_craft.SendMissionItem(m_holdPoint.CraftHoldLatitude(), m_holdPoint.CraftHoldLongitude(), m_holdPoint.CraftTargetAltitude());
                        }
                    }
                }
            }

            // Look for a flight mode command
            if (cmd->CommandType() == HFCMD_FLIGHTMODE)
            {
                if (m_precisionLanding == false)
                {
                    // Toggle flight mode
                    if (m_systemState == flightHold)
                    {
                        UpdateState(flightTranslate);
                        HFLogger::logMessage("Translate mode enabled");
                    }
                    else
                    {
                        UpdateState(flightHold);
                        m_holdPoint.SetFollowMode(false);
                        m_holdPoint.ResetHoldPoint(m_craft.Data.latitude, m_craft.Data.longitude, m_groundGPS.Data.latitude, m_groundGPS.Data.longitude);
                        HFLogger::logMessage("hold mode enabled");
                    }
                }
            }

            // Look for a reposition command
            if (cmd->CommandType() == HFCMD_REPOSITION)
            {
                if (m_systemState == flightHold)
                {
                    m_holdPoint.UpdateCraftPosition(cmd->X, cmd->Y, m_craft.Data.heading);
                    HFLogger::logMessage("Set hold point");
                    if (m_useV35Plus == true)
                    {
                        m_craft.SendSetPositionTargetGlobalInt(m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                        if (m_noLocationLogging == false)
                        {
                            HFLogger::logMessage("New hold pos: %d, %d, %d", m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                        }
                    }
                    else
                    {
                        m_craft.SendMissionItem(m_holdPoint.CraftHoldLatitude(), m_holdPoint.CraftHoldLongitude(), m_holdPoint.CraftTargetAltitude());
                    }
                }
            }

            // Look for a go to home command
            if (cmd->CommandType() == HFCMD_GO_TO_HOME)
            {
                // Only allowed in flight hold mode
                if (m_systemState == flightHold)
                {
                    HFLogger::logMessage("FMS: GO_TO_HOME");
                    HFLogger::logMessage("FMS: Craft position: %d, %d", m_craft.Data.latitude, m_craft.Data.longitude);
                    HFLogger::logMessage("FMS: Sending to home position");
                    m_holdPoint.SetToHomePosition();
                    if (m_useV35Plus == true)
                    {
                        m_craft.SendSetPositionTargetGlobalInt(m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                        if (m_noLocationLogging == false)
                        {
                            HFLogger::logMessage("New hold pos: %d, %d, %d", m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                        }
                    }
                    else
                    {
                        m_craft.SendMissionItem(m_holdPoint.CraftHoldLatitude(), m_holdPoint.CraftHoldLongitude(), m_holdPoint.CraftTargetAltitude());
                    }
                }
            }

            // Look for a navigate to position command
            if (cmd->CommandType() == HFCMD_NAVIGATE_TO_POS)
            {
                if (m_systemState == flightHold)
                {
                    m_holdPoint.SetCraftPosition(cmd->X, cmd->Y);
                    HFLogger::logMessage("Navigate to pos - lat:%f, lon:%f", m_holdPoint.CraftHoldLatitude(), m_holdPoint.CraftHoldLongitude());
                    if (m_useV35Plus == true)
                    {
                        m_craft.SendSetPositionTargetGlobalInt(m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                        if (m_noLocationLogging == false)
                        {
                            HFLogger::logMessage("New hold pos: %d, %d, %d", m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                        }
                    }
                    else
                    {
                        m_craft.SendMissionItem(m_holdPoint.CraftHoldLatitude(), m_holdPoint.CraftHoldLongitude(), m_holdPoint.CraftTargetAltitude());
                    }
                }
            }


            if (cmd->CommandType() == HFCMD_MOUNT_CTRL)
            {
                int deltaTilt = (int)cmd->DeltaTilt;

                m_craft.AdjustGimbalTiltAngle(deltaTilt);
            }

            if (cmd->CommandType() == HFCMD_REMOTE_CTRL)
            {
                m_useRemoteJoystick = cmd->Enable;
            }

            if (cmd->CommandType() == HFCMD_COLOR_PALETTE)
            {
                ColorPaletteControl(cmd->PaletteCommand);
            }

            if (cmd->CommandType() == HFCMD_ZOOM)
            {
                ZoomControl(cmd->ZoomCommand);
                //ZoomControl(1, cmd->ZoomCommand);
                //ZoomControl(2, cmd->ZoomCommand);
            }
            if (cmd->CommandType() == HFCMD_ZOOMSCALE)
            {
                ZoomScaleControl(cmd->ZoomCommand);
            }


            if (cmd->CommandType() == HFCMD_PRECISION_LAND)
            {
                if (cmd->Enable == true)
                {
                    m_precisionLanding = true;
                    m_emergencyLand = false;
                }
                else
                {
                    m_precisionLanding = false;
                }
            }
            
            delete cmd;
        }

        // Send joystick / zoom commands
        UpdateCameraCraftPosition(m_controller.Data.JoyX, m_controller.Data.JoyY, m_controller.Data.JoyZ, m_controller.Data.JoyButton, m_controller.Data.RoiLatitude, m_controller.Data.RoiLongitude, m_controller.Data.RoiAltitude, m_controller.Data.NewRoi);
        m_controller.Data.NewRoi = false;

        // If we are in flight follow - update the position every two seconds
        if (m_systemState == flightFollow)
        {

            // Every 8 loops, send a position update message
            loopCounter++;
            if (loopCounter > 4)
            {
                loopCounter = 0;

                // Update the ground position
                m_holdPoint.UpdateGroundPoint(m_groundGPS.Data.latitude, m_groundGPS.Data.longitude);

                HFLogger::logMessage("Ground Pos     - lat:%d, lon:%d", m_groundGPS.Data.latitude, m_groundGPS.Data.longitude);
                HFLogger::logMessage("Follow point   - lat:%f, lon:%f, alt:%f", m_holdPoint.CraftHoldLatitude(), m_holdPoint.CraftHoldLongitude(), m_holdPoint.CraftTargetAltitude());
                if (m_useV35Plus == true)
                {
                    m_craft.SendSetPositionTargetGlobalInt(m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                    if (m_noLocationLogging == false)
                    {
                        HFLogger::logMessage("New hold pos: %d, %d, %d", m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                    }
                }
                else
                {
                    m_craft.SendMissionItem(m_holdPoint.CraftHoldLatitude(), m_holdPoint.CraftHoldLongitude(), m_holdPoint.CraftTargetAltitude());
                    if (m_noLocationLogging == false)
                    {
                        HFLogger::logMessage("New hold pos: %d, %d, %d", m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                    }
                }
            }
        }

        // REMOVED BECAUSE OF DUAL GPS HARDWARE ISSUE
        /*
        // Check for loss of GPS
        if (m_craft.Data.IsGpsGood() == false)
        {
            HFLogger::logMessage("Loss of GPS - sat:%d, hdop:%d - land at lat:%f, lon:%f, alt:%f", m_craft.Data.gpsNumberSatellites, m_craft.Data.gpsHdop, m_holdPoint.CraftHoldLatitude(), m_holdPoint.CraftHoldLongitude(), m_holdPoint.CraftTargetAltitude());
            m_holdPoint.ResetHoldPoint(m_craft.Data.latitude, m_craft.Data.longitude, m_groundGPS.Data.latitude, m_groundGPS.Data.longitude);
            UpdateState(land);
            return;
        }
        */

        VerifyConnections(true);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}// end HandleFlight



void FlightManagementSystem::HandleLanding()
{
    bool exitLanding = false;
    
    if (craftInitiatedLanding == false)
    {
        HFLogger::logMessage("FMS initiated landing");
        exitLanding = HandleLandingPhase1();
    }
    else 
    {
        HFLogger::logMessage("Craft initiated landing");
    }
    
    if (exitLanding == false) 
    {
        HandleLandingPhase2();
    }
}




bool FlightManagementSystem::HandleLandingPhase1()
{
    m_useRemoteJoystick = false;

    if ((m_landingSetHome == true) && (m_precisionLanding == true))
    {
        if (landingSetHomeDisabledAfterHalt == false) 
        {
            // Only allowed in flight hold mode
            if (landingFromFlightHold == true)
            {
                HFLogger::logMessage("FMS: GO_TO_HOME");
                HFLogger::logMessage("FMS: Craft position: %d, %d", m_craft.Data.latitude, m_craft.Data.longitude);
                HFLogger::logMessage("FMS: Sending to home position");
                m_holdPoint.SetToHomePosition();

                m_craft.SendSetPositionTargetGlobalInt(m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                HFLogger::logMessage("New hold pos: %d, %d, %d", m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                
             
                int landingSetHomeDelayCount = 0;
                while (landingSetHomeDelayCount < 30)
                {
                    landingSetHomeDelayCount++;
                    
                    // Look for a command
                    HFCommand* cmd = m_controller.Data.GetFirstCommand();
                    if (cmd != nullptr)
                    {
                        m_controller.Data.ClearCommandQueue();

                        // Look for a kill craft command
                        if (cmd->CommandType() == HFCMD_KILL_CRAFT)
                        {
                            UpdateState(killCraft);
                            return true;
                        }

                        // Look for a change altitude command
                        if (cmd->CommandType() == HFCMD_CHANGE_ALT)
                        {
                            landingSetHomeDisabledAfterHalt = true;
                            
                            // Verify that we can abort landing
                            bool inLowVoltage = false;
                            int minVoltage;


                            if (m_bigSky == false) {
                                minVoltage = 22000;
                            } else {
                                minVoltage = 44000;
                            }

                            if (m_craft.Data.batteryVoltage < minVoltage)
                            {
                                inLowVoltage = true;
                            }

                            if (inLowVoltage == false)
                            {

                                // Attempt to abort landing

                                // If abortPL is set to true, set this as an emergency landing
                                if (m_abortPL == true)
                                {
                                    m_emergencyLand = true;
                                }

                                // Regardless of command, set the target altitude to the current altitude
                                float abortAltitude = (float)m_minAltitude / 100.0f;
                                abortAltitude = m_craft.Data.relative_alt / 1000.0f;

                                HFLogger::logMessage("ABORT LAND - Change altitude absolute : %f", abortAltitude);

                                // Reset out hold point to this location
                                m_holdPoint.ResetHoldPoint(m_craft.Data.latitude, m_craft.Data.longitude, m_groundGPS.Data.latitude, m_groundGPS.Data.longitude);
                                m_holdPoint.SetAltitude(abortAltitude);


                                HFLogger::logMessage("ABORT LAND New hold pos set");


                                m_craft.SendSetPositionTargetGlobalInt(m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());



                                // Put the craft in guided (alternate)
                                HFLogger::logMessage("ABORT LAND - Forcing mode to alternate guided");
                                while (m_craft.Data.rcChannels[RC_MODE] != 1300)
                                {
                                    // Check for kill craft command
                                    if (CheckForKillCommand() == true)
                                    {
                                        UpdateState(killCraft);
                                        return true;
                                    }
                                    m_craft.SendRcOverridesLow(1500, 1500, 65535, 65535, 1300, 65535, 65535, 65535);
                                    std::this_thread::sleep_for(std::chrono::seconds(1));
                                }


                                // Put the craft in guided
                                HFLogger::logMessage("ABORT LAND - Forcing mode to guided");
                                while (m_craft.Data.rcChannels[RC_MODE] != 2000)
                                {
                                    // Check for kill craft command
                                    if (CheckForKillCommand() == true)
                                    {
                                        UpdateState(killCraft);
                                        return true;
                                    }

                                    m_craft.SendRcOverridesLow(1500, 1500, 65535, 65535, 2000, 65535, 65535, 65535);
                                    std::this_thread::sleep_for(std::chrono::seconds(1));
                                }



                                m_craft.SendSetPositionTargetGlobalInt(m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());


                                UpdateState(flightHold);
                                return true;
                            }

                        }
                    }
                    
                    VerifyConnections(true);
                    
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));

                }
            }      
            else 
            {
                HFLogger::logMessage("Landing set home disabled if not in flight hold");
            }      
        }
        else
        {
            HFLogger::logMessage("Landing set home disabled after halt");
        }
    }
    
    // Send the camera to the 0 position
    m_craft.SetGimbalTiltAngle(90000);
    
    
    uint16_t rcChannels[8];

    rcChannels[RC_PITCH]        = 1500;     // Center pitch
    rcChannels[RC_ROLL]         = 1500;     // Center roll
    rcChannels[RC_THROTTLE]     = 1500;     // Mid Throttle
    rcChannels[RC_YAW]          = 1500;     // Center yaw
    rcChannels[RC_MODE]         = 2000;     // Guided
    rcChannels[RC_PAN]          = 1500;     // Center pan
    rcChannels[RC_ZOOM]         = 1100;     // disable precision land
    rcChannels[RC_TILT]         = m_craft.GetGimbalTiltOutput();

    // If precision land is enabled, release roll and pitch
    if ((m_precisionLanding == true) && (m_emergencyLand == false))
    {
        rcChannels[RC_PITCH] = 0;
        rcChannels[RC_ROLL] = 0;
        rcChannels[RC_ZOOM] = 0;
        
        // If landing retry is enabled, release mode and throttle
        if (m_landingRetryEnabled == true)
        {
            rcChannels[RC_THROTTLE] = 0;
            rcChannels[RC_MODE] = 0;
        }
    }    
    
    HFLogger::logMessage("Landing - RC Values: %d, %d, %d, %d, %d, %d, %d, %d", rcChannels[0], rcChannels[1], rcChannels[2], rcChannels[3], rcChannels[4], rcChannels[5], rcChannels[6], rcChannels[7]);
    bool verified = false;
    while (!verified)
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return true;
        }

        m_craft.SendRcOverridesLow(rcChannels);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        // verify should test only overridden channels
        if (m_craft.VerifyRcOverridesLow(rcChannels) == true)
        {
            verified = true;
        }
        
        VerifyConnections(true);
    }
    
    // If we are using alignTkHeading, slew to it
    if (m_useAlignTkHeading == true)
    {
        bool exitLanding = SlewToTkHeading(rcChannels);
        if (exitLanding == true)
        {
            // SlewToTkHeading returned that it left land mode
            // report this back
            return true;
        }
    }
    
   
    
    // Issue and verify land command
    HFLogger::logMessage("Sending land command");
    bool landing = false;
    int retryCount = 0;
    while ((!landing) && (retryCount < 10))
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return true;
        }

        m_craft.SendLand();
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        if (m_craft.LandAcknowledged() == true)
        {
            landing = true;
        }

        retryCount++;
        
        VerifyConnections(true);
    }
    
    return false;
        
}


void FlightManagementSystem::HandleLandingPhase2()
{
    
    m_controller.ClearManualCommands();


    // Look for landing and/or disarming
    bool craftDisarmed = false;
    int craftDisarmedCount = 0;
    int motorsLowCount = 0;
    int minHeartbeatCount = 0;
    while (craftDisarmed == false) //m_craft.Data.armed == true)
    {
     
        // If this was craft initiated and commLossLandAbort is true
        if ((craftInitiatedLanding == true) && (m_commLossLandAbort == true))
        {
            // If we haven't checked yet, but have new voltage readings
            if ((commReconnectCheckComplete == false) && (m_craft.Data.voltage_set == true))
            {
                // We will have completed a check - don't repeat
                commReconnectCheckComplete = true;
                
                // Look for a good voltage (and that we have received a new one)
                if (CriticalVoltageCheck() == true)
                {
                    landingAutoAborted = true;
                    
                    // This appears to be comm loss reconnection
                    // attempt to abort the landing
                    HFLogger::logMessage("Comm Loss Reconnect - aborting landing");
                    m_craft.SendSetPositionTargetGlobalInt(m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                    
                    // Put the craft in guided (alternate)
                    HFLogger::logMessage("ABORT LAND - Forcing mode to alternate guided");
                    while (m_craft.Data.rcChannels[RC_MODE] != 1300)
                    {
                        // Check for kill craft command
                        if (CheckForKillCommand() == true)
                        {
                            UpdateState(killCraft);
                            return;
                        }
                        m_craft.SendRcOverridesLow(1500, 1500, 65535, 65535, 1300, 65535, 65535, 65535);
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        
                        VerifyConnections(true);
                    }


                    // Put the craft in guided
                    HFLogger::logMessage("ABORT LAND - Forcing mode to guided");
                    while (m_craft.Data.rcChannels[RC_MODE] != 2000)
                    {
                        // Check for kill craft command
                        if (CheckForKillCommand() == true)
                        {
                            UpdateState(killCraft);
                            return;
                        }

                        m_craft.SendRcOverridesLow(1500, 1500, 65535, 65535, 2000, 65535, 65535, 65535);
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        
                        VerifyConnections(true);
                    }



                    m_craft.SendSetPositionTargetGlobalInt(m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());


                    UpdateState(flightHold);
                    return;                    
                }    
           
            }

        }

        // Look for a command
        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            m_controller.Data.ClearCommandQueue();

            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                return;
            }

            // Look for a change altitude command
            if (cmd->CommandType() == HFCMD_CHANGE_ALT)
            {
                
                landingSetHomeDisabledAfterHalt  = true;

                // Verify that we can abort landing
                bool inLowVoltage = false;
                int minVoltage;


                if (m_bigSky == false) {
                    minVoltage = 22000;
                } else {
                    minVoltage = 44000;
                }

                if (m_craft.Data.batteryVoltage < minVoltage)
                {
                    inLowVoltage = true;
                }

                if (inLowVoltage == false)
                {

                    // Attempt to abort landing

                    // If abortPL is set to true, set this as an emergency landing
                    if (m_abortPL == true)
                    {
                        m_emergencyLand = true;
                    }

                    // Regardless of command, set the target altitude to the current altitude
                    float abortAltitude = (float)m_minAltitude / 100.0f;
                    abortAltitude = m_craft.Data.relative_alt / 1000.0f;

                    HFLogger::logMessage("ABORT LAND - Change altitude absolute : %f", abortAltitude);

                    // Reset out hold point to this location
                    m_holdPoint.ResetHoldPoint(m_craft.Data.latitude, m_craft.Data.longitude, m_groundGPS.Data.latitude, m_groundGPS.Data.longitude);
                    m_holdPoint.SetAltitude(abortAltitude);


                    HFLogger::logMessage("ABORT LAND New hold pos set");


                    m_craft.SendSetPositionTargetGlobalInt(m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());



                    // Put the craft in guided (alternate)
                    HFLogger::logMessage("ABORT LAND - Forcing mode to alternate guided");
                    while (m_craft.Data.rcChannels[RC_MODE] != 1300)
                    {
                        // Check for kill craft command
                        if (CheckForKillCommand() == true)
                        {
                            UpdateState(killCraft);
                            return;
                        }
                        m_craft.SendRcOverridesLow(1500, 1500, 65535, 65535, 1300, 65535, 65535, 65535);
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        
                        VerifyConnections(true);
                    }


                    // Put the craft in guided
                    HFLogger::logMessage("ABORT LAND - Forcing mode to guided");
                    while (m_craft.Data.rcChannels[RC_MODE] != 2000)
                    {
                        // Check for kill craft command
                        if (CheckForKillCommand() == true)
                        {
                            UpdateState(killCraft);
                            return;
                        }

                        m_craft.SendRcOverridesLow(1500, 1500, 65535, 65535, 2000, 65535, 65535, 65535);
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        
                        VerifyConnections(true);
                    }



                    m_craft.SendSetPositionTargetGlobalInt(m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());


                    UpdateState(flightHold);
                    return;
                }

            }
        }



        /*
        bool motorsLow = true;
        if (m_craft.Data.servoOutputs[0] > 1200)
          motorsLow = false;
        if (m_craft.Data.servoOutputs[1] > 1200)
          motorsLow = false;
        if (m_craft.Data.servoOutputs[2] > 1200)
          motorsLow = false;
        if (m_craft.Data.servoOutputs[3] > 1200)
          motorsLow = false;

        if (motorsLow == true)
        {
            motorsLowCount++;
        }
        else
        {
            motorsLowCount = 0;
        }

        if (motorsLowCount > 10)
        {
            HFLogger::logMessage("Motors low - disarming");
            m_craft.SendDisarm();
        }
        */
        
        
        if (m_craft.Data.armed == false)
        {
            HFLogger::logMessage("Disarmed - count: %d", craftDisarmedCount);
            craftDisarmedCount++;
            if (craftDisarmedCount > 5)
            {
                if (m_craft.Data.heartbeatCount > minHeartbeatCount)
                {
                    HFLogger::logMessage("Disarmed count is good");
                    craftDisarmed = true;
                }
            }
        }
        else
        {
            craftDisarmedCount = 0;
            minHeartbeatCount = m_craft.Data.heartbeatCount + 5;
        }
        
        VerifyConnections(true);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }


    // Craft is now disarmed

    // mark flight as ended
    m_flightStarted = false;

    // Force craft into land mode
    HFLogger::logMessage("Forcing mode to land");
    while (m_craft.Data.rcChannels[RC_MODE] != 1000)
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        if ((m_precisionLanding == true) && (m_emergencyLand == false))
        {
            m_craft.SendRcOverridesLow(0, 0, 65535, 65535, 1000, 65535, 0, 65535);
        }
        else
        {
            m_craft.SendRcOverridesLow(1500, 1500, 65535, 65535, 1000, 65535, 1100, 65535);
        }

        VerifyConnections(true);

        std::this_thread::sleep_for(std::chrono::seconds(1));
       
    }

    // Wait up to 5 seconds for craft to enter land mode
    int timeout = 0;
    while ((m_craft.Data.custom_mode != flightmode_land) && (timeout < 5))
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        timeout++;
        
        VerifyConnections(true);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Put the craft in stabilize
    HFLogger::logMessage("Forcing mode to stabilize");
    while (m_craft.Data.rcChannels[RC_MODE] != 1420)
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        if ((m_precisionLanding == true) && (m_emergencyLand == false))
        {
            m_craft.SendRcOverridesLow(0, 0, 65535, 65535, 1420, 65535, 0, 65535);
        }
        else
        {
            m_craft.SendRcOverridesLow(1500, 1500, 65535, 65535, 1420, 65535, 1100, 65535);
        }
        
        VerifyConnections(true);
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

     // Wait up to 5 seconds for craft to enter stabilize mode
    timeout = 0;
    while ((m_craft.Data.custom_mode != flightmode_stabilize) && (timeout < 5))
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        timeout++;
        
        VerifyConnections(true);
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Put the craft in guided
    HFLogger::logMessage("Forcing mode to guided");
    while (m_craft.Data.rcChannels[RC_MODE] != 2000)
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        if ((m_precisionLanding == true) && (m_emergencyLand == false))
        {
            m_craft.SendRcOverridesLow(0, 0, 65535, 65535, 2000, 65535, 0, 65535);
        }
        else
        {
            m_craft.SendRcOverridesLow(1500, 1500, 65535, 65535, 2000, 65535, 1100, 65535);
        }
        
        VerifyConnections(true);
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

     // Wait up to 5 seconds for craft to enter guided mode
    timeout = 0;
    while ((m_craft.Data.custom_mode != flightmode_guided) && (timeout < 5))
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        timeout++;
        
        VerifyConnections(true);
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }


    // Reset the ground barometer offset measurement
    m_groundBaroDataValid = false;

    UpdateState(readyToArm);

}



void FlightManagementSystem::HandleLanding_old()
{
    m_useRemoteJoystick = false;

    // Send the camera to the 0 position
    m_craft.SetGimbalTiltAngle(90000);






    /*
    //// Landing Retry Redux
    // Set channels for guided, mid throttle (in case we are in loiter), zero tilt
    rcChannels[RC_PITCH] = 1500;
    rcChannels[RC_ROLL] = 1500;
    //rcChannels[RC_THROTTLE] = 1500;
    rcChannels[RC_THROTTLE] = 0;
    rcChannels[RC_YAW] = 1500;
    //rcChannels[RC_MODE] = 2000;
    rcChannels[RC_MODE] = 0;
    rcChannels[RC_PAN] = 1500;
    rcChannels[RC_ZOOM] = 1500;
    rcChannels[RC_TILT] = m_craft.GetGimbalTiltOutput();
    //// Landing Retry Redux - end
    

    if ((m_precisionLanding == true) && (m_emergencyLand == false))
    {
        rcChannels[RC_PITCH] = 0;
        rcChannels[RC_ROLL] = 0;
    }
    */
    /*** Disable for now
    // allow craft to rotate to desired heading
    if (m_useLandingHeading == true)
    {
        rcChannels[RC_YAW] = 0;
    }
    ***/
    
    uint16_t rcChannels[8];

    rcChannels[RC_PITCH]        = 1500;     // Center pitch
    rcChannels[RC_ROLL]         = 1500;     // Center roll
    rcChannels[RC_THROTTLE]     = 1500;     // Mid Throttle
    rcChannels[RC_YAW]          = 1500;     // Center yaw
    rcChannels[RC_MODE]         = 2000;     // Guided
    rcChannels[RC_PAN]          = 1500;     // Center pan
    rcChannels[RC_ZOOM]         = 1100;     // disable precision land
    rcChannels[RC_TILT]         = m_craft.GetGimbalTiltOutput();

    // If precision land is enabled, release roll and pitch
    if ((m_precisionLanding == true) && (m_emergencyLand == false))
    {
        rcChannels[RC_PITCH] = 0;
        rcChannels[RC_ROLL] = 0;
        rcChannels[RC_ZOOM] = 0;
        
        // If landing retry is enabled, release mode and throttle
        if (m_landingRetryEnabled == true)
        {
            rcChannels[RC_THROTTLE] = 0;
            rcChannels[RC_MODE] = 0;
        }
    }    
    
    HFLogger::logMessage("Landing - RC Values: %d, %d, %d, %d, %d, %d, %d, %d", rcChannels[0], rcChannels[1], rcChannels[2], rcChannels[3], rcChannels[4], rcChannels[5], rcChannels[6], rcChannels[7]);
    bool verified = false;
    while (!verified)
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        m_craft.SendRcOverridesLow(rcChannels);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        // verify should test only overridden channels
        if (m_craft.VerifyRcOverridesLow(rcChannels) == true)
        {
            verified = true;
        }
    }

    /*** Disable for now
    if (m_useLandingHeading == true)
    {
        // Turn to prescribed heading
        // by now, the rcChannels are verified.  These are the same as the values the
        // craft is set to (and hopefully actually getting to)
        if ( SlewToLandHeading(rcChannels) != true )
        {
            // Slewing stopped due to change in mode to stop landing (halt or killcraft)
            // exit HandleLanding and return to state loop
            return;
        }
    }
    ***/

    /*** This appears to be duplicated below
    //Lock the heading now that we are pointing the right way
    rcChannels[RC_YAW] = 1500;
    HFLogger::logMessage("Lock yaw at current heading");

    verified = false;
    while (!verified)
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        m_craft.SendRcOverridesLow(rcChannels);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (m_craft.VerifyRcOverridesLow(rcChannels) == true)
        {
            verified = true;
        }
    }
    ***/

    //// Landing Retry Redux
    /*
    // Now ensure that we are at low throttle
    HFLogger::logMessage("Lowering throttle");
    rcChannels[RC_THROTTLE] = 1000;
    verified = false;
    while (!verified)
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        m_craft.SendRcOverridesLow(rcChannels);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (m_craft.VerifyRcOverridesLow(rcChannels) == true)
        {
            verified = true;
        }
    }
    */
    //// Landing Retry Redux - end
    
    // Issue and verify land command
    HFLogger::logMessage("Sending land command");
    bool landing = false;
    int retryCount = 0;
    while ((!landing) && (retryCount < 10))
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        m_craft.SendLand();
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        if (m_craft.LandAcknowledged() == true)
        {
            landing = true;
        }

        retryCount++;
    }

    m_controller.ClearManualCommands();


    // Look for landing and/or disarming
    bool craftDisarmed = false;
    int craftDisarmedCount = 0;
    int motorsLowCount = 0;
    int minHeartbeatCount = 0;
    while (craftDisarmed == false) //m_craft.Data.armed == true)
    {

        // Look for a command
        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            m_controller.Data.ClearCommandQueue();

            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                return;
            }

            // Look for a change altitude command
            if (cmd->CommandType() == HFCMD_CHANGE_ALT)
            {
                // Verify that we can abort landing
                bool inLowVoltage = false;
                int minVoltage;


                if (m_bigSky == false) {
                    minVoltage = 22000;
                } else {
                    minVoltage = 44000;
                }

                if (m_craft.Data.batteryVoltage < minVoltage)
                {
                    inLowVoltage = true;
                }

                if (inLowVoltage == false)
                {

                    // Attempt to abort landing

                    // If abortPL is set to true, set this as an emergency landing
                    if (m_abortPL == true)
                    {
                        m_emergencyLand = true;
                    }

                    // Regardless of command, set the target altitude to the current altitude
                    float abortAltitude = (float)m_minAltitude / 100.0f;
                    abortAltitude = m_craft.Data.relative_alt / 1000.0f;

                    HFLogger::logMessage("ABORT LAND - Change altitude absolute : %f", abortAltitude);

                    // Reset out hold point to this location
                    m_holdPoint.ResetHoldPoint(m_craft.Data.latitude, m_craft.Data.longitude, m_groundGPS.Data.latitude, m_groundGPS.Data.longitude);
                    m_holdPoint.SetAltitude(abortAltitude);


                    HFLogger::logMessage("ABORT LAND New hold pos set");


                    m_craft.SendSetPositionTargetGlobalInt(m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());



                    // Put the craft in guided (alternate)
                    HFLogger::logMessage("ABORT LAND - Forcing mode to alternate guided");
                    while (m_craft.Data.rcChannels[RC_MODE] != 1300)
                    {
                        // Check for kill craft command
                        if (CheckForKillCommand() == true)
                        {
                            UpdateState(killCraft);
                            return;
                        }
                        m_craft.SendRcOverridesLow(1500, 1500, 65535, 65535, 1300, 65535, 65535, 65535);
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }


                    // Put the craft in guided
                    HFLogger::logMessage("ABORT LAND - Forcing mode to guided");
                    while (m_craft.Data.rcChannels[RC_MODE] != 2000)
                    {
                        // Check for kill craft command
                        if (CheckForKillCommand() == true)
                        {
                            UpdateState(killCraft);
                            return;
                        }

                        m_craft.SendRcOverridesLow(1500, 1500, 65535, 65535, 2000, 65535, 65535, 65535);
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }



                    m_craft.SendSetPositionTargetGlobalInt(m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());


                    UpdateState(flightHold);
                    return;
                }

            }
        }




        bool motorsLow = true;
        if (m_craft.Data.servoOutputs[0] > 1200)
          motorsLow = false;
        if (m_craft.Data.servoOutputs[1] > 1200)
          motorsLow = false;
        if (m_craft.Data.servoOutputs[2] > 1200)
          motorsLow = false;
        if (m_craft.Data.servoOutputs[3] > 1200)
          motorsLow = false;

        if (motorsLow == true)
        {
            motorsLowCount++;
        }
        else
        {
            motorsLowCount = 0;
        }

        if (motorsLowCount > 10)
        {
            HFLogger::logMessage("Motors low - disarming");
            m_craft.SendDisarm();
        }

        if (m_craft.Data.armed == false)
        {
            HFLogger::logMessage("Disarmed - count: %d", craftDisarmedCount);
            craftDisarmedCount++;
            if (craftDisarmedCount > 5)
            {
                if (m_craft.Data.heartbeatCount > minHeartbeatCount)
                {
                    HFLogger::logMessage("Disarmed count is good");
                    craftDisarmed = true;
                }
            }
        }
        else
        {
            craftDisarmedCount = 0;
            minHeartbeatCount = m_craft.Data.heartbeatCount + 5;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }


    // Craft is now disarmed

    // mark flight as ended
    m_flightStarted = false;

    // Force craft into land mode
    HFLogger::logMessage("Forcing mode to land");
    while (m_craft.Data.rcChannels[RC_MODE] != 1000)
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        if ((m_precisionLanding == true) && (m_emergencyLand == false))
        {
            m_craft.SendRcOverridesLow(0, 0, 65535, 65535, 1000, 65535, 0, 65535);
        }
        else
        {
            m_craft.SendRcOverridesLow(1500, 1500, 65535, 65535, 1000, 65535, 1100, 65535);
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Wait up to 5 seconds for craft to enter land mode
    int timeout = 0;
    while ((m_craft.Data.custom_mode != flightmode_land) && (timeout < 5))
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        timeout++;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Put the craft in stabilize
    HFLogger::logMessage("Forcing mode to stabilize");
    while (m_craft.Data.rcChannels[RC_MODE] != 1420)
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        if ((m_precisionLanding == true) && (m_emergencyLand == false))
        {
            m_craft.SendRcOverridesLow(0, 0, 65535, 65535, 1420, 65535, 0, 65535);
        }
        else
        {
            m_craft.SendRcOverridesLow(1500, 1500, 65535, 65535, 1420, 65535, 1100, 65535);
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

     // Wait up to 5 seconds for craft to enter stabilize mode
    timeout = 0;
    while ((m_craft.Data.custom_mode != flightmode_stabilize) && (timeout < 5))
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        timeout++;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Put the craft in guided
    HFLogger::logMessage("Forcing mode to guided");
    while (m_craft.Data.rcChannels[RC_MODE] != 2000)
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        if ((m_precisionLanding == true) && (m_emergencyLand == false))
        {
            m_craft.SendRcOverridesLow(0, 0, 65535, 65535, 2000, 65535, 0, 65535);
        }
        else
        {
            m_craft.SendRcOverridesLow(1500, 1500, 65535, 65535, 2000, 65535, 1100, 65535);
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

     // Wait up to 5 seconds for craft to enter guided mode
    timeout = 0;
    while ((m_craft.Data.custom_mode != flightmode_guided) && (timeout < 5))
    {
        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }

        timeout++;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }


    // Reset the ground barometer offset measurement
    m_groundBaroDataValid = false;

    UpdateState(readyToArm);


}// end HandleLanding

void FlightManagementSystem::HandleError()
{
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Check for kill craft command
        if (CheckForKillCommand() == true)
        {
            UpdateState(killCraft);
            return;
        }
    }
}// end HandleError




void FlightManagementSystem::VerifyConnections(bool reconnect)
{
    if (m_controllerConnectionMade == true)
        VerifyControllerConnection(reconnect);
    if (m_craftConnectionMade == true)
        VerifyCraftConnection(reconnect);
    if (m_groundGpsConnectionMade == true)
        VerifyGroundGpsConnection(reconnect);
}// end VerifyConnections


bool FlightManagementSystem::VerifyControllerConnection(bool reconnect)
{

    bool connectionActive = m_controller.IsConnectionActive(5);

    if (connectionActive == false)
    {
        HFLogger::logMessage("Controller connection not active");
        if (reconnect == true)
        {
            HFLogger::logMessage("Controller reconnecting");
            connectionActive = m_communicationsHub.AttemptConnectToController();
        }
    }

    if (connectionActive == true)
        m_controllerConnectionState = connected;
    else
        m_controllerConnectionState = lostConnection;

    return connectionActive;
}// end VerifyControllerConnection


bool FlightManagementSystem::VerifyCraftConnection(bool reconnect)
{
    bool connectionActive = m_craft.IsConnectionActive(5);


    if (connectionActive == false)
    {
        HFLogger::logMessage("Craft connection not active");
        if (reconnect == true)
        {
            HFLogger::logMessage("Craft reconnecting");
            connectionActive = m_communicationsHub.AttemptConnectToCraft();
        }
    }


    if (connectionActive == true)
        m_craftConnectionState = connected;
    else
        m_craftConnectionState = lostConnection;

    return connectionActive;
}// end VerifyCraftConnection


bool FlightManagementSystem::VerifyGroundGpsConnection(bool reconnect)
{
    bool connectionActive = m_groundGPS.IsConnectionActive(5);


    if (connectionActive == false)
    {
        HFLogger::logMessage("GroundGPS connection not active");
        if (reconnect == true)
        {
            HFLogger::logMessage("GroundGPS reconnecting");
            connectionActive = m_communicationsHub.AttemptConnectToGroundGps();
        }
    }


    if (connectionActive == true)
        m_groundGpsConnectionState = connected;
    else
        m_groundGpsConnectionState = lostConnection;

    return connectionActive;
}// end VerifyGroundGpsConnection


void FlightManagementSystem::HeartbeatLoop()
{
    while (m_runHeartbeatLoop == true)
    {
        // Send a heartbeat to each connected device
        if(m_demoMode == false)
        {
            m_communicationsHub.SendHeartbeat(m_systemState, m_craft.Data.custom_mode);
        }
        else
        {
            if(m_systemState == demoMode)
            {
                m_communicationsHub.SendHeartbeat(flightHold, m_craft.Data.custom_mode);
            }
            else
            {
                m_communicationsHub.SendHeartbeat(m_systemState, m_craft.Data.custom_mode);
            }
        }

        // Send param requests to craft
        char paramId[16];
        strncpy(paramId, "STAT_FLTTIME", 16);
        m_craft.SendParamRequestRead(paramId);
        strncpy(paramId, "STAT_BOOTCNT", 16);
        m_craft.SendParamRequestRead(paramId);
        strncpy(paramId, "STAT_RUNTIME", 16);
        m_craft.SendParamRequestRead(paramId);
        strncpy(paramId, "STAT_RESET", 16);
        m_craft.SendParamRequestRead(paramId);

        if (m_baroCompensate == true)
        {
            // See if we are in a mode that requires compensation
            bool requireBaroCompensation = false;
            switch (m_systemState)
            {
                case readyToArm:
                case armed:
                case launch:
                case flightHold:
                case flightTranslate:
                case flightFollow:
                case land:
                    requireBaroCompensation = true;
                    break;
                default:
                    requireBaroCompensation = false;
                    break;
            }

            if (requireBaroCompensation == true)
            {
                bool groundBaroHealthy = true;
                if (m_groundBaro.Data.getConsecutiveErrorCount() > 10)
                {
                    HFLogger::logMessage("Ground Barometer has had over 10 consecutive errors");
                    groundBaroHealthy = false;
                }

                if (m_groundBaro.IsConnectionActive(30) == false)
                {
                    HFLogger::logMessage("Ground Barometer data is over 30 seconds old");
                    groundBaroHealthy = false;
                }


                if (groundBaroHealthy == false)
                {
                    m_groundBaroDataValid = false;
                }


                float groundPressure = m_groundBaro.Data.pressAbs;

                float deltaPressure = groundPressure - m_groundStartPressure;


                float newCraftBase = m_craftStartPressure + deltaPressure;
                float newCraftBase2 = m_craftStartPressure2 + deltaPressure;
                float newCraftBase3 = m_craftStartPressure3 + deltaPressure;


                float deltaDeltaPressure = deltaPressure - m_lastDeltaPressure;
                if (deltaDeltaPressure < 0)
                {
                    deltaDeltaPressure *= -1.0;
                }
                if (deltaDeltaPressure > 2.0f)
                {
                    m_groundBaroDataValid = false;

                    HFLogger::logMessage("Ground Baro Delta Bad - Last: %f,  Current: %f,  DeltaDelta: %f", m_lastDeltaPressure, deltaPressure, deltaDeltaPressure);
                }


                m_lastDeltaPressure = deltaPressure;







                if (m_groundBaroDataValid == true)
                {
//                    HFLogger::logMessage("Baro Update - gnd: %f  delta: %f  craft: %f", m_groundBaro.Data.pressAbs, deltaPressure, newCraftBase);
//                    m_craft.UpdateBarometerBase(newCraftBase, m_groundBaro.Data.temperature);

//                    HFLogger::logMessage("Baro Update - gnd: %f  delta: %f  craft: %f  craft2: %f", m_groundBaro.Data.pressAbs, deltaPressure, newCraftBase, newCraftBase2);
                    HFLogger::logMessage("Baro Comp - Gnd: %f  GndBase: %f  Delta: %f  C1Base: %f  C2Base: %f  C3Base: %f  C1Comp: %f  C2Comp: %f  C3Comp: %f", groundPressure, m_groundStartPressure, deltaPressure, m_craftStartPressure, m_craftStartPressure2, m_craftStartPressure3, newCraftBase, newCraftBase2, newCraftBase3);
                    m_craft.UpdateBarometerBase3(newCraftBase, newCraftBase2, newCraftBase3, m_groundBaro.Data.temperature);
                }
            }

        }

        // Sleep 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}// end HeartbeatLoop


uint32_t FlightManagementSystem::zeroAltIfDisarmedInt(uint32_t alt)
{
    uint32_t modifiedAlt;
    
    switch (m_systemState)
    {
        case armed:
        case launch:
        case flightHold:
        case flightTranslate:
        case flightFollow:
        case land:
        case demoMode:
            modifiedAlt = alt;
            break;
            
        default:
            modifiedAlt = 0;
            break;
    }
    
    return modifiedAlt;
}


float FlightManagementSystem::zeroAltIfDisarmedFloat(float alt)
{
    float modifiedAlt;
    
    switch (m_systemState)
    {
        case armed:
        case launch:
        case flightHold:
        case flightTranslate:
        case flightFollow:
        case land:
        case demoMode:
            modifiedAlt = alt;
            break;
            
        default:
            modifiedAlt = 0.0f;
            break;
    }
    
    return modifiedAlt;    
}



void FlightManagementSystem::StandardDataStream()
{
    // Send craft Global Position data to all controllers and observers
    m_communicationsHub.SendCraftGlobalPositionInt(m_craft.Data.latitude, m_craft.Data.longitude, zeroAltIfDisarmedInt(m_craft.Data.relative_alt), m_craft.Data.heading);

    // Send craft GPS state to all controllers and observer
    uint8_t craftGpsGood = 0;
    if (m_craft.Data.IsGpsGood() == true)
        craftGpsGood = 1;
    else
        craftGpsGood = 0;

    int numSatellites;

    if (m_craft.Data.gpsNumberSatellites2 > m_craft.Data.gpsNumberSatellites)
    {
        numSatellites = m_craft.Data.gpsNumberSatellites2;
    }
    else
    {
        numSatellites = m_craft.Data.gpsNumberSatellites;
    }

    m_communicationsHub.SendCraftGpsStatus(craftGpsGood, numSatellites, m_craft.Data.gpsHdop, m_craft.Data.gpsVdop);

    // Send RC Channel data to all controllers and observers
    m_communicationsHub.SendRcChannels(m_craft.Data.rcChannels);

    // Send Craft SysStatus to all controllers and observers
    m_communicationsHub.SendCraftSystemStatus(m_craft.Data.batteryVoltage, m_craft.Data.batteryCurrent);

    // Send ground Global Position data to all controllers and observers
    m_communicationsHub.SendGroundGlobalPositionInt(m_groundGPS.Data.latitude, m_groundGPS.Data.longitude, zeroAltIfDisarmedInt(m_groundGPS.Data.relative_alt), m_groundGPS.Data.heading);

    // Send ground GPS state to all controllers and observer
    uint8_t groundGpsGood = 0;
    if (m_groundGPS.Data.IsGpsGood() == true)
        groundGpsGood = 1;
    else
        groundGpsGood = 0;
    m_communicationsHub.SendGroundGpsStatus(groundGpsGood, m_groundGPS.Data.gpsNumberSatellites, m_groundGPS.Data.gpsHdop, m_groundGPS.Data.gpsVdop);

    // Send target Global Position data to all controllers and observers
    m_communicationsHub.SendTargetGlobalPositionInt(m_craft.Data.targetLatitude(), m_craft.Data.targetLongitude(), 0, 0);

    // Send rangefinder data
    m_communicationsHub.SendRangefinder(m_craft.Data.rangefinderDistance);

    // Send tether length
    m_communicationsHub.SendTetherLength(m_skyBox.Data.m_reelData.tetherPos);

    // Send reel status 1
    m_communicationsHub.SendReelStatus1(m_skyBox.Data.m_reelData.state, m_skyBox.Data.m_reelData.faultMask, m_skyBox.Data.m_reelData.errorMask, m_skyBox.Data.m_reelData.tetherPos, m_skyBox.Data.m_reelData.tetherWarning, m_skyBox.Data.m_reelData.motorOn);
    
    // Send reel status 2
    m_communicationsHub.SendReelStatus2(m_skyBox.Data.m_reelData.voltTension, m_skyBox.Data.m_reelData.volt12, m_skyBox.Data.m_reelData.voltMotor, m_skyBox.Data.m_reelData.currMotor, m_skyBox.Data.m_reelData.temp, m_skyBox.Data.m_reelData.tempIr);


    // Send camera zoom level
    m_communicationsHub.SendCameraZoomScale(currentZoomLevel);

    // Send IR color palette
    m_communicationsHub.SendIrColorPalette(currentColorPalette);

    ///////////////////////////////////
    //////  DJB - HACK FOR TARDEC /////
    if (m_skyBox.Data.IsLockerOpen() == true)
    {
        // Force skybox status to rollTop = 2 (open), scissorLift = 0 (up), moveState = 10 (done), timeRemaining = 0
        m_communicationsHub.SendSkyBoxStatus(2, 0, 10, 0);                
    }
    else
    {
        if (m_skyBox.Data.IsLockerClosed() == true)
        {
            // Force skybox status to rollTop = 0 (closed), scissorLift = 2 (down), moveState = 10 (done), timeRemaining = 0
            m_communicationsHub.SendSkyBoxStatus(0, 2, 10, 0);            
        }
        else
        {
            // Force skybox status to rollTop = 1 (moving), scissorLift = 1 (moving), moveState = 41 (opening), timeRemaining = 5
            m_communicationsHub.SendSkyBoxStatus(1, 1, 41, 5);                       
        }
    }
    
    //////  // Send the SkyBox state
    //////  m_communicationsHub.SendSkyBoxStatus(m_skyBox.Data.rollTopPosition, m_skyBox.Data.scissorLiftPosition, m_skyBox.Data.moveState, m_skyBox.Data.moveTimeRemaining);
        
    
    ////// DJB - END HACK FOR TARDEC ////
    /////////////////////////////////////


    // Send the Locker state
    m_communicationsHub.SendLockerStatus(m_skyBox.Data.lockerPosition);

    // Send gimbal status messages
    m_gimbalStatus.SetGimbalPosition(m_craft.Data.latitude, m_craft.Data.longitude, zeroAltIfDisarmedInt(m_craft.Data.relative_alt), m_craft.Data.heading, m_craft.Data.rcChannels[7]);
    m_gimbalStatus.SetGimbalZoom(currentZoomLevel);
    m_communicationsHub.SendGimbalStatus001(m_gimbalStatus.m_latitude, m_gimbalStatus.m_longitude, zeroAltIfDisarmedFloat(m_gimbalStatus.m_relativeAltitude), m_gimbalStatus.m_heading, m_gimbalStatus.m_tiltAngle);
    m_communicationsHub.SendGimbalStatus002(m_gimbalStatus.m_hFoVcurrent, m_gimbalStatus.m_vFoVcurrent, m_gimbalStatus.m_hFoVfull, m_gimbalStatus.m_vFoVfull, m_gimbalStatus.m_magFactor);

    m_communicationsHub.SendFlightStats(m_craft.Data.totalBootCount, m_craft.Data.totalFlightTime, m_craft.Data.totalRunTime, m_craft.Data.timeSinceReset);

    // Send wind messages
    m_communicationsHub.SendWind(bguData.windSpeed, bguData.windHeading, bguData.windBearing);
    
    // Send flight limits
    m_communicationsHub.SendFlightLimits(m_holdPoint.GetFlightCeiling(), m_holdPoint.GetPowerLimitCeiling(), m_precisionLanding, IsATSHealthy(), m_controller.Data.GpsOn);
    
    // Send craft flight status
    m_communicationsHub.SendFlightStatus(craftInitiatedLanding);
    
    
        
                            
            
    // Send BGU status
    int atsStatus;
    if (bguData.atsHealth == 0)
    {
        atsStatus = 0;
    }
    else
    {
        if ((m_precisionLanding == true) && (m_emergencyLand == false))
        {
            atsStatus = 1;
        }
        else 
        {
            atsStatus = 2;
        }
    }
    
    int32_t gpsOn = IsGpsOn() ? 1 : 0;
    m_communicationsHub.SendBguStatus(atsStatus, m_controller.Data.GpsOn, gpsOn);

    // Send the flight time
    int32_t milliseconds = 0;
    if (m_flightStarted == true)
    {
        time_t currentTime;
        time(&currentTime);
        milliseconds = (int32_t) (difftime(currentTime, m_flightStartTime) * 1000);
    }
//        m_communicationsHub.SendSystemTime(0, microseconds);
    m_communicationsHub.SendSystemTime(0, milliseconds);

}// end StandardDataStream

void FlightManagementSystem::DemoDataStream()
{
    // Send craft Global Position data to all controllers and observers
    m_communicationsHub.SendCraftGlobalPositionInt(m_demoCraftData.latitude, m_demoCraftData.longitude, m_demoCraftData.relative_alt, m_demoCraftData.heading);

    // Send craft GPS state to all controllers and observer
    uint8_t craftGpsGood = 0;
    if (m_demoCraftData.IsGpsGood() == true)
        craftGpsGood = 1;
    else
        craftGpsGood = 0;
    m_communicationsHub.SendCraftGpsStatus(craftGpsGood, m_demoCraftData.gpsNumberSatellites, m_demoCraftData.gpsHdop, m_demoCraftData.gpsVdop);

    // Send RC Channel data to all controllers and observers
    m_communicationsHub.SendRcChannels(m_craft.Data.rcChannels);

    // Send Craft SysStatus to all controllers and observers
    m_communicationsHub.SendCraftSystemStatus(m_craft.Data.batteryVoltage, m_craft.Data.batteryCurrent);



    // Send target Global Position data to all controllers and observers
    m_communicationsHub.SendTargetGlobalPositionInt(m_demoCraftData.targetLatitude(), m_demoCraftData.targetLongitude(), 0, 0);

    // Send the flight time
    int32_t milliseconds = 0;
    if (m_flightStarted == true)
    {
        time_t currentTime;
        time(&currentTime);
        milliseconds = (int32_t) (difftime(currentTime, m_flightStartTime) * 1000);
    }
//        m_communicationsHub.SendSystemTime(0, microseconds);
    m_communicationsHub.SendSystemTime(0, milliseconds);

}// end DemoDataStream


void FlightManagementSystem::ControllerDataStreamLoop()
{
    HFLogger::logMessage("Starting controller data stream");
    while (m_runControllerDataStreamLoop == true)
    {

        if (m_demoMode == false)
        {
            StandardDataStream();
        }
        else
        {
            DemoDataStream();
        }


        //std::this_thread::sleep_for(std::chrono::seconds(1));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));    // update to 10Hz
    }
}// end ControllerDataStreamLoop

int FlightManagementSystem::ApplyDeadband(int input, int deadband)
{
  if (input > 0)
  {
    if (input < deadband)
    {
      input = 0;
    }
    else
    {
      input = input - deadband;
    }
  }
  else
  {
    if (input > -deadband)
    {
        input = 0;
    }
    else
    {
        input = input + deadband;
    }
  }

  return input;
}// end ApplyDeadband

uint16_t FlightManagementSystem::AddSawtoothStep(uint16_t base, uint16_t maxDev)
{
    // Adds 5 position sawtooth signal of amplitide maxDev on top of base value.
    // Each call increments the value of the added signal.
    
    uint16_t result = base;
    
    switch (sawtoothStepCount % 5)
    {
        case 0: result = base; break;
        case 1: result = base + (maxDev / 2); break;
        case 2: result = base + maxDev; break;
        case 3: result = base - maxDev; break;
        case 4: result = base - (maxDev / 2); break;
        default: result = base; break;  // should never get here!!!
    }
    sawtoothStepCount++;
    return result;
}// end AddSawtoothStep

void FlightManagementSystem::UpdateCameraCraftPosition(int16_t joyX, int16_t joyY, int16_t joyZ, bool joyButton, float roiLat, float roiLon, float roiAlt, bool newRoi)
{

    uint16_t rollOutput = 1500;;
    uint16_t pitchOutput = 1500;
    uint16_t yawOutput = 1500;
    uint16_t panOutput = 1500;
    uint16_t zoomOutput = 1500;
    uint16_t tiltOutput = 1000;

    uint16_t modeOutput = 2000;
    uint16_t throttleOutput = 1000;

    uint16_t zoomLevel = 1500; // copy of zoomOutput for continuous zoom
    
    // Roll output
    rollOutput = 1500 - joyX;

    // Pitch output
    pitchOutput = 1500 + joyY;




    // Yaw output
    yawOutput = 1500 - joyX;

    panOutput = 1500 - joyX;

    if (joyButton == true)
        yawOutput = 1500;
    else
        panOutput = 1500;

    int scaledX = joyX;
    int scaledY = joyY;

    scaledX = ApplyDeadband(scaledX, 100);
    scaledY = ApplyDeadband(scaledY, 100);


    // Pan / Yaw
    if (m_yawFollowsPan == true)
    {
        if (joyButton == true)
        {
            scaledX /= 2;
        }
        panOutput = 1500 + (scaledX * 500) / 2048;
        yawOutput = 1500;
    }
    else
    {
        if (joyButton == true)
        {
            scaledX /= 2;
            panOutput = 1500 + (scaledX * 500) / 2048;
            yawOutput = 1500;
        }
        else
        {
            panOutput = 1500;
            yawOutput = 1500 + (scaledX * 100) / 2048;
        }
    }


    // Tilt angle

    if (joyButton == true)
    {
        scaledY /=4;
    }

    int camMotion = 0;
    camMotion = -(4500 * scaledY) / 2048;




    // Zoom output
    if (joyZ < -200)
    {
        zoomOutput = 1200;
    }
    else
    {
        if (joyZ > 200)
        {
            zoomOutput = 1800;
        }
        else
        {
            zoomOutput = 1500;
        }
    }
    zoomLevel = zoomOutput;

    // Add sawtooth ripple on zoom rc channel
    zoomOutput = AddSawtoothStep(zoomOutput,20);
    // Don't add jitter lest it not match the 'case'
    // zoomLevel = zoomOutput

    bool sendOverride = false;
    bool adjustTilt = false;

    bool sendRoi = false;

    //// Landing Retry Redux
    //   Land case separated from default group.
    //   Free overrides of mode and throttle for flightHold, fligthFollow, Land          
    //// Landing Retry Redux - end
    switch (m_systemState)
    {
        case acquiringCraftGPS:
        case acquiringControllerGPS:
        case readyToArm:
        case demoMode:
            rollOutput = 1500;
            pitchOutput = 1500;
            yawOutput = 1500;
            modeOutput = 2000;
            throttleOutput = 1000;
            adjustTilt = true;
            sendOverride = true;
            break;

        case flightHold:
            rollOutput = 1500;
            pitchOutput = 1500;
            if (m_yawFollowsPan == true)
            {
                yawOutput = 0;
            }
            modeOutput = 0;
            throttleOutput = 0;            
            adjustTilt = true;
            sendOverride = true;
            sendRoi = true;
            break;

        case flightTranslate:
            if (m_yawFollowsPan == true)
            {
                yawOutput = 0;
            }
            panOutput = 1500;
            modeOutput = 1700;
            throttleOutput = 1500;
            adjustTilt = false;
            sendOverride = true;
            sendRoi = true;
            break;

        case flightFollow:
            if (m_yawFollowsPan == true)
            {
                yawOutput = 0;
            }
            rollOutput = 1500;
            pitchOutput = 1500;
            modeOutput = 0;
            throttleOutput = 0;
            adjustTilt = true;
            sendOverride = true;
            sendRoi = true;
            break;

        case killCraft:
            rollOutput = 1500;
            pitchOutput = 1500;
            yawOutput = 1500;
            modeOutput = 1420;
            throttleOutput = 1000;
            sendOverride = true;
            break;

        case land:
            rollOutput = 1500;  // freed later iff precision land enabled
            pitchOutput = 1500;  // freed later iff precision land enabled
            yawOutput = 1500;
            panOutput = 1500;
            tiltOutput = 1000;
            modeOutput = 0;
            throttleOutput = 0;
            sendOverride = true;
            adjustTilt = false;
            break;
        
        case notSet:
        case startup:
        case connectToController:
        case connectToCraft:
        case connectToGroundGPS:
        case arming:
        case armed:
        case launch:
        case failsafe:
        case safetyStart:
        case lowVoltage:
        case error:
        default:
            rollOutput = 1500;
            pitchOutput = 1500;
            yawOutput = 1500;
            panOutput = 1500;
            tiltOutput = 1000;
            modeOutput = 2000;
            throttleOutput = 1000;
            sendOverride = false;
            adjustTilt = false;
            break;
    }
    


    if (sendRoi == true)
    {
        if (newRoi == true)
        {
            HFLogger::logMessage("ROI: %f %f %f", roiLat, roiLon, roiAlt);
            m_craft.SendRegionOfInterest(roiLat, roiLon, roiAlt);
        }
    }

    if (adjustTilt == true)
    {
        m_craft.AdjustGimbalTiltAngle(camMotion);
    }
    tiltOutput = m_craft.GetGimbalTiltOutput();


    // If precision land is enabled, release roll and pitch
    if ((m_precisionLanding == true) && (m_emergencyLand == false))
    {
        rollOutput = 0;
        pitchOutput = 0;
        zoomOutput = 0;
    }
    else
    {
        zoomOutput = 1100;
    }
    // Don't match these as they are not zoom states
    // zoomLevel = zoomOutput
        
    // If landing retry or precision land is disabled, override mode and throttle
    if ((m_landingRetryEnabled == false) || (m_precisionLanding == false) || (m_emergencyLand == true))
    {
        modeOutput = 2000;
        throttleOutput = 1000;
    }

                
    
    // Check for yaw/pan limits in GPS denied
    if (m_gpsDeniedSupport == true) 
    {
        int panMax = 1500 + m_gpsDeniedPanSlewMaxRate;
        int panMin = 1500 - m_gpsDeniedPanSlewMaxRate;
        int yawMax = 1500 + m_gpsDeniedYawSlewMaxRate;
        int yawMin = 1500 - m_gpsDeniedYawSlewMaxRate;
        
        if (IsGpsOn() == false) 
        {
            HFLogger::logMessage("GPS-DENIED: GPS is off pMax:%d, pMin:%d, yMax:%d, yMin:%d", panMax, panMin, yawMax, yawMin);
            if (m_yawFollowsPan == true)
            {
                if (panOutput > panMax)
                    panOutput = panMax;
                if (panOutput < panMin)
                    panOutput = panMin;
            }
            else
            {
                if (yawOutput > yawMax)
                    yawOutput = yawMax;
                if (yawOutput < yawMin)
                    yawOutput = yawMin;
            }
        }
    }
    

    if (sendOverride == true)
    {
        // Send the RC commands to the craft
        // HFLogger::logMessage("RC OVERRIDE - p: %d  r: %d", rollOutput, pitchOutput);
        m_craft.SendRcOverridesLow(rollOutput, pitchOutput, throttleOutput, yawOutput, modeOutput, panOutput, zoomOutput, tiltOutput);
        
        // Send http zoom commands (if configured)
        if (m_httpZoom == true)
        {
            //switch (zoomOutput)
            switch (zoomLevel)
            {
                case 1500:
                    if (m_zoomState != 0)
                    {
                        m_zoomState = 0;
                        // Send Zoom Stop
                        //PyRun_SimpleString("response = urllib.urlopen(zoom_stop)\n");
                        HttpZoomStop();
                        HFLogger::logMessage("Zoom Stop");
                    }
                    break;

                case 1200:
                    if (m_zoomState != -1)
                    {
                        m_zoomState = -1;
                        // Send Zoom In
                        //PyRun_SimpleString("response = urllib.urlopen(zoom_out_start)\n");
                        HttpZoomOut();
                        HFLogger::logMessage("Zoom In");
                    }
                    break;

                case 1800:
                    if (m_zoomState != 1)
                    {
                        m_zoomState = 1;
                        // Send Zoom Out
                        //PyRun_SimpleString("response = urllib.urlopen(zoom_in_start)\n");
                        HttpZoomIn();
                        HFLogger::logMessage("Zoom Out");
                    }
                    break;
            }

        }


    }
}// end UpdateCameraCraftPosition

void FlightManagementSystem::HttpZoomIn()
{
    if (m_availableCameras[0] == true)
	{
		Py_Initialize();
		PyRun_SimpleString("import urllib\n");
		PyRun_SimpleString(
			"try:\n" \
			"    urllib.urlopen('http://10.20.30.40/cgi-bin/fwptzctr.cgi?FwModId=0&PortId=0&PtzCode=267&PtzParm=10&RcvData=NO&FwCgiVer=0x0001')\n" \
			"except:\n" \
			"    pass\n" \
			);
		Py_Finalize();
	}
	if (m_availableCameras[2] == true)
	{
		Py_Initialize();
		PyRun_SimpleString("import urllib\n");
		PyRun_SimpleString(
			"try:\n" \
			"    urllib.urlopen('http://admin:admin@10.20.30.42/services/io.ion?action=ptz&source=videoinput_1&command=zoom&subcommand=instart&zoom=35')\n" \
			"except:\n" \
			"    pass\n" \
			);
		Py_Finalize();
	}
	if (m_availableCameras[3] == true)
	{
		// get the current zoom level (in case it was changed elsewhere?)
		m_zoomLevel = HttpGetZoom43Pos();
		// Start zooming all the way in
		m_zoomTarget = 100;
			// zooming done in continuousZoomThread
	}
}// end HttpZoomIn

void FlightManagementSystem::HttpZoomStop()
{
    if (m_availableCameras[0] == true)
	{
		Py_Initialize();
		PyRun_SimpleString("import urllib\n");
		PyRun_SimpleString(
			"try:\n" \
			"    urllib.urlopen('http://10.20.30.40/cgi-bin/fwptzctr.cgi?FwModId=0&PortId=0&PtzCode=523&PtzParm=10&RcvData=NO&FwCgiVer=0x0001')\n" \
			"except:\n" \
			"    pass\n" \
			);
		Py_Finalize();
	}
	if (m_availableCameras[2] == true)
	{
		Py_Initialize();
		PyRun_SimpleString("import urllib\n");
		PyRun_SimpleString(
			"try:\n" \
			"    urllib.urlopen('http://admin:admin@10.20.30.42/services/io.ion?action=ptz&source=videoinput_1&command=zoom&subcommand=instop')\n" \
			"except:\n" \
			"    pass\n" \
			);
		Py_Finalize();
	}
	if (m_availableCameras[3] == true)
	{
		// Stop zooming
		//HFLogger::logMessage("   - *** ZOOM STOP ****  %d (%d)", m_zoomTarget, m_zoomLevel)
		//HFLogger::logMessage("   - *** ZOOM STOP calling GetZoomPos ****  %d (%d)", m_zoomTarget, m_zoomLevel);
		m_zoomLevel = HttpGetZoom43Pos();
		//HFLogger::logMessage("   - *** ZOOM STOP equating zooms ****  %d (%d)", m_zoomTarget, m_zoomLevel);
		m_zoomTarget = m_zoomLevel;
		//HFLogger::logMessage("   - *** ZOOM STOP zooming to new target ****  %d (%d)", m_zoomTarget, m_zoomLevel);
		HttpZoomTo43();
	}

}// end HttpZoomStop


void FlightManagementSystem::HttpZoomOut()
{
    if (m_availableCameras[0] == true)
	{
		Py_Initialize();
		PyRun_SimpleString("import urllib\n");
		PyRun_SimpleString(
			"try:\n" \
			"    urllib.urlopen('http://10.20.30.40/cgi-bin/fwptzctr.cgi?FwModId=0&PortId=0&PtzCode=268&PtzParm=10&RcvData=NO&FwCgiVer=0x0001')\n" \
			"except:\n" \
			"    pass\n" \
			);
		Py_Finalize();
	}
	if (m_availableCameras[2] == true)
	{
		Py_Initialize();
		PyRun_SimpleString("import urllib\n");
		PyRun_SimpleString(
			"try:\n" \
			"    urllib.urlopen('http://admin:admin@10.20.30.42/services/io.ion?action=ptz&source=videoinput_1&command=zoom&subcommand=outstart&zoom=35')\n" \
			"except:\n" \
			"    pass\n" \
			);
		Py_Finalize();
	}
	if (m_availableCameras[3] == true)
	{
		// get the current zoom level (in case it was changed elsewhere?)
		m_zoomLevel = HttpGetZoom43Pos();
		// Start zooming all the way out
		m_zoomTarget = 0;
			// zooming done in continuousZoomThread
	}

}// end HttpZoomOut


void FlightManagementSystem::DisplayLoop()
{
    cout << "Display thread start\n";
    try
    {
        m_display.AttachController(&m_controller, &m_controllerConnectionState);
        m_display.AttachCraft(&m_craft, &m_craftConnectionState);
        m_display.AttachGroundGps(&m_groundGPS, &m_groundGpsConnectionState);
        m_display.AttachGroundBaro(&m_groundBaro, &m_groundBaroConnectionState);
        m_display.AttachCommunicationsHub(&m_communicationsHub);

        m_display.Initialize(&m_systemState);

        while (m_runDisplayLoop == true)
        {
            m_display.Update();
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
    }
    catch (const std::exception& e)
    {
        cout << "Display thread exception\n";
        HFLogger::logMessage("EXCEPTION - %s", e.what());
    }
}// end DisplayLoop

int FlightManagementSystem::FindAllCameras()
{
	int port = 0;
	char ipAddr[20];
	int camCount = 0;

	if (m_availableCameras[0] == false)
	{
		// check for Sony FCB-7100
		port = 80;
		strncpy(ipAddr, "10.20.30.40", 20);
		if (FindSingleCamera(port, ipAddr) == 1)
		{
			m_availableCameras[0] = true;
			camCount++;
		}
	}

	if (m_availableCameras[1] == false)
	{
		// check for FLIR Vue
		port = 80;
		strncpy(ipAddr, "10.20.30.41", 20);
		if (FindSingleCamera(port, ipAddr) == 1)
		{
			m_availableCameras[1] = true;
			camCount++;
		}
	}

	if (m_availableCameras[2] == false)
	{
		// check for KTnC & FLIR Boson (both on ION Atomas Mini Dual)
		port = 80;
		strncpy(ipAddr, "10.20.30.42", 20);
		if (FindSingleCamera(port, ipAddr) == 1)
		{
			m_availableCameras[2] = true;
			camCount++;
		}
	}

	if (m_availableCameras[3] == false)
	{
		// check for fixed mount Sony FPV cam (on ION Atomas Micro)
		port = 80;
		strncpy(ipAddr, "10.20.30.43", 20);
		if (FindSingleCamera(port, ipAddr) == 1)
		{
			m_availableCameras[3] = true;
			camCount++;
		}
	}

	// That's it for now


	return camCount;
}// end FindAllCameras

int FlightManagementSystem::FindSingleCamera(int port, char *ip)
{
	int retVal = 0;

	if (ip == NULL)
		return 0;

    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in addr = { AF_INET, htons(port), inet_addr(ip) };

    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(timeout));

    if (connect(sockfd, (struct sockaddr *) &addr, sizeof(addr)) == 0)
    {
        retVal = 1;
    }

    close(sockfd);

	return retVal;
}// end FindSingleCamera

void FlightManagementSystem::CameraFinderLoop()
{
	int camIndex = 0;
	int camFinderLoops = 0;
	for (camIndex=0; camIndex<10; camIndex++)
	{
		m_availableCameras[camIndex] = false;
	}

    bool firstCurl = true;
    
	while (m_runCameraFinderLoop == true)
	{
		if (m_cameraFinderStart == true)
		{
			HFLogger::logMessage("Checking camera types");
			FindAllCameras();

			for (camIndex=0; camIndex<10; camIndex++)
			{
				if (m_availableCameras[camIndex] == true)
				{
					HFLogger::logMessage("    Found camera 10.20.30.4%d", camIndex);
				}
                
                if (m_availableCameras[2] == true)  // for now only do this for Ionodes MiniDual with KT&C and Boson
                {
                    if (firstCurl)
                    {
                        // Force cURL library loading/initialization by getting the current IR color palette
                        HFLogger::logMessage("cURL first use: call GetColorPalette()");
                        int colorPalette = GetColorPalette();
                        HFLogger::logMessage("cURL first use: current palette = %d", colorPalette);
                        firstCurl = false;
                    }
                }
			}
			camFinderLoops++;	// only increment if we are actually looking
		}
		else
		{
			// Just not looking yet...
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(10000));
		if (camFinderLoops > 12)
		{
			m_runCameraFinderLoop = false;
			m_cameraFinderStart = false;
			HFLogger::logMessage("Stopped looking for cameras.  Final list:");
			for (camIndex=0; camIndex<10; camIndex++)
			{
				if (m_availableCameras[camIndex] == true)
				{
					HFLogger::logMessage("    Found camera 10.20.30.4%d", camIndex);
				}
			}
		}
	}
}// end CameraFinderLoop

void FlightManagementSystem::ContinuousZoomLoop()
{

    while (m_runContinuousZoomLoop == true)
    {
		// Adjust the desired zoom level
		//HFLogger::logMessage("   - ZOOMtar ZOOMlev Status %d : %d)", m_zoomTarget, m_zoomLevel);

		if (m_zoomTarget == m_zoomLevel)
		{
			// Done zooming
			// do nothing more
			//HFLogger::logMessage("   - ZOOM idle");
		}
		else
		{
			// Set new zoom target position
			m_zoomLevel += (m_zoomTarget > m_zoomLevel) ? 5 : -5;
			if (m_zoomLevel > 100)
			{
				m_zoomLevel = 100;
			}
			if (m_zoomLevel < 0)
			{
				m_zoomLevel = 0;
			}

			// Zoom to new position
			//HFLogger::logMessage("   - New zoomLevel %d (%d)", m_zoomTarget, m_zoomLevel);
			HttpZoomTo43();
		}

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}// end ContinuousZoomLoop

void FlightManagementSystem::StepZoomLoop()
{

    while (m_runStepZoomLoop == true)
    {
        if (m_stepZoomOpticalTarget1 == m_stepZoomOpticalLevel1)
        {
            // Camera1 optical is zoomed
            // Do nothing more
        }
        else
        {
            // Set camera1 optical zoom to new position
            m_stepZoomOpticalLevel1 = m_stepZoomOpticalTarget1;
            SetZoomOptical(1, m_stepZoomOpticalTarget1);
        }

        if (m_stepZoomDigitalTarget1 == m_stepZoomDigitalLevel1)
        {
            // Camera1 digital is zoomed
            // Do nothing more
        }
        else
        {
            // Set camera1 digital zoom to new position
            m_stepZoomDigitalLevel1 = m_stepZoomDigitalTarget1;
            SetZoomDigital(1, m_stepZoomDigitalTarget1);
        }

        if (m_stepZoomOpticalTarget2 == m_stepZoomOpticalLevel2)
        {
            // Camera2 optical is zoomed
            // Do nothing more
        }
        else
        {
            // Set camera2 optical zoom to new position
            m_stepZoomOpticalLevel2 = m_stepZoomOpticalTarget2;
            SetZoomOptical(2, m_stepZoomOpticalTarget2);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

} // end StepZoomLoop

size_t FlightManagementSystem::curl_res2chararray(void *ptr, size_t size, size_t nmemb, void *stream)
{
	int i=0;
	for(i=0;i<(int)(size*nmemb); i++)
	{
//		buffer[i] = ((char*)ptr)[i];
		((char*)stream)[i] = ((char*)ptr)[i];
	}
	((char*)stream)[i++] = '\0';
	((char*)stream)[i++] = '\0';

	// curl expects a return of the number of bytes written.
	// if != size of block that ptr points to, then an error
	// occured (wrote less than total... or somehow more?)
	// thus we just return the exact size of the block
	return nmemb*size;
}// end curl_res2chararray

void FlightManagementSystem::HttpZoomTo43()
{
	//HFLogger::logMessage("   - *** Starting HttpZoomTo43 ****  %d (%d)", m_zoomTarget, m_zoomLevel);

	memset(curl_buffer, 0x00, sizeof(curl_buffer));

	char postthis[255];
	struct curl_slist *hs=NULL;
	CURLcode res;
	CURL *curl;
	curl = curl_easy_init();

	sprintf(postthis,"<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n<configuration.ion>\n<setparams>\nvideoinput_1.sensor.currentzoomlevel=%d\n</setparams></configuration.ion>", m_zoomLevel);

	if(curl) {
		curl_easy_setopt(curl, CURLOPT_URL, "http://admin:admin@10.20.30.43/services/configuration.ion?action=setparams&format=text");
		/* example.com is redirected, so we tell libcurl to follow redirection */
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_res2chararray);
		curl_easy_setopt(curl, CURLOPT_WRITEDATA, &curl_buffer);

		hs = curl_slist_append(hs, "Content-Type: text/xml");
		curl_easy_setopt(curl, CURLOPT_HTTPHEADER, hs);
		curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postthis);

		/* Perform the request, res will get the return code */
		res = curl_easy_perform(curl);
		/* Check for errors */
		if(res != CURLE_OK) { fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res)); }
 		/* always cleanup */
		curl_easy_cleanup(curl);
	}
	//HFLogger::logMessage("   - *** Ending HttpZoomTo43 ****  %d (%d) (%d)", m_zoomTarget, m_zoomLevel);

}// end HttpZoomTo43

int FlightManagementSystem::HttpGetZoom43Pos()
{
	//HFLogger::logMessage("   - *** Starting HttpGetZoom43Pos ****  %d (%d)", m_zoomTarget, m_zoomLevel);
	memset(curl_buffer, 0x00, sizeof(curl_buffer));

	CURL *curl;
	CURLcode res;

	curl = curl_easy_init();
	if(curl)
	{
		curl_easy_setopt(curl, CURLOPT_URL, "http://admin:admin@10.20.30.43/services/configuration.ion?sel=paramlist&params=videoinput_1:0.sensor.currentzoomlevel");
		/* example.com is redirected, so we tell libcurl to follow redirection */
		//curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_res2chararray);
		curl_easy_setopt(curl, CURLOPT_WRITEDATA, &curl_buffer);

		/* Perform the request, res will get the return code */
		res = curl_easy_perform(curl);
		/* Check for errors */
		if(res != CURLE_OK) { fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res)); }
 		/* always cleanup */
		curl_easy_cleanup(curl);
	}
	char *matchBuffer = NULL;
	char match[] = "\"currentzoomlevel\">";
	int matchLen = strlen(match);
	int zoomLevel = 0;
	matchBuffer = strstr(curl_buffer, match);
	if (matchBuffer != NULL)
	{
		zoomLevel = atoi(matchBuffer+matchLen);
	}
	//HFLogger::logMessage("   - *** Ending HttpGetZoom43Pos ****  %d (%d)", m_zoomTarget, m_zoomLevel);

	return zoomLevel;
}// end HttpGetZoom43Pos



//void FlightManagementSystem::ZoomControl(int camera, int command)
void FlightManagementSystem::ZoomControl(int command)
{
    int newZoomLevel;

    int maxZoomLevel;
    if (m_disableDigitalZoom == true)
    {
        maxZoomLevel = 100;
    }
    else
    {
        maxZoomLevel = 200;
    }

    switch (command)
    {
        case 1:
            newZoomLevel = currentZoomLevel + 20;
            if (newZoomLevel > maxZoomLevel)
            {
                newZoomLevel = maxZoomLevel;
            }
            break;

        case -1:
            newZoomLevel = currentZoomLevel - 20;
            if (newZoomLevel < 0)
            {
                newZoomLevel = 0;
            }
            break;

        default:
            newZoomLevel = currentZoomLevel;
            break;
    }

    currentZoomLevel = newZoomLevel;

    // Visible camera
    SetZoom(1, currentZoomLevel);
    // IR Camera
    SetZoom(2, currentZoomLevel);
}


void FlightManagementSystem::ZoomScaleControl(int zoomLevel)
{
    int maxZoomLevel;
    if (m_disableDigitalZoom == true)
    {
        maxZoomLevel = 100;
    }
    else
    {
        maxZoomLevel = 200;
    }

    if (zoomLevel < 0)
        zoomLevel = 0;
    if (zoomLevel > maxZoomLevel)
        zoomLevel = maxZoomLevel;

    currentZoomLevel = zoomLevel;

    SetZoom(1, currentZoomLevel);
    SetZoom(2, currentZoomLevel);


    int oZoom, dZoom;

    oZoom = GetOpticalZoomLevel();
    dZoom = GetDigitalZoomLevel();

    HFLogger::logMessage("ZOOMLEVEL %d %d  %d", zoomLevel, oZoom, dZoom);
}

void FlightManagementSystem::SetZoom(int camera, int zoomLevel)
{
    HFLogger::logMessage("SetZoom - cam: %d  level: %d", camera, zoomLevel);

    if (camera == 1)
    {
        if (zoomLevel > 100)
        {
            //SetZoomOptical(1, 100);
            //SetZoomDigital(1, zoomLevel - 100);

            m_stepZoomOpticalTarget1 = 100;
            m_stepZoomDigitalTarget1 = zoomLevel - 100;
        }
        else
        {
            //SetZoomOptical(1, zoomLevel);
            //SetZoomDigital(1, 0);

            m_stepZoomOpticalTarget1 = zoomLevel;
            m_stepZoomDigitalTarget1 = 0;
        }
    }
    else
    {
        if (zoomLevel > 100)
        {
            //SetZoomOptical(2, 100);
            m_stepZoomOpticalTarget2 = 100;
        }
        else
        {
            //SetZoomOptical(2, zoomLevel);
            m_stepZoomOpticalTarget2 = zoomLevel;
        }
    }
}

void FlightManagementSystem::SetZoomOptical(int camera, int zoomLevel)
{
    HFLogger::logMessage("SetOpticalZoom - cam: %d  level: %d", camera, zoomLevel);

    memset(curl_buffer, 0x00, sizeof(curl_buffer));

    char postthis[255];
    struct curl_slist *hs=NULL;
    CURLcode res;
    CURL *curl;
    curl = curl_easy_init();

    int zoom = zoomLevel;

    if (curl)
    {
        sprintf(postthis, "<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n<configuration.ion>\n<setparams>\nvideoinput_%d.sensor.currentzoomlevel=%d\n</setparams></configuration.ion>", camera, zoom);
        curl_easy_setopt(curl, CURLOPT_URL, "http://admin:admin@10.20.30.42/services/configuration.ion?action=setparams&format=text");
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_res2chararray);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &curl_buffer);

        hs = curl_slist_append(hs, "Content-Type: text/xml");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, hs);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postthis);

        res = curl_easy_perform(curl);
        if(res != CURLE_OK)
        {
            HFLogger::logMessage("CURL ERROR: %s", curl_easy_strerror(res));
        }
        curl_easy_cleanup(curl);

    }

}


int FlightManagementSystem::GetOpticalZoomLevel()
{
	//HFLogger::logMessage("   - *** Starting GetZoomLevel ****  ");
	memset(curl_buffer, 0x00, sizeof(curl_buffer));

	CURL *curl;
	CURLcode res;

	curl = curl_easy_init();
	if(curl)
	{
		curl_easy_setopt(curl, CURLOPT_URL, "http://admin:admin@10.20.30.42/services/configuration.ion?sel=paramlist&params=videoinput_1:0.sensor.currentzoomlevel");
		/* example.com is redirected, so we tell libcurl to follow redirection */
		//curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_res2chararray);
		curl_easy_setopt(curl, CURLOPT_WRITEDATA, &curl_buffer);

		/* Perform the request, res will get the return code */
		res = curl_easy_perform(curl);
		/* Check for errors */
		if(res != CURLE_OK) { fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res)); }
 		/* always cleanup */
		curl_easy_cleanup(curl);
	}
	char *matchBuffer = NULL;
	char match[] = "\"currentzoomlevel\">";
	int matchLen = strlen(match);
	int zoomLevel = 0;
	matchBuffer = strstr(curl_buffer, match);
	if (matchBuffer != NULL)
	{
		zoomLevel = atoi(matchBuffer+matchLen);
	}
	//HFLogger::logMessage("   - *** Ending GetZoomLevel **** ");

	return zoomLevel;
}// end GetZoomLevel


void FlightManagementSystem::ColorPaletteControl(int command)
{
    int newPalette;


    switch (command)
    {
      case 0:
        SetColorPalette(lastColorPalette);
        break;

      case 1:
        newPalette = currentColorPalette + 1;
        if (newPalette > 110)
        {
            newPalette = 101;
        }
        SetColorPalette(newPalette);
        break;


      case -1:
        newPalette = currentColorPalette - 1;
        if (newPalette < 101)
        {
            newPalette = 110;
        }
        SetColorPalette(newPalette);
        break;


      default:
        SetColorPalette(command);
        break;
    }
}




void FlightManagementSystem::SetColorPalette(int colorPalette)
{
    lastColorPalette = currentColorPalette;
    currentColorPalette = colorPalette;

    memset(curl_buffer, 0x00, sizeof(curl_buffer));

    char postthis[255];
    struct curl_slist *hs=NULL;
    CURLcode res;
    CURL *curl;
    curl = curl_easy_init();


    char colorName[20];
    switch(colorPalette)
    {
        case 101: strcpy(colorName, "whitehot");  break;
        case 102: strcpy(colorName, "blackhot");  break;
        case 103: strcpy(colorName, "rainbow");  break;
        case 104: strcpy(colorName, "rainbowhc");  break;
        case 105: strcpy(colorName, "ironbow");  break;
        case 106: strcpy(colorName, "lava");  break;
        case 107: strcpy(colorName, "arctic");  break;
        case 108: strcpy(colorName, "globow");  break;
        case 109: strcpy(colorName, "gradefire");  break;
        case 110: strcpy(colorName, "hottest");  break;
        default:  strcpy(colorName, "whitehot");  break;
    }

    if (curl)
    {
        sprintf(postthis, "<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n<configuration.ion>\n<setparams>\nvideoinput_2.sensor.colorlut=%s\n</setparams></configuration.ion>", colorName);
        curl_easy_setopt(curl, CURLOPT_URL, "http://admin:admin@10.20.30.42/services/configuration.ion?action=setparams&format=text");
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_res2chararray);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &curl_buffer);

        hs = curl_slist_append(hs, "Content-Type: text/xml");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, hs);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postthis);

        res = curl_easy_perform(curl);
        if(res != CURLE_OK)
        {
            HFLogger::logMessage("CURL ERROR: %s", curl_easy_strerror(res));
        }
        curl_easy_cleanup(curl);
    }
    else
    {
        HFLogger::logMessage("====> no curl");
    }
}

int FlightManagementSystem::GetColorPalette()
{
	//HFLogger::logMessage("   - *** Starting GetColorPalette ****  ");
	memset(curl_buffer, 0x00, sizeof(curl_buffer));

	CURL *curl;
	CURLcode res;

	curl = curl_easy_init();
	if(curl)
	{
		curl_easy_setopt(curl, CURLOPT_URL, "http://admin:admin@10.20.30.42/services/configuration.ion?sel=paramlist&params=videoinput_2:0.sensor.colorlut");
		/* example.com is redirected, so we tell libcurl to follow redirection */
		//curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_res2chararray);
		curl_easy_setopt(curl, CURLOPT_WRITEDATA, &curl_buffer);

		/* Perform the request, res will get the return code */
		res = curl_easy_perform(curl);
		/* Check for errors */
		if(res != CURLE_OK) { fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res)); }
 		/* always cleanup */
		curl_easy_cleanup(curl);
	}
	char *matchBuffer = NULL;
	char match[] = "\"colorlut\">";
	int matchLen = strlen(match);
	int colorLut = 0;
	matchBuffer = strstr(curl_buffer, match);
	if (matchBuffer != NULL)
	{
		colorLut = atoi(matchBuffer+matchLen);
	}
	//HFLogger::logMessage("   - *** Ending GetColorPalette **** ");

	return colorLut;
}// end GetColorPalette

int FlightManagementSystem::GetDigitalZoomLevel()
{
	//HFLogger::logMessage("   - *** Starting GetDigitalZoomLevel ****  ");
	memset(curl_buffer, 0x00, sizeof(curl_buffer));

	CURL *curl;
	CURLcode res;

	curl = curl_easy_init();
	if(curl)
	{
		curl_easy_setopt(curl, CURLOPT_URL, "http://admin:admin@10.20.30.42/services/configuration.ion?sel=paramlist&params=videoinput_1:0.sensor.currentdigitalzoomlevel");
		/* example.com is redirected, so we tell libcurl to follow redirection */
		//curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_res2chararray);
		curl_easy_setopt(curl, CURLOPT_WRITEDATA, &curl_buffer);

		/* Perform the request, res will get the return code */
		res = curl_easy_perform(curl);
		/* Check for errors */
		if(res != CURLE_OK) { fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res)); }
 		/* always cleanup */
		curl_easy_cleanup(curl);
	}
	char *matchBuffer = NULL;
	char match[] = "\"currentdigitalzoomlevel\">";
	int matchLen = strlen(match);
	int zoomLevelDigital = 0;
	matchBuffer = strstr(curl_buffer, match);
	if (matchBuffer != NULL)
	{
		zoomLevelDigital = atoi(matchBuffer+matchLen);
	}
	//HFLogger::logMessage("   - *** Ending GetDigitalZoomLevel **** ");

	return zoomLevelDigital;
}// end GetDigitalZoomLevel


void FlightManagementSystem::SetZoomDigital(int camera, int zoomLevelDigital)
{
    HFLogger::logMessage("SetDigitalZoom - cam: %d  level: %d", camera, zoomLevelDigital);

    memset(curl_buffer, 0x00, sizeof(curl_buffer));

    char postthis[255];
    struct curl_slist *hs=NULL;
    CURLcode res;
    CURL *curl;
    curl = curl_easy_init();

    int zoom = zoomLevelDigital;

    if (curl)
    {
        sprintf(postthis, "<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n<configuration.ion>\n<setparams>\nvideoinput_%d.sensor.currentdigitalzoomlevel=%d\n</setparams></configuration.ion>", camera, zoom);
        curl_easy_setopt(curl, CURLOPT_URL, "http://admin:admin@10.20.30.42/services/configuration.ion?action=setparams&format=text");
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_res2chararray);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &curl_buffer);

        hs = curl_slist_append(hs, "Content-Type: text/xml");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, hs);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postthis);

        res = curl_easy_perform(curl);
        if(res != CURLE_OK)
        {
            HFLogger::logMessage("CURL ERROR: %s", curl_easy_strerror(res));
        }
        curl_easy_cleanup(curl);

    }

}// end SetZoomDigital

void FlightManagementSystem::InitGimbalStatus()
{
    int length = 0;
    int result = 0;
    char myFile[256];

    //strcpy(myFile, "hfCamera.xbc-kz10.json");
    strcpy(myFile, "/home/pi/hti/apps/fms/hfCamera.xbc-kz10.json");
    length = strlen(myFile);
    result = m_gimbalStatus.LoadCamera(myFile, length);

    if (result == 0)
    {
        HFLogger::logMessage("HF Camera file loaded successfully! ('%s')", myFile);
    }
    else
    {
        HFLogger::logMessage("HF Camera file FAILED to load. ('%s')", myFile);
    }
}// end InitGimbalStatus






bool FlightManagementSystem::SlewToTkHeading(uint16_t rcChannels[])
{
    // Returns true if we need to exit landing (ABORT / KILL_CRAFT)
    // A failure due to timeout or will still return false and allow landing to continue.
    // Skipping also returns false

    if ((m_slewToTkHeadingFirstTime == false) || (m_emergencyLand == true) || (m_abortPL == true))
    {
        HFLogger::logMessage("Slew to TK Heading - SKIPPING; Hdg=%03.2f", m_craft.Data.heading/100.0f);
        HFLogger::logMessage("Slew to TK Heading -  FirstTime=%d, Emergency=%d, abortPL=%d", m_slewToTkHeadingFirstTime, m_emergencyLand, m_abortPL);
        m_slewToTkHeadingFirstTime = false;
        return false;
    }
    

    m_slewToTkHeadingFirstTime = false;

    float tempHeading;
    
    float craftLandStartHeading;
    float tkLandStartHeading;
    float nominalTargetHeading;
    float closestTargetHeading;
    bool clockwise;
    

    HFLogger::logMessage("Slew to TK Heading - Craft Init=%03.2f, Craft Realign=%03.2f, Delta=%03.2f", landHeadingCraftInitial, landHeadingCraftAfterRealign, landHeadingCraftDelta);
    
    
    craftLandStartHeading = m_craft.Data.heading / 100.0f;
    tkLandStartHeading = m_groundGPS.Data.heading / 100.0f;
    float tkAlignedHeading;
    //tkAlignedHeading = tkStartHeading + landHeadingCraftDelta;
    tkAlignedHeading = tkLandStartHeading;
   
    HFLogger::logMessage("Slew to TK Heading - Craft at=%03.2f, TK at=%03.2f, TK-aligned=%03.2f", craftLandStartHeading, tkLandStartHeading, tkAlignedHeading);
    
    nominalTargetHeading = tkAlignedHeading + m_tkHeadingTarget;
    if (nominalTargetHeading > 360) 
        nominalTargetHeading -= 360;
    if (nominalTargetHeading < 0)
        nominalTargetHeading += 360;

    HFLogger::logMessage("Slew to TK Heading - nominalTargetHeading=%03.2f", nominalTargetHeading);
    
    float delta = nominalTargetHeading - craftLandStartHeading;
    if (delta > 180)
        delta -= 360;
    if (delta < -180)
        delta += 360;
      
    
    HFLogger::logMessage("Slew to TK Heading - Initial Delta = %03.2f", delta);
        
    
    // Only one point:
    if (delta > 0)
    {
        while (delta > 180)
            delta -= 360;
    }
    else
    {
        while (delta < -180)
            delta += 360;
    }
    
    
        
    
    // Find direction
    if (delta > 0)
    {
        clockwise = true;
    }
    else
    {
        clockwise = false;
    }
    
    HFLogger::logMessage("Slew to TK Heading - Final delta = %03.2f", delta);
    if (clockwise == true)
        HFLogger::logMessage("Slew to TK Heading - clockwise");
    else
        HFLogger::logMessage("Slew to TK Heading - counter-clockwise");
    
    // Get target
    closestTargetHeading = craftLandStartHeading + delta;
    if (closestTargetHeading > 360)
        closestTargetHeading -= 360;
    if (closestTargetHeading < 0)
        closestTargetHeading += 360;
        
   
    HFLogger::logMessage("Slew to TK Heading - Start");
    HFLogger::logMessage("Slew to TK Heading - Craft Start Heading = %03.2f", craftLandStartHeading);
    HFLogger::logMessage("Slew to TK Heading - TK Start Heading = %03.2f", tkLandStartHeading);
    HFLogger::logMessage("Slew to TK Heading - Nominal Target = %03.2f", nominalTargetHeading);
    HFLogger::logMessage("Slew to TK Heading - Closest Target = %03.2f", closestTargetHeading);
    if (clockwise == true)
    {
        HFLogger::logMessage("Slew to TK Heading - Clockwise");
    }
    else    
    {
        HFLogger::logMessage("Slew to TK Heading - Counter-clockwise");
    }        
    
    
    // When this function is called, craft should already be set to the rc values that are passed in
    // (and hopefully craft is moving toward those positions now, like camera level, etc)

    float target = closestTargetHeading;
    float deadband = 5;    // = 5 deg

    HFLogger::logMessage("Slew to TK Heading - TargetHdg=%03.2f", target);

    HFLogger::logMessage("Slew to TK Heading - Yaw Follows Pan = %d", m_yawFollowsPan);
    HFLogger::logMessage("Slew to TK Heading - Yaw Rate = %d", m_tkHeadingYawSlewRate);
    HFLogger::logMessage("Slew to TK Heading - Pan Rate = %d", m_tkHeadingPanSlewRate);
    
    int baseRate;
    if (m_yawFollowsPan == true)
        baseRate = m_tkHeadingPanSlewRate;
    else
        baseRate = m_tkHeadingYawSlewRate;
    
    HFLogger::logMessage("Slew to TK Heading - Base Rate = %d", baseRate);
        
    int controlDelta;
    if (clockwise == true)
        controlDelta = baseRate;
    else
        controlDelta = -baseRate;
    
    // start a slow pan
    if (m_yawFollowsPan == true)
    {
        // gimbal pans and tells craft to yaw
        rcChannels[RC_YAW] = 0;
        rcChannels[RC_PAN] = 1500 + controlDelta;
    }
    else
    {
        // craft yaws and gimbal follows
        rcChannels[RC_YAW] = 1500 + controlDelta;
        rcChannels[RC_PAN] = 1500;
    }
    
    HFLogger::logMessage("Slew to TK Heading - RC_YAW = %d. RC_PAN = %d", rcChannels[RC_YAW], rcChannels[RC_PAN]);
    

    bool done = false;


    float currentCraftHeading;    
    float currentError;
    
    currentCraftHeading = m_craft.Data.heading / 100.0f;    
    currentError = currentCraftHeading - target;
    if (currentError > 180)
        currentError -= 360;
    if (currentError < -180)
        currentError += 360;
        
    HFLogger::logMessage("Slew to TK Heading - craft=%03.2f, target=%03.2f, error=%03.2f, deadband=%03.2f", currentCraftHeading, target, currentError, deadband);
    // Check if we are already in the deadband
    if (fabs(currentError) < deadband)
    {
        HFLogger::logMessage("Slew to TK Heading - In deadband");
        done = true;
    }
    else
    {
           // Set RC channels to slew craft
        m_craft.SendRcOverridesLow(rcChannels);
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    int whileLoops = 0;

    while (!done)
    {

        // Look for a command
        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            m_controller.Data.ClearCommandQueue();

            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                return true;
            }

            // Look for a change altitude command
            if (cmd->CommandType() == HFCMD_CHANGE_ALT)
            {
                
                // Stop yawing / panning    
                if (m_yawFollowsPan == true)
                {
                    // gimbal pans and tells craft to yaw
                    rcChannels[RC_YAW] = 0;
                    rcChannels[RC_PAN] = 1500;
                }
                else
                {
                    // craft yaws and gimbal follows
                    rcChannels[RC_YAW] = 1500;
                    rcChannels[RC_PAN] = 1500;
                }
                m_craft.SendRcOverridesLow(rcChannels);

    
                    
                // Verify that we can abort landing
                bool inLowVoltage = false;
                int minVoltage = 22000;


                if (m_bigSky == false) {
                    minVoltage = 22000;
                } else {
                    minVoltage = 44000;
                }

                if (m_craft.Data.batteryVoltage < minVoltage)
                {
                    inLowVoltage = true;
                }

                if (inLowVoltage == false)
                {

                    // Attempt to abort landing

                    // If abortPL is set to true, set this as an emergency landing
                    if (m_abortPL == true)
                    {
                        m_emergencyLand = true;
                    }

                    // Regardless of command, set the target altitude to the current altitude
                    float abortAltitude = (float) m_minAltitude / 100.0f;
                    abortAltitude = m_craft.Data.relative_alt / 1000.0f;

                    HFLogger::logMessage("ABORT LAND - Change altitude absolute : %f", abortAltitude);
                    m_holdPoint.SetAltitude(abortAltitude);


                    HFLogger::logMessage("ABORT LAND New hold pos set");



                    // Put the craft in land
                    HFLogger::logMessage("ABORT LAND - Forcing mode to alternate guided");
                    while (m_craft.Data.rcChannels[RC_MODE] != 1300)
                    {
                        // Check for kill craft command
                        if (CheckForKillCommand() == true)
                        {
                            UpdateState(killCraft);
                            return true;
                        }

                        // Stop yawing / panning    
                        if (m_yawFollowsPan == true)
                        {
                            m_craft.SendRcOverridesLow(1500, 1500, 65535, 65535, 1300, 1500, 65535, 65535);
                        }
                        else
                        {
                            m_craft.SendRcOverridesLow(1500, 1500, 65535, 1500, 1300, 1500, 65535, 65535);
                        }
 

                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }


                    // Put the craft in guided
                    HFLogger::logMessage("ABORT LAND - Forcing mode to guided");
                    while (m_craft.Data.rcChannels[RC_MODE] != 2000)
                    {
                        // Check for kill craft command
                        if (CheckForKillCommand() == true)
                        {
                            UpdateState(killCraft);
                            return true;
                        }

                        
                        if (m_yawFollowsPan == true)
                        {
                            m_craft.SendRcOverridesLow(1500, 1500, 65535, 65535, 2000, 1500, 65535, 65535);
                        }
                        else
                        {
                            m_craft.SendRcOverridesLow(1500, 1500, 65535, 1500, 2000, 1500, 65535, 65535);
                        }                        

                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }



                    if (m_useV35Plus == true)
                    {
                        m_craft.SendSetPositionTargetGlobalInt(m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                    }
                    else
                    {
                        m_craft.SendMissionItem(m_holdPoint.CraftHoldLatitude(), m_holdPoint.CraftHoldLongitude(), m_holdPoint.CraftTargetAltitude());
                    }

                    UpdateState(flightHold);
                    return true;
                }

            }
        }


        // Resend slew command (why not?)
        m_craft.SendRcOverridesLow(rcChannels);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        currentCraftHeading = m_craft.Data.heading / 100.0f;    
        currentError = currentCraftHeading - target;
        if (currentError > 180)
            currentError -= 360;
        if (currentError < -180)
            currentError += 360;


        HFLogger::logMessage("Slew to TK Heading - craft=%03.2f, target=%03.2f, error=%03.2f, deadband=%03.2f", currentCraftHeading, target, currentError, deadband);
        // Check if we are already in the deadband
        if (fabs(currentError) < deadband)
        {
            HFLogger::logMessage("Slew to TK Heading - In deadband");
            done = true;
        }

        whileLoops++;
        if (whileLoops > 350)  // 35 seconds
        {
            done = true;
        }

        HFLogger::logMessage("Slew to TK Heading - Slewing; Hdg=%03.2f", m_craft.Data.heading/100.0f);

    }// end while waiting for desired heading

    if (whileLoops > 350)
    {
        HFLogger::logMessage("Slew to TK Heading - Timeout; Hdg=%03.2f", m_craft.Data.heading/100.0f);
    }
    else
    {
        HFLogger::logMessage("Slew to TK Heading - Complete");
    }

    // stop slewing
    if (m_yawFollowsPan == true)
    {
        // gimbal pans and tells craft to yaw
        rcChannels[RC_YAW] = 1500;
        rcChannels[RC_PAN] = 1500;
    }
    else
    {
        // craft yaws and gimbal follows
        rcChannels[RC_YAW] = 1500;
        rcChannels[RC_PAN] = 1500;
    }
    m_craft.SendRcOverridesLow(rcChannels);

    HFLogger::logMessage("Slew to TK Heading - End; Hdg=%03.2f", m_craft.Data.heading/100.0f);

    return false;
}// end SlewToTKHeading()









bool FlightManagementSystem::SlewToLandHeading(uint16_t rcChannels[])
{
    // Returns false if landing stops (KillCraft or Halt)
    // A failure due to timeout or will still return true and allow landing to continue.
    // Skipping also returns true

    if (m_slewToLandHeadingFirstTime == false)
    {
        HFLogger::logMessage("Slew to Land Heading - SKIPPING; Hdg=%03.2f", m_craft.Data.heading/100.0f);
        return true;
    }

    m_slewToLandHeadingFirstTime = false;

    // TESTING
    // only do this for 10m < alt < 20m
    if ((m_craft.Data.relative_alt < 10000) || (m_craft.Data.relative_alt > 20000))
    {
        HFLogger::logMessage("Slew to Land Heading - ALT not in range; Hdg=%03.2f", m_craft.Data.heading/100.0f);
        return true;
    }

    HFLogger::logMessage("Slew to Land Heading - Start; Hdg=%03.2f", m_craft.Data.heading/100.0f);

    // When this function is called, craft should already be set to the rc values that are passed in
    // (and hopefully craft is moving toward those positions now, like camera level, etc)

    uint16_t target = 9000;     // = 90.00 deg = due East
    uint16_t deadband = 500;    // = 5 deg

    m_landingHeadingTarget = 9000;
    target = m_landingHeadingTarget;
    HFLogger::logMessage("Slew to Land Heading - TargetHdg=%03.2f", target/100.0f);

     // start a slow pan clockwise
    if (m_yawFollowsPan == true)
    {
        // gimbal pans and tells craft to yaw
        rcChannels[RC_YAW] = 0;
        rcChannels[RC_PAN] = 1500 + 25;
    }
    else
    {
        // craft yaws and gimbal follows
        rcChannels[RC_YAW] = 1500 + 25;
        rcChannels[RC_PAN] = 1500;
    }

    bool done = false;

    if (abs(m_craft.Data.heading - target) < deadband)
    {
        done = true;
    }
    else
    {
           // Set RC channels to slew craft
        m_craft.SendRcOverridesLow(rcChannels);
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    int whileLoops = 0;

    while (!done)
    {

        // Look for a command
        HFCommand* cmd = m_controller.Data.GetFirstCommand();
        if (cmd != nullptr)
        {
            m_controller.Data.ClearCommandQueue();

            // Look for a kill craft command
            if (cmd->CommandType() == HFCMD_KILL_CRAFT)
            {
                UpdateState(killCraft);
                return false;
            }

            // Look for a change altitude command
            if (cmd->CommandType() == HFCMD_CHANGE_ALT)
            {
                // Verify that we can abort landing
                bool inLowVoltage = false;
                int minVoltage = 22000;


                if (m_bigSky == false) {
                    minVoltage = 22000;
                } else {
                    minVoltage = 44000;
                }

                if (m_craft.Data.batteryVoltage < minVoltage)
                {
                    inLowVoltage = true;
                }

                if (inLowVoltage == false)
                {

                    // Attempt to abort landing

                    // If abortPL is set to true, set this as an emergency landing
                    if (m_abortPL == true)
                    {
                        m_emergencyLand = true;
                    }

                    // Regardless of command, set the target altitude to the current altitude
                    float abortAltitude = (float) m_minAltitude / 100.0f;
                    abortAltitude = m_craft.Data.relative_alt / 1000.0f;

                    HFLogger::logMessage("ABORT LAND - Change altitude absolute : %f", abortAltitude);
                    m_holdPoint.SetAltitude(abortAltitude);


                    HFLogger::logMessage("ABORT LAND New hold pos set");



                    // Put the craft in land
                    HFLogger::logMessage("ABORT LAND - Forcing mode to alternate guided");
                    while (m_craft.Data.rcChannels[RC_MODE] != 1300)
                    {
                        // Check for kill craft command
                        if (CheckForKillCommand() == true)
                        {
                            UpdateState(killCraft);
                            return false;
                        }

                        m_craft.SendRcOverridesLow(1500, 1500, 65535, 65535, 1300, 65535, 65535, 65535);

                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }


                    // Put the craft in guided
                    HFLogger::logMessage("ABORT LAND - Forcing mode to guided");
                    while (m_craft.Data.rcChannels[RC_MODE] != 2000)
                    {
                        // Check for kill craft command
                        if (CheckForKillCommand() == true)
                        {
                            UpdateState(killCraft);
                            return false;
                        }

                        m_craft.SendRcOverridesLow(1500, 1500, 65535, 65535, 2000, 65535, 65535, 65535);

                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }



                    if (m_useV35Plus == true)
                    {
                        m_craft.SendSetPositionTargetGlobalInt(m_holdPoint.CraftHoldLatitudeInt(), m_holdPoint.CraftHoldLongitudeInt(), m_holdPoint.CraftTargetAltitude());
                    }
                    else
                    {
                        m_craft.SendMissionItem(m_holdPoint.CraftHoldLatitude(), m_holdPoint.CraftHoldLongitude(), m_holdPoint.CraftTargetAltitude());
                    }

                    UpdateState(flightHold);
                    return false;
                }

            }
        }


        // Resend slew command (why not?)
        m_craft.SendRcOverridesLow(rcChannels);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        if (abs(m_craft.Data.heading - target) < deadband)
        {
            done = true;
        }

        whileLoops++;
        if (whileLoops > 30)  // 30 seconds
        {
            done = true;
        }

        HFLogger::logMessage("Slew to Land Heading - Slewing; Hdg=%03.2f", m_craft.Data.heading/100.0f);

    }// end while waiting for desired heading

    if (whileLoops > 120)
    {
        HFLogger::logMessage("Slew to Land Heading - Timeout; Hdg=%03.2f", m_craft.Data.heading/100.0f);
    }

    // stop slewing
    if (m_yawFollowsPan == true)
    {
        // gimbal pans and tells craft to yaw
        rcChannels[RC_YAW] = 1500;
        rcChannels[RC_PAN] = 1500;
    }
    else
    {
        // craft yaws and gimbal follows
        rcChannels[RC_YAW] = 1500;
        rcChannels[RC_PAN] = 1500;
    }
    m_craft.SendRcOverridesLow(rcChannels);

    HFLogger::logMessage("Slew to Land Heading - End; Hdg=%03.2f", m_craft.Data.heading/100.0f);

    return true;
}// end SlewToLandHeading()



int FlightManagementSystem::detectGimbal()
{
    int tiltChannel = m_craft.Data.rcChannels[RC_TILT];

    HFLogger::logMessage("GIMBAL_AUTODETECT: RC_TILT value = %d", tiltChannel);

    if ((tiltChannel > (GIMBAL_FOUND_VALUE - 50)) && (tiltChannel < (GIMBAL_FOUND_VALUE + 50)))
    {
        return GIMBAL_FOUND;
    }

    if ((tiltChannel > (GIMBAL_NOT_FOUND_VALUE - 50)) && (tiltChannel < (GIMBAL_NOT_FOUND_VALUE + 50)))
    {
        return GIMBAL_NOT_FOUND;
    }

    return GIMBAL_NOT_SET;
}


void FlightManagementSystem::ThermalLoop()
{

    HFLogger::logMessage("Thermal :: Starting thermal polling loop.");
    try
    {
        bool priorBadOpen = false;
        char thermalFilename[256] = "/home/pi/hti/apps/utilities/thermal/temperature";
        char dataLine[256] = "";
        int result = 0;
        float tempMax = 100.0;
        float tempMin = -40.0;
        float tempBad = -100.0;
        float dataTemp = tempBad;
        while (m_runThermalLoop == true)
        {
            memset(dataLine, 0x00, sizeof(dataLine));

            FILE *thermalFile = fopen(thermalFilename, "r");
            if (thermalFile != NULL)
            {
                // File contains only a single line of data with the temperature in degrees C
                result = fscanf(thermalFile, "%s\n", dataLine);
                if (result != EOF)
                {
                    dataTemp = atof(dataLine);
                    if ( (dataTemp < tempMax) && (dataTemp > tempMin) )
                    {
                        m_rpiTemperature = dataTemp;
                    }
                    else
                    {
                        m_rpiTemperature = tempBad;
                    }
                    HFLogger::logMessage("Thermal :: rpi temperature (deg C) = %2.1f", m_rpiTemperature);
                }
                fclose(thermalFile);
                priorBadOpen = false;
            }
            else // file didn't open
            {
                if (priorBadOpen == false)
                {
                    HFLogger::logMessage("Thermal :: Failed to open temperature file: '%s'", thermalFilename);
                }
                priorBadOpen = true;
            }

            // Sleep 1 second
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }// end while
   }
    catch (const std::exception& e)
    {
        cout << "Thermal thread exception\n";
        HFLogger::logMessage("EXCEPTION - %s", e.what());
    }


    cout << "End of Thermal thread.\n";
    HFLogger::logMessage("Thermal :: Exited Thermal polling loop. Thread will end.");
 }// end ThermalLoop

void FlightManagementSystem::BguMonitorLoop()
{

    HFLogger::logMessage("BGU Monitor :: Starting BGU monitor polling loop.");
    try
    {
        char reportFilename[256] = "";
        char commandFilename[256] = "";
        
        int result = 0;
        bool priorBadUpdate = false;
        
        strcpy(reportFilename, "/home/pi/hti/apps/bguLogServer/bguReport.txt");
        strcpy(commandFilename, "/home/pi/hti/apps/bguLogServer/bguCommand.txt");
        
        while (m_runBguMonitorLoop == true)
        {
            result = bguData.UpdateFromFile(reportFilename, strlen(reportFilename));
           
            if (result == 0)
            {
                //HFLogger::logMessage("BGU Monitor:: UpdateFromFile : count=%d, mlState=%d, tetherVoltage=%d, powerLossCount=%d",
                //                       bguData.count, bguData.mlState, bguData.tetherVoltage, bguData.powerLossCount);
                                        
                priorBadUpdate = false;
            }
            else // didn't update properly
            {
                if (priorBadUpdate == false)
                {
                    HFLogger::logMessage("BGU Monitor :: Failed to open/read BGU report file: '%s'", reportFilename);
                }
                priorBadUpdate = true;
            }
            
            // Output regardless to test non-update with failed read
            HFLogger::logMessage("BGU Monitor :: UpdateFromFile : count=%d, mlState=%d, tetherVoltage=%d, batteryVoltage=%d, totalCurrent=%d, powerLossCount=%d, craftRoll=%d, craftPitch=%d, craftHeading=%d, windSpeed=%d, windBearing=%d, windHeaing=%d, atsHealth=%d",
                                        bguData.count, bguData.mlState, bguData.tetherVoltage, bguData.batteryVoltage, bguData.totalCurrent, bguData.powerLossCount,
                                        bguData.craftRoll, bguData.craftPitch, bguData.craftHeading, bguData.windSpeed, bguData.windBearing, bguData.windHeading, bguData.atsHealth);
            
            
            
            // Update the bguCommand data from various sources.  Update file when changed.
            bguCommand.ledState = m_controller.Data.LedState;
            bguCommand.laserState = m_controller.Data.LaserState;
            bguCommand.tetherPos = m_skyBox.Data.m_reelData.tetherPos;
            bguCommand.gpsOn = m_controller.Data.GpsOn;
            bguCommand.gpsGuiLat = m_controller.Data.gpsDeniedDefaultLatitude;
            bguCommand.gpsGuiLon = m_controller.Data.gpsDeniedDefaultLongitude;

            if (m_precisionLanding == true)
            {
                bguCommand.corrOn = 1;
            }
            else
            {
                bguCommand.corrOn = 0;
            }
            HFLogger::logMessage("BGU Command :: UpdateFile : ledState=%d, laserState=%d, tetherPos=%d, gpsOn=%d",bguCommand.ledState, bguCommand.laserState, bguCommand.tetherPos, bguCommand.gpsOn);
            bguCommand.UpdateFile(commandFilename, strlen(commandFilename));
            
            
            // Sleep 100 milliseconds
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        }// end while
   }
    catch (const std::exception& e)
    {
        cout << "BGU Monitor thread exception\n";
        HFLogger::logMessage("EXCEPTION - %s", e.what());
    }


    cout << "End of BGU MOnitor thread.\n";
    HFLogger::logMessage("BGU Monitor :: Exited BGU Monitor polling loop. Thread will end.");
 }// end BguMonitorLoop



bool FlightManagementSystem::IsVibrationAllowable()
{
    bool allowable = true;
    if (m_craft.Data.vibrationX > m_vibrationThresholdXY)
      allowable = false;
    if (m_craft.Data.vibrationY > m_vibrationThresholdXY)
      allowable = false;
    if (m_craft.Data.vibrationZ > m_vibrationThresholdZ)
      allowable = false;      
    return allowable;
}



bool FlightManagementSystem::IsATSHealthy()
{
    // 0 is a magic number for ATS health, should be a define somewhere or a parameter
    // 1 is Healthy
    // 0 is Unhealthy
    // -1 is Unknown
    return bguData.atsHealth != 0;
}

bool FlightManagementSystem::IsGpsOn()
{
    //return bguData.gpsOn;
    return ((bguData.gps1Trust == 100) && (bguData.gps2Trust == 100));
}
