#ifndef FLIGHTMANAGEMENTSYSTEM_H
#define FLIGHTMANAGEMENTSYSTEM_H

#include "CommunicationsHub.h"
#include "GroundGPS.h"
#include "Craft.h"
#include "Controller.h"
#include "GroundBaro.h"

#include "SystemState.h"

#include "HoldPoint.h"

#include "Display.h"

#include "BoxcarFilter.h"

#include "GimbalStatus.h"

#include "BguData.h"
#include "BguCommand.h"

#include "PowerLossMonitor.h"



#define GIMBAL_NOT_SET          100
#define GIMBAL_FOUND            101
#define GIMBAL_NOT_FOUND        102

#define GIMBAL_INITIAL_VALUE    1500
#define GIMBAL_FOUND_VALUE      2000
#define GIMBAL_NOT_FOUND_VALUE  1000



class FlightManagementSystem
{
    public:
        FlightManagementSystem();
        virtual ~FlightManagementSystem();

        void Start();
        void Stop();

        bool m_precisionLanding;
        bool m_noGroundGps;
        int  m_minCraftSat;
        int  m_maxCraftHdop;
        int  m_minGroundSat;
        int  m_maxGroundHdop;
        bool m_httpZoom;
        bool m_baroCompensate;
        bool m_debugSkipGps;
        bool m_yawFollowsPan;
        int  m_maxAltitude;
        bool m_demoMode;
        bool m_useV35Plus;
        bool m_useExtendedChannels;
        bool m_requireBackupController;
        bool m_disableVoltageCheck;
        bool m_bigSky;
        bool m_abortPL;
        bool m_disableDigitalZoom;
        bool m_useSkyBox;
        bool m_useLocker;
        bool m_noLocationLogging;
        int  m_lostCommTimeout;
        bool m_usePowerLimitCeiling;
        float  m_powerLimitFloor;
        float  m_powerLimitDelta;
        int  m_powerLimitCountMax;
        float m_vibrationThresholdXY;
        float m_vibrationThresholdZ;
        int m_vibrationFilterLength;
        int m_minAltitude;
        
        bool m_landingRetryEnabled;
        bool m_landingSetHome;
        
        bool m_useControllerProxy;
        
        bool m_useAlignTkHeading;
        float m_tkHeadingTarget;
        int m_tkHeadingYawSlewRate;
        int m_tkHeadingPanSlewRate;
        
        bool m_gpsDeniedSupport;
        int m_gpsDeniedYawSlewMaxRate;
        int m_gpsDeniedPanSlewMaxRate;
        int m_gpsDeniedMaxAltitude;
        
        int m_launchClimbRate;
        int m_flightClimbRate;
        bool m_commLossLandAbort;

        bool m_gpsDeniedLandOnLoss;
        

    protected:


        CommunicationsHub m_communicationsHub;
        GroundGPS m_groundGPS;
        Craft m_craft;
        Controller m_controller;
        BackupController m_backupController;
        GroundBaro m_groundBaro;
        SkyBox m_skyBox;

        deviceConnectionState_t m_craftConnectionState;
        deviceConnectionState_t m_groundGpsConnectionState;
        deviceConnectionState_t m_controllerConnectionState;
        deviceConnectionState_t m_groundBaroConnectionState;
        deviceConnectionState_t m_skyBoxConnectionState;

        CraftData m_demoCraftData;
        void InitializeDemoMode();

        systemState_t m_systemState;


        std::thread m_stateMachineThread;
        bool m_runStateMachineLoop;
        void StateMachine();

        std::thread m_heartbeatThread;
        bool m_runHeartbeatLoop;
        void HeartbeatLoop();

        std::thread m_thermalThread;
        bool m_runThermalLoop;
        void ThermalLoop();

        std::thread m_bguMonitorThread;
        bool m_runBguMonitorLoop;
        void BguMonitorLoop();

        std::thread m_displayThread;
        bool m_runDisplayLoop;
        void DisplayLoop();

        std::thread m_continuousZoomThread;
        bool m_runContinuousZoomLoop;
        void ContinuousZoomLoop();

        std::thread m_stepZoomThread;
        bool m_runStepZoomLoop;
        void StepZoomLoop();

        std::thread m_cameraFinderThread;
        bool m_runCameraFinderLoop;
        void CameraFinderLoop();
        bool m_cameraFinderStart;


        void StepStateMachine();
        void UpdateState(systemState_t newState);

        void HandleKillCraft();
        void HandleStartup();
        void HandleConnectToController();
        void HandleConnectToCraft();
        void HandleConnectToGroundGPS();
        void HandleConnectToGroundBaro();
        void HandleConnectToSkyBox();
        void HandleAcquiringCraftGPS();
        void HandleAcquiringControllerGPS();
        void HandleReadySkyBoxClosed();
        void HandleClosingSkyBoxFromReady();
        void HandleReadyToArm();
        void HandleArming();
        void HandleArmed();
        void HandleLaunching();
        void HandleFlight();
        void HandleLanding();
        bool HandleLandingPhase1();
        void HandleLandingPhase2();
        void HandleLanding_old();
        void HandleError();
        void HandleDemoMode();

        bool CheckForKillCommand();


        void HttpZoomIn();
        void HttpZoomStop();
        void HttpZoomOut();

        void SendStatusMessage(char* msg, int len);


//        std::thread m_craftUpdateThread;
//        bool m_runCraftUpdateThread;
//        void CraftUpdateLoop();
//        bool m_suspendCraftUpdates;
//        bool m_craftUpdatesSuspended;

        std::thread m_controllerDataStreamThread;
        bool m_runControllerDataStreamLoop;
        void ControllerDataStreamLoop();
        void StandardDataStream();
        void DemoDataStream();


        void VerifyConnections(bool reconnect);
        bool VerifyControllerConnection(bool reconnect);
        bool VerifyCraftConnection(bool reconnect);
        bool VerifyGroundGpsConnection(bool reconnect);
        bool VerifyGroundBaroConnection(bool reconnect);
        bool VerifySkyBoxConnection(bool reconnect);

        bool m_controllerConnectionMade;
        bool m_craftConnectionMade;
        bool m_groundGpsConnectionMade;
        bool m_groundBaroConnectionMade;
        bool m_skyBoxConnectionMade;

        bool IsVoltageGoodToArm();
        bool IsBadBatteryVoltage();
        bool IsVoltageRangeGood();
        bool CriticalVoltageCheck();
        bool CriticalTetherVoltageCheck();
        int VerifyFlightVoltage();

        bool IsBguRcGood();

        HoldPoint m_holdPoint;

        int ApplyDeadband(int input, int deadband);

        void UpdateCameraCraftPosition(int16_t joyX, int16_t joyY, int16_t joyZ, bool joyButton, float roiLat = 0, float roiLon = 0, float roiAlt = 0, bool newRoi = false);

        Display m_display;



        bool IsAllGpsGood();

        int m_zoomState;

        bool m_useRemoteJoystick;

        //uint16_t m_rcChannels[8];

        bool IsCraftFlying();

        bool m_flightStarted;
        time_t m_flightStartTime;


        float m_craftStartPressure;
        float m_craftStartPressure2;
        float m_craftStartPressure3;
        float m_groundStartPressure;


        bool m_groundBaroDataValid;
        float m_lastDeltaPressure;


        BoxcarFilter m_baroFilter;

//// 20180516 001 PLS
		bool m_availableCameras[10];
		int FindAllCameras();
		int FindSingleCamera(int port, char *ip);

//// 20180516 001 END

		int m_zoomLevel;	// camera's current percentage of max 0-100
		int m_zoomTarget;	// desired zoom percentage

        // Camera 1's current and target step zoom levels
        int m_stepZoomOpticalLevel1;
        int m_stepZoomOpticalTarget1;
        int m_stepZoomDigitalLevel1;
        int m_stepZoomDigitalTarget1;

        // Camera 2's current and target step zoom levels
        int m_stepZoomOpticalLevel2;
        int m_stepZoomOpticalTarget2;

		char curl_buffer[1023];
		static size_t curl_res2chararray(void *ptr, size_t size, size_t nmemb, void *stream);
		void HttpZoomTo43();
		int HttpGetZoom43Pos();


        void ColorPaletteControl(int command);
        void SetColorPalette(int colorPalette);
        int GetColorPalette();
        int currentColorPalette;
        int lastColorPalette;


        void ZoomControl(int command);
        void ZoomScaleControl(int zoomLevel);
        void SetZoom(int camera, int zoomLevel);




        int GetOpticalZoomLevel();
        int GetDigitalZoomLevel();

        void SetZoomOptical(int camera, int zoomLevel);
        void SetZoomDigital(int camera, int zoomLevelDigital);

        int currentZoomLevel;


        GimbalStatus m_gimbalStatus;

/*
        int currentVisibleZoomLevelOptical;
		int currentVisibleZoomLevelDigital;

		int currentIRZoomLevelOptical;
		int currentIRZoomLevelDigital;
*/


        bool m_emergencyLand;

        void InitGimbalStatus();

        bool m_slewToLandHeadingFirstTime;
        bool m_slewToTkHeadingFirstTime;

        bool SlewToLandHeading(uint16_t rcChannels[]);
        bool SlewToTkHeading(uint16_t rcChannels[]);

        bool m_useLandingHeading;
        int m_landingHeadingTarget;


        int detectGimbal();

        float m_rpiTemperature; // in deg C

        int sawtoothStepCount;
        uint16_t AddSawtoothStep(uint16_t base, uint16_t maxDev);
        
        BguData bguData;
        BguCommand bguCommand;

        PowerLossMonitor powerLossMonitor;
        
        
        bool IsVibrationAllowable();

        /**
         * Checks the last ATS health value from the bgu log for a good value
         * @return True if the ATS is healthy, false otherwise
        */
        bool IsATSHealthy();
     
        bool IsGpsOn();        

        
        float m_yawRealignmentAltitude;
        bool SetYawResetAltitude();
        
        int countHandleAquireControllerGPS;
        int countHandleAquireCraftGPS;
        int countHandleReadyToArm;
        
        
        bool craftInitiatedLanding;
        
        bool landingSetHomeDisabledAfterHalt;
        
        bool landingFromFlightHold;
        
        uint32_t zeroAltIfDisarmedInt(uint32_t alt);
        float zeroAltIfDisarmedFloat(float alt);

        float landHeadingCraftInitial;
        float landHeadingCraftAfterRealign;
        float landHeadingCraftDelta;
        
        
        bool commReconnectCheckComplete;
        
        
        bool landingAutoAborted;
        
        
        
    private:
};

#endif // FLIGHTMANAGEMENTSYSTEM_H
