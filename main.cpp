#include <iostream>

#include <thread>

#include <fstream>

#include <algorithm>

#include "FlightManagementSystem.h"

#include "Version.h"


#include <python2.7/Python.h>

#include "HFLogger.h"



using namespace std;


bool precisionLand;
bool noGroundGps;
int minCraftSat;
int maxCraftHdop;
int minGroundSat;
int maxGroundHdop;
bool httpZoom;
bool debugSkipGPS;
bool baroCompensate;
bool yawFollowsPan;
int maxAltitude;
bool useDemoMode;
bool useV3_5_plus;
bool useExtendedChannels;
bool requireBackupController;
bool disableVoltageCheck;
bool bigSky;
bool abortPL;
bool disableDigitalZoom;
bool useSkyBox;
bool useLocker;
bool noLocationLogging;
int lostCommTimeout;
bool usePowerLimitCeiling;
int powerLimitFloor;
int powerLimitDelta;
int powerLimitCountMax;
int vibrationThresholdXY;
int vibrationThresholdZ;
int vibrationFilterLength;  
int minAltitude;
bool landingRetryEnabled;
bool landingSetHome;
bool useControllerProxy;
bool useAlignTkHeading;
int  tkHeadingTarget;
int tkHeadingYawSlewRate;
int tkHeadingPanSlewRate;
bool gpsDeniedSupport;
int gpsDeniedYawSlewMaxRate;
int gpsDeniedPanSlewMaxRate;
int gpsDeniedMaxAltitude;
int launchClimbRate;
int flightClimbRate;
bool commLossLandAbort;
bool gpsDeniedLandOnLoss;

void TestFunction()
{
    Py_Initialize();

    PyRun_SimpleString("import urllib2 as urllib\n");
    PyRun_SimpleString("zoom_in_start = 'http://10.20.30.40/cgi-bin/fwptzctr.cgi?FwModId=0&PortId=0&PtzCode=267&PtzParm=10&RcvData=NO&FwCgiVer=0x0001'\n");
    PyRun_SimpleString("zoom_out_start = 'http://10.20.30.40/cgi-bin/fwptzctr.cgi?FwModId=0&PortId=0&PtzCode=268&PtzParm=10&RcvData=NO&FwCgiVer=0x0001'\n");
    PyRun_SimpleString("zoom_stop = 'http://10.20.30.40/cgi-bin/fwptzctr.cgi?FwModId=0&PortId=0&PtzCode=523&PtzParm=10&RcvData=NO&FwCgiVer=0x0001'\n");


    printf("in\n");
    PyRun_SimpleString("response = urllib.urlopen(zoom_in_start)\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    printf("out\n");
    PyRun_SimpleString("response = urllib.urlopen(zoom_out_start)\n");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    printf("in\n");
    PyRun_SimpleString("response = urllib.urlopen(zoom_in_start)\n");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    printf("out\n");
    PyRun_SimpleString("response = urllib.urlopen(zoom_out_start)\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    printf("in\n");
    PyRun_SimpleString("response = urllib.urlopen(zoom_in_start)\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    Py_Finalize();
}

inline void ltrim(std::string &s)
{
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch)
    {
        return !std::isspace(ch);
    }));
}

inline void rtrim(std::string &s)
{
    s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}


inline void trim(std::string &s)
{
    ltrim(s);
    rtrim(s);
}

inline void toupper(std::string &s)
{
    for_each(s.begin(), s.end(), [](char & c)
    {
        c = ::toupper(c);
    });
}


bool ParseParametersConfigFile(char filename[])
{
    printf("Parsing parameters from config file\n");


    ifstream configFile(filename);
    if (configFile.is_open())
    {
        string line;
        while (getline (configFile, line))
        {
            bool badParameter = true;

            trim(line);
            toupper(line);



            string option;
            string type;
            string value;



            if (line.compare("====") == 0)
            {
                getline (configFile, option);
                trim(option);
                toupper(option);

                getline (configFile, type);
                trim(type);
                toupper(type);

                getline (configFile, value);
                trim(value);
                toupper(value);

                cout << option << " : " << value << "\n";


                if (option.compare("HTTPZOOM") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        httpZoom = true;
                    else
                        httpZoom = false;
                    badParameter = false;
                }

                if (option.compare("PRECISIONLAND") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        precisionLand = true;
                    else
                        precisionLand = false;
                    badParameter = false;
                }

                if (option.compare("NOGROUNDGPS") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        noGroundGps = true;
                    else
                        noGroundGps = false;
                    badParameter = false;
                }

                if (option.compare("MINCRAFTSAT") == 0)
                {
                    minCraftSat = std::stoi(value);
                    badParameter = false;
                }

                if (option.compare("MAXCRAFTHDOP") == 0)
                {
                    maxCraftHdop = std::stoi(value);
                    badParameter = false;
                }

                if (option.compare("MINGROUNDSAT") == 0)
                {
                    minGroundSat = std::stoi(value);
                    badParameter = false;
                }

                if (option.compare("MAXGROUNDHDOP") == 0)
                {
                    maxGroundHdop = std::stoi(value);
                    badParameter = false;
                }

                if (option.compare("DEBUGSKIPGPS") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        debugSkipGPS = true;
                    else
                        debugSkipGPS = false;
                    badParameter = false;
                }

                if (option.compare("BAROCOMPENSATE") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        baroCompensate = true;
                    else
                        baroCompensate = false;
                    badParameter = false;
                }

                if (option.compare("YAWFOLLOWSPAN") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        yawFollowsPan = true;
                    else
                        yawFollowsPan = false;
                    badParameter = false;
                }

                if (option.compare("MAXALTITUDE") == 0)
                {
                    maxAltitude = std::stoi(value);
                    badParameter = false;
                }

                if (option.compare("DEMOMODE") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        useDemoMode = true;
                    else
                        useDemoMode = false;
                    badParameter = false;
                }

                if (option.compare("USEV_3_5_PLUS") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        useV3_5_plus = true;
                    else
                        useV3_5_plus = false;
                    badParameter = false;
                }

                if (option.compare("EXTENDEDCHANNELS") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        useExtendedChannels = true;
                    else
                        useExtendedChannels = false;
                    badParameter = false;
                }

                if (option.compare("REQUIREBACKUPCONTROLLER") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        requireBackupController = true;
                    else
                        requireBackupController = false;
                    badParameter = false;
                }

                if (option.compare("DISABLEVOLTAGECHECK") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        disableVoltageCheck = true;
                    else
                        disableVoltageCheck = false;
                    badParameter = false;
                }

                if (option.compare("BIGSKY") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        bigSky = true;
                    else
                        bigSky = false;
                    badParameter = false;
                }

                if (option.compare("ABORTPL") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        abortPL = true;
                    else
                        abortPL = false;
                    badParameter = false;
                }

                if (option.compare("DISABLEDIGITALZOOM") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        disableDigitalZoom = true;
                    else
                        disableDigitalZoom = false;
                    badParameter = false;
                }

                if (option.compare("USELOCKER") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        useLocker = true;
                    else
                        useLocker = false;
                    badParameter = false;
                }

                if (option.compare("USESKYBOX") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        useSkyBox = true;
                    else
                        useSkyBox = false;
                    badParameter = false;
                }

                if (option.compare("NOLOCATIONLOGGING") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        noLocationLogging = true;
                    else
                        noLocationLogging = false;
                    badParameter = false;
                }

                if (option.compare("LOSTCOMMTIMEOUT") == 0)
                {
                    lostCommTimeout = std::stoi(value);
                    badParameter = false;
				}
                
                if (option.compare("USEPOWERLIMITCEILING") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        usePowerLimitCeiling = true;
                    else
                        usePowerLimitCeiling = false;
                    badParameter = false;
                }
                
                
                if (option.compare("POWERLIMITFLOOR") == 0)
                {
                    powerLimitFloor = std::stoi(value);
                    badParameter = false;
				}
                                
                if (option.compare("POWERLIMITDELTA") == 0)
                {
                    powerLimitDelta = std::stoi(value);
                    badParameter = false;
				}
                                
                
                if (option.compare("POWERLIMITCOUNTMAX") == 0)
                {
                    powerLimitCountMax = std::stoi(value);
                    badParameter = false;
				}
                               
                               
                if (option.compare("VIBRATIONTHRESHXY") == 0)
                {
                    vibrationThresholdXY = std::stoi(value);
                    badParameter = false;
				}
                                
                if (option.compare("VIBRATIONTHRESHZ") == 0)
                {
                    vibrationThresholdZ = std::stoi(value);
                    badParameter = false;
				}
                                
                if (option.compare("VIBRATIONFILTERLENGTH") == 0)
                {
                    vibrationFilterLength = std::stoi(value);
                    badParameter = false;
				}                                
                
                if (option.compare("MINALTITUDE") == 0)
                {
                    minAltitude = std::stoi(value);
                    badParameter = false;
                }
                       
                if (option.compare("LANDINGRETRYENABLED") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        landingRetryEnabled = true;
                    else
                        landingRetryEnabled = false;
                    badParameter = false;
                }         
                
                
                if (option.compare("LANDINGSETHOME") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        landingSetHome = true;
                    else
                        landingSetHome = false;
                    badParameter = false;
                }         
                 
                
                if (option.compare("USECONTROLLERPROXY") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        useControllerProxy = true;
                    else
                        useControllerProxy = false;
                    badParameter = false;
                }                       
                
                
                if (option.compare("USEALIGNTKHEADING") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        useAlignTkHeading = true;
                    else
                        useAlignTkHeading = false;
                    badParameter = false;
                }
                
                
                if (option.compare("TKHEADINGTARGET") == 0)
                {
                    tkHeadingTarget = std::stoi(value);
                    badParameter = false;
                }


                
                if (option.compare("TKHEADINGYAWSLEWRATE") == 0)
                {
                    tkHeadingYawSlewRate = std::stoi(value);
                    badParameter = false;
                }
                

                
                if (option.compare("TKHEADINGPANSLEWRATE") == 0)
                {
                    tkHeadingPanSlewRate = std::stoi(value);
                    badParameter = false;
                }                                                            

                
                if (option.compare("GPSDENIEDSUPPORT") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        gpsDeniedSupport = true;
                    else
                        gpsDeniedSupport = false;
                    badParameter = false;
                }
                
                if (option.compare("GPSDENIEDYAWSLEWMAXRATE") == 0)
                {
                    gpsDeniedYawSlewMaxRate = std::stoi(value);
                    badParameter = false;
                }      
                
                                            
                if (option.compare("GPSDENIEDPANSLEWMAXRATE") == 0)
                {
                    gpsDeniedPanSlewMaxRate = std::stoi(value);
                    badParameter = false;
                }        
                
                
                if (option.compare("GPSDENIEDMAXALTITUDE") == 0)
                {
                    gpsDeniedMaxAltitude = std::stoi(value);
                    badParameter = false;
                }        
                
                                
                if (option.compare("LAUNCHCLIMBRATE") == 0)
                {
                    launchClimbRate = std::stoi(value);
                    badParameter = false;
                }        
                
                                
                if (option.compare("FLIGHTCLIMBRATE") == 0)
                {
                    flightClimbRate = std::stoi(value);
                    badParameter = false;
                }                     
                
                
                if (option.compare("COMMLOSSLANDABORT") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        commLossLandAbort = true;
                    else
                        commLossLandAbort = false;
                    badParameter = false;
                }         
                
                                
                if (option.compare("GPSDENIEDLANDONLOSS") == 0)
                {
                    if (value.compare("TRUE") == 0)
                        gpsDeniedLandOnLoss = true;
                    else
                        gpsDeniedLandOnLoss = false;
                    badParameter = false;
                }         
                
                                
				if (badParameter == true)
                {
                    cout << "BAD PARAM: " << option << "\n";
                    return false;
                }




			}

        }
        configFile.close();
    }
    return true;

}


bool ParseParametersCommandLine(int argc, char* argv[])
{

    printf("Parsing parameters from command line\n");

    int i = 1;

    while (i < argc)
    {
        bool badParameter = true;

        char option[41];
        option[40] = 0;
        strncpy(option, argv[i], 40);
        int index = 0;
        while (option[index] != 0)
        {
          option[index] = toupper(option[index]);
          index++;
        }

        if (strncmp(option, "-HTTPZOOM", 40) == 0)
        {
            httpZoom = true;
            i += 1;
            badParameter = false;
        }

        if (strncmp(option, "-PRECISIONLAND", 40) == 0)
        {
            precisionLand = true;
            i += 1;
            badParameter = false;
        }

        if (strncmp(option, "-NOGROUNDGPS", 40) == 0)
        {
            noGroundGps = true;
            i += 1;
            badParameter = false;
        }

        if (strncmp(option, "-MINCRAFTSAT", 40) == 0)
        {
            minCraftSat = atoi(argv[i+1]);
            i += 2;
            badParameter = false;
        }

        if (strncmp(option, "-MAXCRAFTHDOP", 40) == 0)
        {
            maxCraftHdop = atoi(argv[i+1]);
            i += 2;
            badParameter = false;
        }

        if (strncmp(option, "-MINGROUNDSAT", 40) == 0)
        {
            minGroundSat = atoi(argv[i+1]);
            i += 2;
            badParameter = false;
        }

        if (strncmp(option, "-MAXGROUNDHDOP", 40) == 0)
        {
            maxGroundHdop = atoi(argv[i+1]);
            i += 2;
            badParameter = false;
        }

        if (strncmp(option, "-DEBUGSKIPGPS", 40) == 0)
        {
            debugSkipGPS = true;
            i += 1;
            badParameter = false;
        }

        if (strncmp(option, "-BAROCOMPENSATE", 40) == 0)
        {
            baroCompensate = true;
            i += 1;
            badParameter = false;
        }

        if (strncmp(option, "-YAWFOLLOWSPAN", 40) == 0)
        {
            yawFollowsPan = true;
            i += 1;
            badParameter = false;
        }

        if (strncmp(option, "-MAXALTITUDE", 40) == 0)
        {
            maxAltitude = atoi(argv[i+1]);
            i += 2;
            badParameter = false;
        }

        if (strncmp(option, "-DEMOMODE", 40) == 0)
        {
            useDemoMode = true;
            i += 1;
            badParameter = false;
        }

        if (strncmp(option, "-USEV_3_5_PLUS", 40) == 0)
        {
            useV3_5_plus = true;
            i += 1;
            badParameter = false;
        }


        if (strncmp(option, "-EXTENDEDCHANNELS", 40) == 0)
        {
            useExtendedChannels = true;
            i += 1;
            badParameter = false;
        }


        if (strncmp(option, "-REQUIREBACKUPCONTROLLER", 40) == 0)
        {
            requireBackupController = true;
            i += 1;
            badParameter = false;
        }


        if (strncmp(option, "-DISABLEVOLTAGECHECK", 40) == 0)
        {
            disableVoltageCheck = true;
            i += 1;
            badParameter = false;
        }


        if (strncmp(option, "-BIGSKY", 40) == 0)
        {
            bigSky = true;
            i += 1;
            badParameter = false;
        }


        if (strncmp(option, "-ABORTPL", 40) == 0)
        {
            abortPL = true;
            i += 1;
            badParameter = false;
        }

        if (strncmp(option, "-DISABLEDIGITALZOOM", 40) == 0)
        {
            disableDigitalZoom = true;
            i += 1;
            badParameter = false;
        }

        if (strncmp(option, "-USELOCKER", 40) == 0)
        {
            useLocker = true;
            i += 1;
            badParameter = false;
        }

        if (strncmp(option, "-USESKYBOX", 40) == 0)
        {
            useSkyBox = true;
            i += 1;
            badParameter = false;
        }

        if (strncmp(option, "-NOLOCATIONLOGGING", 40) == 0)
        {
            noLocationLogging = true;
            i += 1;
		}

        if (strncmp(option, "-LOSTCOMMTIMEOUT", 40) == 0)
        {
            lostCommTimeout = atoi(argv[i+1]);
            i += 2;
            badParameter = false;
        }


        if (badParameter == true)
        {
            return false;
        }
    }

    return true;
}



bool ParseParameters(int argc, char* argv[])
{
    for (int i = 0; i < argc; i++)
    {
        printf("%d   : %s\n", i, argv[i]);
    }

    precisionLand = false;
    noGroundGps = false;
    minCraftSat = 8;
    maxCraftHdop = 250;
    minGroundSat = 7;
    maxGroundHdop = 250;
    httpZoom = false;
    baroCompensate = false;
    debugSkipGPS = false;
    yawFollowsPan = false;
    maxAltitude = 6096;
    useDemoMode = false;
    useV3_5_plus = false;
    useExtendedChannels = false;
    requireBackupController = false;
    disableVoltageCheck = false;
    bigSky = false;
    abortPL = false;
    disableDigitalZoom = false;
    useSkyBox = false;
    noLocationLogging = false;
	lostCommTimeout = 5;
    usePowerLimitCeiling = false;
    powerLimitFloor = 10000;
    powerLimitDelta = 10000;
    powerLimitCountMax = 1;
    vibrationThresholdXY = 15;
    vibrationThresholdZ = 100;
    vibrationFilterLength = 20;
    minAltitude = 500;
    landingRetryEnabled = false;
    landingSetHome = true;
    useControllerProxy = false;
    useAlignTkHeading = false;
    tkHeadingTarget = 0;
    tkHeadingYawSlewRate = 25;
    tkHeadingPanSlewRate = 25;
    gpsDeniedSupport = false;
    gpsDeniedYawSlewMaxRate = 25;
    gpsDeniedPanSlewMaxRate = 100;
    gpsDeniedMaxAltitude = 4000;
    launchClimbRate = 70;
    flightClimbRate = 70;
    commLossLandAbort = false;
    gpsDeniedLandOnLoss = true;
    

    bool useConfigFile = false;

    if (argc == 3)
    {
        if (strcmp(argv[1], "-c") == 0)
        {
            useConfigFile = true;
        }
    }


    if (useConfigFile == true)
    {
        return ParseParametersConfigFile(argv[2]);
    }
    else
    {
        return ParseParametersCommandLine(argc, argv);
    }
}


void TestBoxCar()
{
    BoxcarFilter f;

    f.setLength(10);

    for (int i = 0; i < 100; i++)
    {
        float avg = f.addSample(i);
        cout << i << "  :  " << avg << "\n";
    }
}

int main (int argc, char* argv[])
{
   // TestBoxCar();
   // return 0;
 //   TestFunction();
  //  return 0;

    if (argc == 2)
    {
        if (strcmp(argv[1], "-v") == 0)
        {
            printf("%s\n", FMS_VERSION);
            return -1;
        }
    }


    if (ParseParameters(argc, argv) == true)
    {
        cout << "Precision Land " << precisionLand << "\n";
        cout << "No Ground Gps " << noGroundGps << "\n";
        cout << "Min Craft Sat " << minCraftSat << "\n";
        cout << "Max Craft Hdop " << maxCraftHdop << "\n";
        cout << "Min Ground Sat " << minGroundSat << "\n";
        cout << "Max Ground Hdop " << maxGroundHdop << "\n";
        cout << "HTTP Zoom " << httpZoom << "\n";
        cout << "Baro Compensate " << baroCompensate << "\n";
        cout << "Debug Skip GPS " << debugSkipGPS << "\n";
        cout << "Yaw Follows Pan " << yawFollowsPan << "\n";
        cout << "Max Altitude " << maxAltitude << "\n";
        cout << "Demo Mode " << useDemoMode << "\n";
        cout << "Use v3.5 plus " << useV3_5_plus << "\n";
        cout << "Use Extended Channels " << useExtendedChannels << "\n";
        cout << "Require Backup Controller " << requireBackupController << "\n";
        cout << "Disable Voltage Check " << disableVoltageCheck << "\n";
        cout << "BigSky " << bigSky << "\n";
        cout << "Abort Precision Land " << abortPL << "\n";
        cout << "Disable Digital Zoom " << disableDigitalZoom << "\n";
        cout << "Use SkyBox " << useSkyBox << "\n";
        cout << "Use Locker " << useLocker << "\n";
        cout << "NoLocationLogging " << noLocationLogging << "\n";
        cout << "Lost Comm Timeout " << lostCommTimeout << "\n";
        cout << "Use Power Limit Ceiling " << usePowerLimitCeiling << "\n";
        cout << "Power Limit Floor " << powerLimitFloor << "\n";
        cout << "Power Limit Delta " << powerLimitDelta << "\n";
        cout << "Power Limit Count Max " << powerLimitCountMax << "\n";
        cout << "Vibration Threshold XY " << vibrationThresholdXY << "\n";
        cout << "Vibration Threshold Z " << vibrationThresholdZ << "\n";
        cout << "Vibration Filter Length " << vibrationFilterLength << "\n";
        cout << "Minimum Altitude " << minAltitude << "\n";
        cout << "Landing Retry Enabled " << landingRetryEnabled << "\n";
        cout << "Landing Set Home " << landingSetHome << "\n";
        cout << "Use Controller Proxy " << useControllerProxy << "\n";
        cout << "Use Align TK Heading " << useAlignTkHeading << "\n";
        cout << "TK Heading Target " << tkHeadingTarget << "\n";
        cout << "TK Heading Yaw Slew Rate " << tkHeadingYawSlewRate << "\n";
        cout << "TK Heading Pan Slew Rate " << tkHeadingPanSlewRate << "\n";
        cout << "GPS Denied Support " << gpsDeniedSupport << "\n";
        cout << "GPS Denied Yaw Slew Max Rate " << gpsDeniedYawSlewMaxRate << "\n";
        cout << "GPS Denied Pan Slew Max Rate " << gpsDeniedPanSlewMaxRate << "\n";
        cout << "GPS Denied Max Altitude " << gpsDeniedMaxAltitude << "\n";
        cout << "Launch Climb Rate " << launchClimbRate << "\n";
        cout << "Flight Climb Rate " << flightClimbRate << "\n";
        cout << "Comm Loss Land Abort " << commLossLandAbort << "\n";
        cout << "GPS Denied Land on Loss " << gpsDeniedLandOnLoss << "\n";
        
       
    }
    else
    {
        cout << "Bad parameter\n";
        return 0;
    }

    HFLogger::openLog();

    HFLogger::logMessage("fms_core start");
    HFLogger::logMessage("version: %s", FMS_VERSION);

    HFLogger::logMessage("Precision Land   : %d", precisionLand);
    HFLogger::logMessage("No Ground GPS    : %d", noGroundGps);
    HFLogger::logMessage("Craft Min Sat    : %d", minCraftSat);
    HFLogger::logMessage("Craft Max Hdop   : %d", maxCraftHdop);
    HFLogger::logMessage("Ground Min Sat   : %d", minGroundSat);
    HFLogger::logMessage("Ground Max Hdop  : %d", maxGroundHdop);
    HFLogger::logMessage("HTTP Zoom        : %d", httpZoom);
    HFLogger::logMessage("Baro Compensate  : %d", baroCompensate);
    HFLogger::logMessage("Debug Skip GPS   : %d", debugSkipGPS);
    HFLogger::logMessage("Yaw Follows Pan  : %d", yawFollowsPan);
    HFLogger::logMessage("Max Altitude     : %d", maxAltitude);
    HFLogger::logMessage("Demo Mode        : %d", useDemoMode);
    HFLogger::logMessage("Use V3.5 Plus    : %d", useV3_5_plus);
    HFLogger::logMessage("Use Extd Channels: %d", useExtendedChannels);
    HFLogger::logMessage("Require Backup   : %d", requireBackupController);
    HFLogger::logMessage("Disable Volt Chk : %d", disableVoltageCheck);
    HFLogger::logMessage("BigSky           : %d", bigSky);
    HFLogger::logMessage("Abort Prcsn Land : %d", abortPL);
    HFLogger::logMessage("Disable Dig Zoom : %d", disableDigitalZoom);
    HFLogger::logMessage("Use SkyBox       : %d", useSkyBox);
    HFLogger::logMessage("Use Locker       : %d", useLocker);
    HFLogger::logMessage("No Location Log  : %d", noLocationLogging);
    HFLogger::logMessage("Lost Comm Timeout: %d", lostCommTimeout);
    HFLogger::logMessage("Power Limit Clng : %d", usePowerLimitCeiling);
    HFLogger::logMessage("Power Limit Floor: %d", powerLimitFloor);
    HFLogger::logMessage("Power Limit Delta: %d", powerLimitDelta);
    HFLogger::logMessage("Power Limit Ct Mx: %d", powerLimitCountMax);
    HFLogger::logMessage("Vibe Threshld XY:  %d", vibrationThresholdXY);
    HFLogger::logMessage("Vibe Threshld Z :  %d", vibrationThresholdZ);
    HFLogger::logMessage("Vibe Filter Lngth: %d", vibrationFilterLength);
    HFLogger::logMessage("Minimum Altitude : %d", minAltitude);
    HFLogger::logMessage("Landing Rtry Enbl: %d", landingRetryEnabled);
    HFLogger::logMessage("Landing Set Home : %d", landingSetHome);
    HFLogger::logMessage("Use Contrl Proxy : %d", useControllerProxy);
    HFLogger::logMessage("Use Align TK Hdg : %d", useAlignTkHeading);
    HFLogger::logMessage("TK Heading Trgt  : %d", tkHeadingTarget);
    HFLogger::logMessage("TK Heading Yaw Rt: %d", tkHeadingYawSlewRate);
    HFLogger::logMessage("TK Heading Pan Rt: %d", tkHeadingPanSlewRate);
    HFLogger::logMessage("GPS Denied Supprt: %d", gpsDeniedSupport);
    HFLogger::logMessage("GPS Denied Yaw Rt: %d", gpsDeniedYawSlewMaxRate);
    HFLogger::logMessage("GPS Denied Pan Rt: %d", gpsDeniedPanSlewMaxRate);
    HFLogger::logMessage("GPS Denied Mx Alt: %d", gpsDeniedMaxAltitude);
    HFLogger::logMessage("Launch Climb Rate: %d", launchClimbRate);
    HFLogger::logMessage("Flight Climb Rate: %d", flightClimbRate);
    HFLogger::logMessage("Comm Loss Lnd Abt: %d", commLossLandAbort);
    HFLogger::logMessage("GPS Denied Lnd Ls: %d", gpsDeniedLandOnLoss);

/*
    if (baroCompensate && noGroundGps)
    {
        HFLogger::logMessage("-baroCompensate and -noGroundGps are incompatible");
        cout << "-baroCompensate and -noGroundGps are incompatible";
        return 0;
    }

*/

    try {
        FlightManagementSystem fms;
        fms.m_precisionLanding = precisionLand;
        fms.m_noGroundGps = noGroundGps;
        fms.m_minCraftSat = minCraftSat;
        fms.m_maxCraftHdop = maxCraftHdop;
        fms.m_minGroundSat = minGroundSat;
        fms.m_maxGroundHdop = maxGroundHdop;
        fms.m_httpZoom = httpZoom;
        fms.m_baroCompensate = baroCompensate;
        fms.m_debugSkipGps = debugSkipGPS;
        fms.m_yawFollowsPan = yawFollowsPan;
        fms.m_maxAltitude = maxAltitude;
        fms.m_demoMode = useDemoMode;
        fms.m_useV35Plus = useV3_5_plus;
        fms.m_useExtendedChannels = useExtendedChannels;
        fms.m_requireBackupController = requireBackupController;
        fms.m_disableVoltageCheck = disableVoltageCheck;
        fms.m_bigSky = bigSky;
        fms.m_abortPL = abortPL;
        fms.m_disableDigitalZoom = disableDigitalZoom;
        fms.m_useSkyBox = useSkyBox;
        fms.m_useLocker = useLocker;
        fms.m_noLocationLogging = noLocationLogging;
		fms.m_lostCommTimeout = lostCommTimeout;
        fms.m_usePowerLimitCeiling = usePowerLimitCeiling;
        fms.m_powerLimitFloor = ((float)powerLimitFloor / 1000.0f);
        fms.m_powerLimitDelta = ((float)powerLimitDelta / 1000.0f);
        fms.m_powerLimitCountMax = powerLimitCountMax;
        fms.m_vibrationThresholdXY = vibrationThresholdXY;
        fms.m_vibrationThresholdZ = vibrationThresholdZ;
        fms.m_vibrationFilterLength = vibrationFilterLength;
        fms.m_minAltitude = minAltitude;
        fms.m_landingRetryEnabled = landingRetryEnabled;
        fms.m_landingSetHome = landingSetHome;
        fms.m_useControllerProxy = useControllerProxy;
        fms.m_useAlignTkHeading = useAlignTkHeading;
        fms.m_tkHeadingTarget = tkHeadingTarget;
        fms.m_tkHeadingYawSlewRate = tkHeadingYawSlewRate;
        fms.m_tkHeadingPanSlewRate = tkHeadingPanSlewRate;
        fms.m_gpsDeniedSupport = gpsDeniedSupport;
        fms.m_gpsDeniedYawSlewMaxRate = gpsDeniedYawSlewMaxRate;
        fms.m_gpsDeniedPanSlewMaxRate = gpsDeniedPanSlewMaxRate;
        fms.m_gpsDeniedMaxAltitude = gpsDeniedMaxAltitude;
        fms.m_launchClimbRate = launchClimbRate;
        fms.m_flightClimbRate = flightClimbRate;
        fms.m_commLossLandAbort = commLossLandAbort;
        fms.m_gpsDeniedLandOnLoss = gpsDeniedLandOnLoss;
        

        fms.Start();

        bool runFMS = true;
        while (runFMS == true)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }


        fms.Stop();
    } catch (const std::exception& e) {
        HFLogger::logMessage("EXCEPTION - %s", e.what());
    }
    HFLogger::closeLog();

    return 0;

}
