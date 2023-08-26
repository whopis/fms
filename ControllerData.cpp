#include "ControllerData.h"

#include <stdio.h>

ControllerData::ControllerData()
{
    //ctor
    JoyX = 0;
    JoyY = 0;
    JoyZ = 0;
    JoyButton = false;
    counter = 0;

    RemoteJoyX = 0;
    RemoteJoyY = 0;
    RemoteJoyZ = 0;
    RemoteJoyButton = false;

    RoiLatitude = 0;
    RoiLongitude = 0;
    RoiAltitude = 0;
    NewRoi = false;

    LedState = 1;
    LaserState = 0;
    
    GpsOn = 1;
    
    gpsDeniedDefaultLatitude = 2000000000;  // 200.0 degrees
    gpsDeniedDefaultLongitude = 2000000000;  // 200.0 degrees
    
    
}

ControllerData::~ControllerData()
{
    //dtor
}


void ControllerData::QueueCommand(HFCommand* hfCommand)
{
    m_commandQueue.Enqueue(hfCommand);
}

void ControllerData::ClearCommandQueue()
{
    m_commandQueue.FlushQueue();
}

HFCommand* ControllerData::GetFirstCommand()
{
    HFCommand* cmd = nullptr;

    if (m_commandQueue.TryDequeue(cmd) == true)
    {
        return cmd;
    }
    else
    {
        return nullptr;
    }
}


void ControllerData::WriteCommandFile()
{
    try
    {
        FILE *fp;
        fp = fopen("/home/pi/hti/apps/bguLogServer/bguCommand.txt", "w");
        if (!fp)
            return;
        fprintf(fp, "{\n");
        fprintf(fp, "\t\"ledState\" : %d,\n", LedState);
        fprintf(fp, "\t\"laserState\" : %d\n", LaserState);
        fprintf(fp, "}");
        fclose(fp);
    }
    catch (const std::exception& e)
    {
    }
}

