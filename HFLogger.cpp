#include <stdio.h>
#include <stdarg.h>

#include <string.h>


#include "HFLogger.h"


std::mutex HFLogger::logMutex;


char HFLogger::msgTimestamp[256];
char HFLogger::fileTimestamp[256];
bool HFLogger::isLogOpen = false;
FILE* HFLogger::fp = nullptr;

int HFLogger::fileOrdinal;

bool HFLogger::openLog()
{
    bool success = false;

    try
    {
        setFileTimestamp();

        char logFile[512];
        strncpy(logFile, "/home/pi/hti/logs/fms/", 512);
        strncat(logFile, fileTimestamp, 512);

        printf("log file = %s\n", logFile);


        fp = fopen(logFile, "w");
        success = true;
    }
    catch (const std::exception& e)
    {
        success = false;
    }

    if (success == true)
    {
        isLogOpen = true;
    }
    else
    {
        isLogOpen = false;
    }


    return success;
}


bool HFLogger::closeLog()
{
    bool success = false;

    try
    {
        fclose(fp);
        success = true;
    }
    catch (const std::exception& e)
    {
        success = false;
    }

    return success;
}



void HFLogger::logMessage(const char* format...)
{

    std::lock_guard<std::mutex> lock(logMutex);

    try
    {
        if (fp != nullptr)
        {
            setMsgTimestamp();

            va_list args;
            va_start(args, format);
            fprintf(fp, "%s ", msgTimestamp);
            vfprintf(fp, format, args);
            fprintf(fp, "\n");
            fflush(fp);
            va_end(args);
        }
    }
    catch (const std::exception& e)
    {
    }
}



void HFLogger::setMsgTimestamp()
{
    struct timeval tv;
    struct tm *tm;

    gettimeofday(&tv, NULL);

    tm = localtime(&tv.tv_sec);

    sprintf(msgTimestamp, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
            tm->tm_year + 1900,
            tm->tm_mon + 1,
            tm->tm_mday,
            tm->tm_hour,
            tm->tm_min,
            tm->tm_sec,
            (int) (tv.tv_usec / 1000)
        );
}


void HFLogger::setFileTimestamp()
{
    struct timeval tv;
    struct tm *tm;

    setFileOrdinal();
    gettimeofday(&tv, NULL);

    tm = localtime(&tv.tv_sec);

//    sprintf(fileTimestamp, "fms_%04d%02d%02dT%02d%02d%02d.txt",
    sprintf(fileTimestamp, "%06d_fms_%04d%02d%02dT%02d%02d%02d.txt",
            fileOrdinal,
            tm->tm_year + 1900,
            tm->tm_mon + 1,
            tm->tm_mday,
            tm->tm_hour,
            tm->tm_min,
            tm->tm_sec
        );
}

void HFLogger::setFileOrdinal()
{
    char inFile[256];
    char inData[256];
    
    strncpy(inFile, "/home/pi/hti/apps/session/session.txt", 256);
    printf("session file name = %s\n", inFile);

    FILE* fpIn = nullptr;
    int fileValue = 0;
    
    try
    {
        fpIn = fopen(inFile, "r");
        if (fpIn != nullptr)
        {
            fscanf(fpIn, "%d", &fileValue);
            printf("session file value = %06d\n", fileValue);
            fileOrdinal = fileValue;
            fclose(fpIn);
        }
        
    }
    catch (const std::exception& e)
    {
        printf("Exception while reading log file ordinal\n");
        printf("%s", e.what());
    }

}
