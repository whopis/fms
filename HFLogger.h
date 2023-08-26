#ifndef HFLOGGER_H
#define HFLOGGER_H

#include <stdint.h>
#include <mutex>
#include <sys/time.h>


class HFLogger
{
    public:
        static bool openLog();
        static bool closeLog();

        static void logMessage(const char* format...);


    protected:
        static std::mutex logMutex;

        static char msgTimestamp[256];
        static char fileTimestamp[256];
        
        static int fileOrdinal;
        
        static FILE *fp;
        static bool isLogOpen;

        static void setMsgTimestamp();
        static void setFileTimestamp();
        static void setFileOrdinal();
};


#endif //HFLOGGER_H
