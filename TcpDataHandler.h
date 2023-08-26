#ifndef TCPDATAHANDLER_H
#define TCPDATAHANDLER_H

#include "SerialDataHandler.h"

#include "ConcurrentQueue.h"

#include <thread>

#include <time.h>




class TcpDataHandler : public SerialDataHandler
{
    public:
        TcpDataHandler();
        virtual ~TcpDataHandler();

        void AttachSocket(int socketDesc);
        void Start();
        void Stop();

        bool IsConnected();

        bool SendMessage(std::vector<uint8_t>* msgVector);

        bool IsConnectionActive(int maxSeconds);


    protected:
    private:
        std::thread m_incomingDataThread;
        std::thread m_outgoingDataThread;

        int m_socketDesc;
        void ConnectionHandler();
        void OutgoingHandler();

        bool m_isConnected;

        bool m_runOutgoingThread;


        time_t m_lastReceivedTime;

};

#endif // TCPDATAHANDLER_H
