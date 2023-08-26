#ifndef SERIALDATAHANDLER_H
#define SERIALDATAHANDLER_H

#include "ConcurrentQueue.h"



class SerialDataHandler
{
    public:
        SerialDataHandler();
        virtual ~SerialDataHandler();


        void AttachIncomingDataQueue(ConcurrentQueue<std::vector<uint8_t>*>* incomingDataQueue);


        virtual void Start() = 0;
        virtual void Stop() = 0;
        virtual bool IsConnected() = 0;
        virtual bool SendMessage(std::vector<uint8_t>* msgVector) = 0;
        virtual bool IsConnectionActive(int maxSeconds) = 0;

    protected:
        ConcurrentQueue<std::vector<uint8_t>*>* m_incomingDataQueue;
        ConcurrentQueue<std::vector<uint8_t>*> m_outgoingDataQueue;

    private:

};

#endif // SERIALDATAHANDLER_H





