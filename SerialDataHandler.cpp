#include "SerialDataHandler.h"

SerialDataHandler::SerialDataHandler()
{
    //ctor
    m_incomingDataQueue = nullptr;
}

SerialDataHandler::~SerialDataHandler()
{
    //dtor
}


void SerialDataHandler::AttachIncomingDataQueue(ConcurrentQueue<std::vector<uint8_t>*>* incomingDataQueue)
{
    m_incomingDataQueue = incomingDataQueue;
}
