#include "TcpDataHandler.h"

#include <sys/socket.h>
#include <unistd.h>

#include <iostream>



TcpDataHandler::TcpDataHandler()
{
    //ctor
    m_isConnected = false;
    m_socketDesc = -1;

}

TcpDataHandler::~TcpDataHandler()
{
    //dtor
}


bool TcpDataHandler::IsConnected()
{
    return m_isConnected;
}

bool TcpDataHandler::IsConnectionActive(int maxSeconds)
{
    bool active = false;
    if (m_isConnected == true)
    {
        time_t currentTime;
        time(&currentTime);
        if ((currentTime - m_lastReceivedTime) < maxSeconds)
        {
            active = true;
        }
    }

    return active;
}

void TcpDataHandler::AttachSocket(int socketDesc)
{
    // Record the socket file descriptor
    m_socketDesc = socketDesc;

}


void TcpDataHandler::Start()
{
    time(&m_lastReceivedTime);
    m_isConnected = true;

    // Begin a thread that reads from the socket
    m_incomingDataThread = std::thread(&TcpDataHandler::ConnectionHandler, this);


    // Begin a thread that writes to the socket
    m_runOutgoingThread = true;
    m_outgoingDataThread = std::thread(&TcpDataHandler::OutgoingHandler, this);


}

void TcpDataHandler::Stop()
{
    close(m_socketDesc);


    m_runOutgoingThread = false;
    m_outgoingDataQueue.SignalAbort();
    m_outgoingDataThread.join();

    m_incomingDataThread.join();
    m_isConnected = false;
    m_incomingDataQueue->SignalAbort();

}

bool TcpDataHandler::SendMessage(std::vector<uint8_t>* msgVector)
{
    m_outgoingDataQueue.Enqueue(msgVector);
    return true;
}


void TcpDataHandler::OutgoingHandler()
{
    while (m_runOutgoingThread == true)
    {
        std::vector<uint8_t>* msgVector;

        if (m_outgoingDataQueue.Dequeue(msgVector) == true)
        {
            if (m_socketDesc != -1)
            {
                write(m_socketDesc, msgVector->data(), msgVector->size());
            }
            delete msgVector;
        }
        else
        {
        }
    }

    m_outgoingDataQueue.FlushQueue();
}


void TcpDataHandler::ConnectionHandler()
{
    int read_size;
    char client_message[256];

    bool openConnection = true;
    while (openConnection)
    {
        if (m_socketDesc != -1)
        {
            read_size = recv(m_socketDesc, client_message, 256, 0);

            if (read_size <= 0)
            {
                openConnection = false;
            }
            else
            {
                std::vector<uint8_t>* chunk = new std::vector<uint8_t>(read_size);
                for (int i = 0; i < read_size; i++)
                  (*chunk)[i] = client_message[i];

                if (m_incomingDataQueue != nullptr)
                {
                    m_incomingDataQueue->Enqueue(chunk);
                }
                time(&m_lastReceivedTime);
            }


            if (read_size == 0)
                openConnection = false;
        }
    }

    close(m_socketDesc);

}
