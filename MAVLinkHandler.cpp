#include "MAVLinkHandler.h"

#include <iostream>


#include "HFLogger.h"

using namespace std;

MAVLinkHandler::MAVLinkHandler(SerialDataHandler* serialDataHandler)
{
    //ctor

    // Attach the incoming data queue to the serial data handler
    serialDataHandler->AttachIncomingDataQueue(&m_incomingDataQueue);

    // Attach the data handler
    m_serialDataHandler = serialDataHandler;


    m_badPacketCount = 0;

}

MAVLinkHandler::~MAVLinkHandler()
{
    //dtor
    delete m_serialDataHandler;
}

bool MAVLinkHandler::IsConnectionActive(int maxSeconds)
{
    return m_serialDataHandler->IsConnectionActive(maxSeconds);
}

void MAVLinkHandler::AttachMessageQueue(ConcurrentQueue<mavlink_message_t*>* messageQueue)
{
    m_messageQueue = messageQueue;
}



void MAVLinkHandler::Start()
{
    // Start the underlying serial data handler
    m_serialDataHandler->Start();

    // Begin a thread to convert incoming data to MAVLink packets
    m_packetThread = std::thread(&MAVLinkHandler::PacketHandler, this);

}

void MAVLinkHandler::Stop()
{
    m_isConnected = true;
    m_serialDataHandler->Stop();
    m_packetThread.join();
}

bool MAVLinkHandler::IsConnected()
{
    return m_isConnected;
}

void MAVLinkHandler::SendMessage(mavlink_message_t* msg)
{
    std::vector<uint8_t>* msgVector = new std::vector<uint8_t>(msg->len + MAVLINK_NUM_NON_PAYLOAD_BYTES);

    mavlink_msg_to_send_buffer(msgVector->data(), msg);

    m_serialDataHandler->SendMessage(msgVector);

}



void MAVLinkHandler::SendMessage_Heartbeat(SourceDeviceType device, mavlink_heartbeat_t* heartbeat)
{
    uint8_t systemId = SourceData::SystemId(device);
    uint8_t componentId = SourceData::ComponentId(device);

    // Pack this into a message
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
    mavlink_msg_heartbeat_encode(systemId, componentId, &msg, heartbeat);
    SendMessage(&msg);
}


void MAVLinkHandler::SendMessage_SysStatus(SourceDeviceType device, mavlink_sys_status_t* msgSysStatus)
{
    uint8_t systemId = SourceData::SystemId(device);
    uint8_t componentId = SourceData::ComponentId(device);

    // Pack this into a message
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
    mavlink_msg_sys_status_encode(systemId, componentId, &msg, msgSysStatus);
    SendMessage(&msg);
}



void MAVLinkHandler::SendMessage_RequestDataStream(SourceDeviceType device, mavlink_request_data_stream_t* msgRequestDataStream)
{
    uint8_t systemId = SourceData::SystemId(device);
    uint8_t componentId = SourceData::ComponentId(device);

    // Pack this into a message
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
    mavlink_msg_request_data_stream_encode(systemId, componentId, &msg, msgRequestDataStream);
    SendMessage(&msg);
}



void MAVLinkHandler::SendMessage_RCChannelsOverride(SourceDeviceType device, mavlink_rc_channels_override_t* msgRCChannelsOverride)
{
    uint8_t systemId = SourceData::SystemId(device);
    uint8_t componentId = SourceData::ComponentId(device);

    // Packet this into a message
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
    mavlink_msg_rc_channels_override_encode(systemId, componentId, &msg, msgRCChannelsOverride);
    SendMessage(&msg);
}


void MAVLinkHandler::SendMessage_GpsRawInt(SourceDeviceType device, mavlink_gps_raw_int_t* msgGpsRawInt)
{
    uint8_t systemId = SourceData::SystemId(device);
    uint8_t componentId = SourceData::ComponentId(device);

    // Packet this into a message
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
    mavlink_msg_gps_raw_int_encode(systemId, componentId, &msg, msgGpsRawInt);
    SendMessage(&msg);
}


void MAVLinkHandler::SendMessage_GlobalPositionInt(SourceDeviceType device, mavlink_global_position_int_t* msgGlobalPositionInt)
{
    uint8_t systemId = SourceData::SystemId(device);
    uint8_t componentId = SourceData::ComponentId(device);

    // Packet this into a message
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
    mavlink_msg_global_position_int_encode(systemId, componentId, &msg, msgGlobalPositionInt);
    SendMessage(&msg);
}


void MAVLinkHandler::SendMessage_SetPositionTargetGlobalInt(SourceDeviceType device, mavlink_set_position_target_global_int_t* msgSetPositionTargetGlobalInt)
{
    uint8_t systemId = SourceData::SystemId(device);
    uint8_t componentId = SourceData::ComponentId(device);

    // Packet this into a message
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
    mavlink_msg_set_position_target_global_int_encode(systemId, componentId, &msg, msgSetPositionTargetGlobalInt);
    SendMessage(&msg);
}



void MAVLinkHandler::SendMessage_CommandLong(SourceDeviceType device, mavlink_command_long_t* msgCommandLong)
{
    uint8_t systemId = SourceData::SystemId(device);
    uint8_t componentId = SourceData::ComponentId(device);

    // Packet this into a message
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
    mavlink_msg_command_long_encode(systemId, componentId, &msg, msgCommandLong);
    SendMessage(&msg);
}



void MAVLinkHandler::SendMessage_MissionItem(SourceDeviceType device, mavlink_mission_item_t* msgMissionItem)
{
    uint8_t systemId = SourceData::SystemId(device);
    uint8_t componentId = SourceData::ComponentId(device);

    // Packet this into a message
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
    mavlink_msg_mission_item_encode(systemId, componentId, &msg, msgMissionItem);
    SendMessage(&msg);
}

void MAVLinkHandler::SendMessage_RCChannelsRaw(SourceDeviceType device, mavlink_rc_channels_raw_t* rcChannelsRaw)
{
    uint8_t systemId = SourceData::SystemId(device);
    uint8_t componentId = SourceData::ComponentId(device);

    // Packet this into a message
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
    mavlink_msg_rc_channels_raw_encode(systemId, componentId, &msg, rcChannelsRaw);
    SendMessage(&msg);
}


void MAVLinkHandler::SendMessage_StatusText(SourceDeviceType device, mavlink_statustext_t* statusText)
{
    uint8_t systemId = SourceData::SystemId(device);
    uint8_t componentId = SourceData::ComponentId(device);

    // Packet this into a message
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
    mavlink_msg_statustext_encode(systemId, componentId, &msg, statusText);
    SendMessage(&msg);
}


void MAVLinkHandler::SendMessage_SystemTime(SourceDeviceType device, mavlink_system_time_t* systemTime)
{
    uint8_t systemId = SourceData::SystemId(device);
    uint8_t componentId = SourceData::ComponentId(device);

    // Packet this into a message
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
    mavlink_msg_system_time_encode(systemId, componentId, &msg, systemTime);
    SendMessage(&msg);
}

void MAVLinkHandler::SendMessage_ParamSet(SourceDeviceType device, mavlink_param_set_t* paramSet)
{
    uint8_t systemId = SourceData::SystemId(device);
    uint8_t componentId = SourceData::ComponentId(device);

    // Pack this into a message
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
    mavlink_msg_param_set_encode(systemId, componentId, &msg, paramSet);
    SendMessage(&msg);
}


void MAVLinkHandler::SendMessage_ParamRequestRead(SourceDeviceType device, mavlink_param_request_read_t* paramRequest)
{
    uint8_t systemId = SourceData::SystemId(device);
    uint8_t componentId = SourceData::ComponentId(device);

    // Pack this into a message
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
    mavlink_msg_param_request_read_encode(systemId, componentId, &msg, paramRequest);
    SendMessage(&msg);
}


void MAVLinkHandler::SendMessage_ParamValue(SourceDeviceType device, mavlink_param_value_t* paramValue)
{
    uint8_t systemId = SourceData::SystemId(device);
    uint8_t componentId = SourceData::ComponentId(device);

    // Pack this into a message
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
    mavlink_msg_param_value_encode(systemId, componentId, &msg, paramValue);
    SendMessage(&msg);
}



void MAVLinkHandler::SendMessage_AutopilotVersionRequest(SourceDeviceType device, mavlink_autopilot_version_request_t* autopilotVersionRequest)
{
    uint8_t systemId = SourceData::SystemId(device);
    uint8_t componentId = SourceData::ComponentId(device);

    // Pack this into a message
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
    mavlink_msg_autopilot_version_request_encode(systemId, componentId, &msg, autopilotVersionRequest);
    SendMessage(&msg);
}


void MAVLinkHandler::SendMessage_Rangefinder(SourceDeviceType device, mavlink_rangefinder_t* rangefinder)
{
    uint8_t systemId = SourceData::SystemId(device);
    uint8_t componentId = SourceData::ComponentId(device);

    // Pack this into a message
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
    mavlink_msg_rangefinder_encode(systemId, componentId, &msg, rangefinder);
    SendMessage(&msg);
}

void MAVLinkHandler::PacketHandler()
{
    packet_state = PACKET_STATE_SYNC;

    bool runPacketHandler = true;
    while (runPacketHandler)
    {
        // Read a chunk from the queue
        std::vector<uint8_t>* chunk;
        if (m_incomingDataQueue.Dequeue(chunk) == true)
        {
            // Loop through the bytes in the chunk
            for (unsigned int i = 0; i < chunk->size(); i++)
            {
                uint8_t* buffer;
                buffer = ParseByte(chunk->data()[i]);

                if (buffer != nullptr)
                {
                    HandleFullPacket(buffer);
                }
            }

            delete chunk;
        }
        else
        {
            // The underlying queue may have been aborted
            if (m_serialDataHandler->IsConnected() == false)
            {
                runPacketHandler = false;
                m_incomingDataQueue.FlushQueue();

                m_isConnected = false;
                m_messageQueue->SignalAbort();
            }
        }
    }
}


void MAVLinkHandler::HandleFullPacket(uint8_t* buffer)
{
    // Caclulate the CRC from the packet
    uint16_t crc = 0;

    crc = crc_calculate(&(buffer[1]), packet_payloadLength + 5);
    crc_accumulate(m_messageCRCs[buffer[5]], &crc);

    mavlink_message_t* msg = new mavlink_message_t;
    msg->len = buffer[1];
    msg->seq = buffer[2];
    msg->sysid = buffer[3];
    msg->compid = buffer[4];
    msg->msgid = buffer[5];
    memcpy(msg->payload64, &(buffer[6]), msg->len);

    uint8_t cs0 = buffer[packet_totalLength - 2];
    uint8_t cs1 = buffer[packet_totalLength - 1];

    // We have copied all the data from the buffer
    delete buffer;

    // Verify the checksum
    msg->checksum = cs0 | (cs1 << 8);


    m_totalPacketCount++;
    if (crc == msg->checksum)
    {
        m_goodPacketCount++;
        m_messageQueue->Enqueue(msg);

    }
    else
    {
        m_badPacketCount++;
        HFLogger::logMessage("MAVLinkHandler: Failed checksum - badCount: %d  goodCount: %d  totalCount: %d  msgId: %d object: %08x", m_badPacketCount, m_goodPacketCount, m_totalPacketCount, msg->msgid, this);
        delete msg;
    }

    if ((m_totalPacketCount % 100) == 0)
    {
        HFLogger::logMessage("MAVLinkHandler: Packet Count - badCount: %d  goodCount: %d  totalCount: %d  object: %08x", m_badPacketCount, m_goodPacketCount, m_totalPacketCount, this);
    }
}



uint8_t*  MAVLinkHandler::ParseByte(uint8_t dataByte)
{
    uint8_t* msgBuffer = nullptr;

    switch (packet_state)
    {
        case PACKET_STATE_SYNC:
            if (dataByte == 0xfe)
            {
                packet_state = PACKET_STATE_LENGTH;
            }
            break;

        case PACKET_STATE_LENGTH:
            packet_payloadLength = dataByte;
            packet_totalLength = packet_payloadLength + 8;
            packet_buffer = new uint8_t[packet_totalLength];
            packet_buffer[0] = 0xfe;
            packet_buffer[1] = dataByte;
            packet_currentPosition = 2;
            packet_state = PACKET_STATE_REMAINDER;
            break;


        case PACKET_STATE_REMAINDER:
            packet_buffer[packet_currentPosition] = dataByte;
            packet_currentPosition++;
            if (packet_currentPosition >= packet_totalLength)
            {
                msgBuffer = packet_buffer;
                packet_state = PACKET_STATE_SYNC;
            }
            break;

        default:
            packet_state = PACKET_STATE_SYNC;
            break;
    }


    return msgBuffer;
}


/*

void MAVLinkHandler::PacketHandler()
{

    uint8_t completeBuffer[2048];
    int completeBufferLength = 0;

    int packetStartIndex = 0;

    bool runPacketHandler = true;
    while (runPacketHandler)
    {
        // Read a chunk from the queue
        std::vector<uint8_t>* chunk;
        if (m_incomingDataQueue.Dequeue(chunk) == true)
        {
            // Check data size vs buffer size
            if ((completeBufferLength + chunk->size()) < 2048)
            {
                // Move any remaining data to the start of the buffer
                int remainingDataSize = completeBufferLength - packetStartIndex;
                for (int i = 0; i < remainingDataSize; i++)
                {
                    completeBuffer[i] = completeBuffer[i + packetStartIndex];
                }
                packetStartIndex = 0;
                completeBufferLength = remainingDataSize;

                // Copy in the new chunk
                int chunkSize = chunk->size();
                for (int i = 0; i < chunkSize; i++)
                {
                    completeBuffer[completeBufferLength++] = chunk->data()[i];
                }
            }
            else
            {
                // Drop the buffer and packet and start over
                completeBufferLength = 0;
                packetStartIndex = 0;
            }

            delete chunk;

            bool dataRemaining = true;

            // 8 bytes is the minumum packet size (sync, length, sequence, system Id, component Id, message Id, 2 byte CRC)
            if ((completeBufferLength - packetStartIndex) < 8)
            {
                dataRemaining = false;
            }

            // Loop throught the buffer and pull out packets
            while (dataRemaining == true)
            {
                // Search for the frame start
                while ((completeBuffer[packetStartIndex] != 0xFE) && (dataRemaining == true))
                {
                    packetStartIndex++;
                    if ((completeBufferLength - packetStartIndex) < 8)
                      dataRemaining = false;
                }

                // Check to see if we have enough data to continue
                if (dataRemaining == true)
                {
                    int payloadLength = completeBuffer[packetStartIndex + 1];

                    // Check if we have the full packet
                    if ((completeBufferLength - packetStartIndex) < (payloadLength + 8))
                    {
                        // We don't have the full packet - wait for the next chunk
                        dataRemaining = false;
                    }
                    else
                    {
                        // We have a full packet

                        // Caclulate the CRC from the packet
                        uint16_t crc = 0;

                        crc = crc_calculate(&(completeBuffer[packetStartIndex+1]), payloadLength + 5);
                        crc_accumulate(m_messageCRCs[completeBuffer[packetStartIndex+5]], &crc);


                       // packetStartIndex += 8 + payloadLength;

                        mavlink_message_t* msg = new mavlink_message_t;
                        packetStartIndex++; // Skip the frame synce
                        msg->len = completeBuffer[packetStartIndex++];
                        msg->seq = completeBuffer[packetStartIndex++];
                        msg->sysid = completeBuffer[packetStartIndex++];
                        msg->compid = completeBuffer[packetStartIndex++];
                        msg->msgid = completeBuffer[packetStartIndex++];
                        memcpy(msg->payload64, &(completeBuffer[packetStartIndex]), msg->len);

                        packetStartIndex += msg->len;
                        uint8_t cs0 = completeBuffer[packetStartIndex++];
                        uint8_t cs1 = completeBuffer[packetStartIndex++];

                        // Verify the checksum
                        msg->checksum = cs0 | (cs1 << 8);


                        if (crc == msg->checksum)
                        {
                            m_messageQueue->Enqueue(msg);

                        }
                        else
                        {
                            delete msg;
                            m_badPacketCount++;
                            HFLogger::logMessage("MAVLinkHandler: Failed checksum - count: %d  object: %08x", m_badPacketCount, this);
                        }

                    }
                }

                if ((completeBufferLength - packetStartIndex) < 8)
                  dataRemaining = false;

            }

        }
        else
        {
            // The underlying queue may have been aborted
            if (m_serialDataHandler->IsConnected() == false)
            {
                runPacketHandler = false;
                m_incomingDataQueue.FlushQueue();

                m_isConnected = false;
                m_messageQueue->SignalAbort();
            }
        }


    }
}

*/
