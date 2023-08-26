#include "GimbalStatus.h"

#include <cmath>

#include <syslog.h>

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/writer.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"

using namespace rapidjson;

GimbalStatus::GimbalStatus()
{
    //ctor

    m_magScaling[0] = 1.0f;
    m_magScaling[1] = 1.335f;
    m_magScaling[2] = 1.894f;
    m_magScaling[3] = 2.732f;
    m_magScaling[4] = 4.287f;
    m_magScaling[5] = 7.371f;
    m_magScaling[6] = 9.048f;
    m_magScaling[7] = 11.059f;
    m_magScaling[8] = 15.739f;
    m_magScaling[9] = 25.582f;
    m_magScaling[10] = 89.074f;

    m_hFoVfull = 54.0f;
    m_vFoVfull = 31.0f;

    memset(m_cameraFilename, 0x00, sizeof(m_cameraFilename));

}

GimbalStatus::~GimbalStatus()
{
    //dtor
}


void GimbalStatus::SetGimbalPosition(int32_t lat, int32_t lon, int32_t alt, uint16_t hdg, uint16_t tiltRcValue)
{
	m_latitude = (float) lat / 10000000.0f;
	m_longitude = (float) lon / 10000000.0f;
	m_relativeAltitude = (float) alt / 1000.0f;
	m_heading = (float) hdg / 100.0f;
	m_tiltAngle = ((float) (tiltRcValue - 1000)) * 90.0f / 1000.0f;
}


void GimbalStatus::SetGimbalZoom(int zoomLevel)
{
	m_magFactor = CalculateMagFactorFromZoomLevel(zoomLevel);
	m_hFoVcurrent = m_hFoVfull / m_magFactor;
	m_vFoVcurrent = m_vFoVfull / m_magFactor;
}


float GimbalStatus::CalculateMagFactorFromZoomLevel(int zoomLevel)
{
	// convert to a 0 => 10 scale
	float zoomStep = (float) zoomLevel / 20.0f;

	// get the integer and fractional part
	int zoomStepWhole = (int) trunc(zoomStep);
	float zoomStepFraction = zoomStep - zoomStepWhole;

	// if the integer is out of bounds, return 1.0f
	if (zoomStepWhole < 0)
		return 1.0f;
	if (zoomStepWhole > 10)
		return 1.0f;

	// if the integer is 10 (max zoom), return that value
	if (zoomStepWhole == 10)
		return m_magScaling[10];

	// else return the integer value plus the fractional portion
	float magFactor;
	magFactor = m_magScaling[zoomStepWhole];
	magFactor += (m_magScaling[zoomStepWhole + 1] - m_magScaling[zoomStepWhole]) * zoomStepFraction;
	return magFactor;
}

int GimbalStatus::LoadCamera(char *filename, int len)
{
    int retVal = 0;


    //return -1;
        // Always just fail until cause for rapidJson errors (seg fault, etc) are fixed



    if ((filename != NULL) && (len > 0))
    {
        memcpy(m_cameraFilename, filename, len);

        FILE *fp = fopen(m_cameraFilename, "r");
        char readBuffer[65535];
        Document document;
        if (fp)
        {
            FileReadStream is(fp, readBuffer, sizeof(readBuffer));
            document.ParseStream(is);
            //fclose(fp);
        }

        if (document.IsObject())
        {
            if (document["zoom table"].IsObject())
            {
                const Value& zt = document["zoom table"];
                int arraySize = 0;
                if (zt["array size"].IsInt() && zt["mag array"].IsArray())
                {
                    arraySize = zt["array size"].GetInt();
                    if (arraySize <= 11)
                    {
                        const Value& magArray = zt["mag array"];
                        for (SizeType i=0; i<magArray.Size(); i++)
                        {
                            m_magScaling[i] = magArray[i].GetDouble();
                        }
                    }
                    else
                    {
                        // bad array size
                        retVal = -5;
                    }
                }
                else
                {
                    // bad mag array
                    retVal = -4;
                }
            }
            else
            {
                // bad zoom table object
                retVal = -3;
            }
        }
        else
        {
            // document is NOT an object
            retVal = -2;
        }

        if (fp != NULL)
        {
            fclose(fp);
        }

    }
    else
    {
        // can't open file
        retVal = -1;
    }

    return retVal;
}// end LoadCamera

float GimbalStatus::GetMagScaling(int index)
{
    if ((index >=0) && (index < 11))  // 11 for now!!!
    {
        return m_magScaling[index];
    }
    else
    {
        return 1.0f;
    }
}// end GetMagScaling

