#ifndef GIMBALSTATUS_H
#define GIMBALSTATUS_H

#include <stdint.h>


class GimbalStatus
{
    public:
        GimbalStatus();
        virtual ~GimbalStatus();


        void SetGimbalPosition(int32_t lat, int32_t lon, int32_t alt, uint16_t hdg, uint16_t tiltRcValue);
        void SetGimbalZoom(int zoomLevel);

        int LoadCamera(char *filename, int len);
        float GetMagScaling(int index);

        float	m_latitude;
        float 	m_longitude;
        float 	m_relativeAltitude;
        float 	m_heading;
        float 	m_tiltAngle;
        float 	m_magFactor;
        float 	m_hFoVcurrent;
        float 	m_vFoVcurrent;
        float 	m_hFoVfull;
        float 	m_vFoVfull;
        char    m_cameraFilename[256];



    protected:
    private:

        float 	CalculateMagFactorFromZoomLevel(int zoomLevel);
        float 	m_magScaling[11];

};

#endif // GIMBALSTATUS_H

