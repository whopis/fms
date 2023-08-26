#ifndef HOLDPOINT_H
#define HOLDPOINT_H

#include <stdint.h>


class HoldPoint
{
    public:
        HoldPoint();
        virtual ~HoldPoint();

        void ResetHoldPoint(float craftLat, float craftLon, float groundLat, float groundLon);
        void ResetHoldPoint(int32_t craftLat, int32_t craftLon, int32_t groundLat, int32_t groundLon);

        void UpdateGroundPoint(float groundLat, float groundLon);
        void UpdateGroundPoint(int32_t groundLat, int32_t groundLon);


        float CraftHoldLatitude() { return m_craftCurrentHoldLatitude; }
        float CraftHoldLongitude() { return m_craftCurrentHoldLongitude; }

        long CraftHoldLatitudeInt() { return (long) (m_craftCurrentHoldLatitude * 1e7); }
        long CraftHoldLongitudeInt() { return (long) (m_craftCurrentHoldLongitude * 1e7); }

        float CraftStartLatitude() { return m_craftStartLatitude; }
        float CraftStartLongitude() { return m_craftStartLongitude; }

        float CraftTargetAltitude() { return m_craftTargetAltitude; }

        void UpdateCraftPosition(float fwd, float right, int heading);

        void SetCraftPosition(float lat, float lon);


        void SetFloorCeiling(float floor, float ceiling);
        void AdjustAltitude(float deltaAltitude);
        void SetAltitude(float altitude);

        void SetToHomePosition();

        void SetFollowMode(bool inFollow);
        bool m_noLocationLogging;
        
        void SetPowerLimitCeiling(float ceiling);
        void SetPowerLimitCeilingMin(float min);
        
        float GetFlightCeiling() { return m_flightCeiling; }
        float GetPowerLimitCeiling() { return m_powerLimitCeiling; }
        
        void SetGpsDeniedCeiling(float ceiling);
        void SetGpsDeniedMode(bool enabled);
        bool GetGpsDeniedMode();
        

    protected:
    private:

        bool    m_inFollowMode;

        float   m_craftStartLatitude;
        float   m_craftStartLongitude;

        float   m_groundStartLatitude;
        float   m_groundStartLongitude;

        float   m_craftCurrentHoldLatitude;
        float   m_craftCurrentHoldLongitude;

        float   m_deltaLatitude;
        float   m_deltaLongitude;


        float   m_flightFloor;
        float   m_flightCeiling;
        float   m_flightCeilingFollow;

        float   m_craftTargetAltitude;

        float   m_translateDeltaLatMeters;
        float   m_translateDeltaLonMeters;
        
        
        float   m_powerLimitCeiling;
        float   m_powerLimitCeilingMin;
        
        bool    m_inGpsDeniedMode;
        float   m_gpsDeniedCeiling;


};

#endif // HOLDPOINT_H
