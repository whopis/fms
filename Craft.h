#ifndef CRAFT_H
#define CRAFT_H

#include "CraftData.h"
#include "CraftConnection.h"





class Craft
{
    public:
        Craft();
        virtual ~Craft();

        CraftData Data;

        void AttachConnection(CraftConnection* craftConnection);

        unsigned int GetHeartbeatCount();
        bool DataStreamValid();
        void RequestDataStream();

        void SendRcOverridesLow(uint16_t rc1, uint16_t rc2, uint16_t rc3, uint16_t rc4, uint16_t rc5, uint16_t rc6, uint16_t rc7, uint16_t rc8);
        void SendRcOverridesLow(uint16_t rc[]);
        bool VerifyRcOverridesLow(uint16_t rc[]);
        bool SendAndVerifyRcOverridesLow(uint16_t rc[], uint16_t timeout_ms);

        void SendRcOverridesHigh(uint16_t rc1, uint16_t rc2, uint16_t rc3, uint16_t rc4, uint16_t rc5, uint16_t rc6, uint16_t rc7, uint16_t rc8);
        void SendRcOverridesHigh(uint16_t rc[]);
        bool VerifyRcOverridesHigh(uint16_t rc[]);
        bool SendAndVerifyRcOverridesHigh(uint16_t rc[], uint16_t timeout_ms);


        void SendArm();
        void SendDisarm();

        void SendTerminateFlight();

        void SendMissionItem(float latitude, float longitude, float altitude);
        bool MissionItemAcknowledged();

        void SendSetPositionTargetGlobalInt(long latInt, long lonInt, float altitude);

        void SendAutopilotVersionRequest();
        void SendVersionBannerRequest();


        void SendTakeoff(float altitude);
        bool TakeoffAcknowledged();
        void SendLand();
        bool LandAcknowledged();

        void SendRegionOfInterest(float latitude, float longitude, float altitude);

        void UpdateBarometerBase(float pressAbs, int temperature);
        void UpdateBarometerBase2(float pressAbs, float pressAbs2, int temperature);
        void UpdateBarometerBase3(float pressAbs, float pressAbs2, float pressAbs3, int temperature);

        void UpdateYawResetAltitude(float alt);

        void SendResetFlightStats();

        bool IsConnectionActive(int maxSeconds);


        void SetGimbalTiltAngle(int angle);
        void AdjustGimbalTiltAngle(int deltaAngle);
        int  GetGimbalTiltAngle();
        int  GetGimbalTiltOutput();


        void SendParamRequestRead(char* param_id);
        
        bool IsInLandMode();
        
        void UpdateWpNavSpeedUp(float wpNavSpeedUp);



    protected:
        CraftConnection* m_craftConnection;

        int m_gimbalTiltAngle;


    private:

};

#endif // CRAFT_H


