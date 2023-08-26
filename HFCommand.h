#ifndef HFCOMMAND_H
#define HFCOMMAND_H

enum HFCommandType {
    HFCMD_ARM           = 0,
    HFCMD_DISARM        = 1,
    HFCMD_LAUNCH        = 2,
    HFCMD_LAND          = 3,
    HFCMD_CHANGE_ALT    = 4,
    HFCMD_FOLLOW        = 5,
    HFCMD_FLIGHTMODE    = 6,
    HFCMD_REPOSITION    = 7,
    HFCMD_MOUNT_CTRL    = 8,
    HFCMD_REMOTE_CTRL   = 9,
    HFCMD_NAVIGATE_TO_POS = 10,
    HFCMD_COLOR_PALETTE = 11,
    HFCMD_ZOOM          = 12,
    HFCMD_ZOOMSCALE     = 13,
    HFCMD_GO_TO_HOME    = 14,
    HFCMD_SKYBOX_OPEN   = 15,
    HFCMD_SKYBOX_CLOSE  = 16,
    HFCMD_LOCKER_OPEN   = 17,
    HFCMD_LOCKER_CLOSE  = 18,
    HFCMD_REEL_RESETPOS  = 210,
    HFCMD_PRECISION_LAND = 300,
    HFCMD_KILL_CRAFT    = 9999
};


enum HFFlightMode {
    HFMODE_HOLD         = 0,
    HFMODE_TRANSLATE    = 1
};

enum HFReferenceFrame {
    HFFRAME_CRAFT       = 0,
    HFFRAME_GROUND      = 1,
    HFFRAME_NED         = 2
};

class HFCommand
{
    public:
        HFCommand(HFCommandType commandType);
        virtual ~HFCommand();

        HFCommandType CommandType();

        bool Enable;

        HFFlightMode Mode;

        HFReferenceFrame Frame;

        float X;
        float Y;
        float Z;

        bool AbsoluteAltitude;

        int DeltaTilt;

        int PaletteCommand;

        int ZoomCommand;

        bool UseLandHeading;
        int LandHeading;


    protected:
    private:

        HFCommandType m_commandType;
};

#endif // HFCOMMAND_H
