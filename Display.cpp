#include "Display.h"

#include "Version.h"



Display::Display()
{
    //ctor
    m_initialized = false;

    m_craft = nullptr;

    time(&m_systemStartTime);

    counter = 10;
}

Display::~Display()
{
    //dtor
    endwin();
}


void Display::AttachCraft(Craft* craft, deviceConnectionState* craftConnectionState)
{
    m_craftConnectionState = craftConnectionState;
    m_craft = craft;
}

void Display::AttachController(Controller* controller, deviceConnectionState* controllerConnectionState)
{
    m_controllerConnectionState = controllerConnectionState;
    m_controller = controller;
}

void Display::AttachGroundGps(GroundGPS* groundGps, deviceConnectionState* groundGpsConnectionState)
{
    m_groundGpsConnectionState = groundGpsConnectionState;
    m_groundGps = groundGps;
}

void Display::AttachGroundBaro(GroundBaro* groundBaro, deviceConnectionState* groundBaroConnectionState)
{
    m_groundBaroConnectionState = groundBaroConnectionState;
    m_groundBaro = groundBaro;
}


void Display::AttachCommunicationsHub(CommunicationsHub* communicationsHub)
{
    m_communicationsHub = communicationsHub;
}

void Display::Initialize(systemState_t* systemState)
{    

    m_systemState = systemState;


    initscr();
    start_color();
    curs_set(0);

    init_pair(1, COLOR_YELLOW, COLOR_BLUE);
    init_pair(2, COLOR_WHITE, COLOR_BLACK);
    init_pair(3, COLOR_WHITE, COLOR_WHITE);
    init_pair(4, COLOR_WHITE, COLOR_GREEN);
    init_pair(5, COLOR_WHITE, COLOR_YELLOW);
    init_pair(6, COLOR_WHITE, COLOR_RED);


    // Title window
    m_titleWindow = newwin(1, 80, 0, 0);
    wmove(m_titleWindow, 0, 0);
    wattron(m_titleWindow, COLOR_PAIR(1));
    wattron(m_titleWindow, A_BOLD);
    wprintw(m_titleWindow, "         Hoverfly Technologies, Inc. - Flight Management System - %s              ", FMS_VERSION);
    wrefresh(m_titleWindow);

    // System state window
    m_systemWindow = newwin(10, 80, 1, 0);
    box(m_systemWindow, 0, 0);
    wmove(m_systemWindow, 1, 1);
    wattron(m_systemWindow, A_BOLD);
    wattron(m_systemWindow, COLOR_PAIR(2));
    wprintw(m_systemWindow, "SYSTEM STATUS       ");
    wattroff(m_systemWindow, COLOR_PAIR(2));
    wrefresh(m_systemWindow);


    // Communications Hub window
    m_communicationsHubWindow = newwin(16, 80, 11, 0);
    box(m_communicationsHubWindow, 0, 0);
    wmove(m_communicationsHubWindow, 1, 1);
    wattron(m_communicationsHubWindow, A_BOLD);
    wattron(m_communicationsHubWindow, COLOR_PAIR(2));
    wprintw(m_communicationsHubWindow, "COM HUB STATUS       ");
    wattroff(m_communicationsHubWindow, COLOR_PAIR(2));

    wattron(m_communicationsHubWindow, COLOR_PAIR(1));
    wmove(m_communicationsHubWindow, 6, 5);
    wprintw(m_communicationsHubWindow, "      Connections               ");
    wmove(m_communicationsHubWindow, 7, 5);
    wprintw(m_communicationsHubWindow, "          Total  Active  Stale  ");
    wattroff(m_communicationsHubWindow, COLOR_PAIR(1));
    wmove(m_communicationsHubWindow, 8, 5);
    wprintw(m_communicationsHubWindow, "Craft   :");
    wmove(m_communicationsHubWindow, 9, 5);
    wprintw(m_communicationsHubWindow, "Control :");
    wmove(m_communicationsHubWindow, 10, 5);
    wprintw(m_communicationsHubWindow, "GPS     :");
    wrefresh(m_communicationsHubWindow);





    // Craft window
    m_craftWindow = newwin(24, 80, 11, 80);
    box(m_craftWindow, 0, 0);
    wmove(m_craftWindow, 1, 1);
    wattron(m_craftWindow, A_BOLD);
    wattron(m_craftWindow, COLOR_PAIR(2));
    wprintw(m_craftWindow, "CRAFT STATUS         ");
    wattroff(m_craftWindow, COLOR_PAIR(2));
    wrefresh(m_craftWindow);

    wattron(m_craftWindow, COLOR_PAIR(1));

    wmove(m_craftWindow, 3, 5);
    wprintw(m_craftWindow, "    Connection State    ");

    wmove(m_craftWindow, 3, 30);
    wprintw(m_craftWindow, "       Arm State        ");

    wmove(m_craftWindow, 3, 55);
    wprintw(m_craftWindow, "    Heartbeat Count     ");



    wmove(m_craftWindow, 6, 5);
    wattron(m_craftWindow, COLOR_PAIR(1));
    wprintw(m_craftWindow, "       GPS Status       ");
    wattroff(m_craftWindow, COLOR_PAIR(1));
    wmove(m_craftWindow, 7, 5);
    wprintw(m_craftWindow, "Fix  :");
    wmove(m_craftWindow, 8, 5);
    wprintw(m_craftWindow, "#Sat :");
    wmove(m_craftWindow, 9, 5);
    wprintw(m_craftWindow, "HDoP :");
    wmove(m_craftWindow, 10, 5);
    wprintw(m_craftWindow, "VDoP :");

    wmove(m_craftWindow, 12, 5);
    wattron(m_craftWindow, COLOR_PAIR(1));
    wprintw(m_craftWindow, "         Power          ");
    wattroff(m_craftWindow, COLOR_PAIR(1));
    wmove(m_craftWindow, 13, 5);
    wprintw(m_craftWindow, "Volt :");
    wmove(m_craftWindow, 14, 5);
    wprintw(m_craftWindow, "Curr :");

    wmove(m_craftWindow, 16, 5);
    wattron(m_craftWindow, COLOR_PAIR(1));
    wprintw(m_craftWindow, "      Flight Mode       ");
    wattroff(m_craftWindow, COLOR_PAIR(1));




    wmove(m_craftWindow, 6, 30);
    wattron(m_craftWindow, COLOR_PAIR(1));
    wprintw(m_craftWindow, "        Position        ");
    wattroff(m_craftWindow, COLOR_PAIR(1));
    wmove(m_craftWindow, 7, 30);
    wprintw(m_craftWindow, "Lat  :");
    wmove(m_craftWindow, 8, 30);
    wprintw(m_craftWindow, "Lon  :");
    wmove(m_craftWindow, 9, 30);
    wprintw(m_craftWindow, "Alt  :");
    wmove(m_craftWindow, 10, 30);
    wprintw(m_craftWindow, "Hdg  :");

    wmove(m_craftWindow, 12, 30);
    wattron(m_craftWindow, COLOR_PAIR(1));
    wprintw(m_craftWindow, "         Baro           ");
    wattroff(m_craftWindow, COLOR_PAIR(1));
    wmove(m_craftWindow, 13, 30);
    wprintw(m_craftWindow, "Prs  :");
    wmove(m_craftWindow, 14, 30);
    wprintw(m_craftWindow, "Tmp  :");



    wmove(m_craftWindow, 6, 55);
    wattron(m_craftWindow, COLOR_PAIR(1));
    wprintw(m_craftWindow, "   RC In   ");
    wmove(m_craftWindow, 6, 67);
    wprintw(m_craftWindow, "   Servo   ");
    wattroff(m_craftWindow, COLOR_PAIR(1));
    wmove(m_craftWindow, 7, 55);
    wprintw(m_craftWindow, "1:");
    wmove(m_craftWindow, 8, 55);
    wprintw(m_craftWindow, "2:");
    wmove(m_craftWindow, 9, 55);
    wprintw(m_craftWindow, "3:");
    wmove(m_craftWindow, 10, 55);
    wprintw(m_craftWindow, "4:");
    wmove(m_craftWindow, 11, 55);
    wprintw(m_craftWindow, "5:");
    wmove(m_craftWindow, 12, 55);
    wprintw(m_craftWindow, "6:");
    wmove(m_craftWindow, 13, 55);
    wprintw(m_craftWindow, "7:");
    wmove(m_craftWindow, 14, 55);
    wprintw(m_craftWindow, "8:");
    wmove(m_craftWindow, 15, 55);
    wprintw(m_craftWindow, "9:");
    wmove(m_craftWindow, 16, 54);
    wprintw(m_craftWindow, "10:");
    wmove(m_craftWindow, 17, 54);
    wprintw(m_craftWindow, "11:");
    wmove(m_craftWindow, 18, 54);
    wprintw(m_craftWindow, "12:");
    wmove(m_craftWindow, 19, 54);
    wprintw(m_craftWindow, "13:");
    wmove(m_craftWindow, 20, 54);
    wprintw(m_craftWindow, "14:");
    wmove(m_craftWindow, 21, 54);
    wprintw(m_craftWindow, "15:");
    wmove(m_craftWindow, 22, 54);
    wprintw(m_craftWindow, "16:");

    wmove(m_craftWindow, 7, 67);
    wprintw(m_craftWindow, "1:");
    wmove(m_craftWindow, 8, 67);
    wprintw(m_craftWindow, "2:");
    wmove(m_craftWindow, 9, 67);
    wprintw(m_craftWindow, "3:");
    wmove(m_craftWindow, 10, 67);
    wprintw(m_craftWindow, "4:");

    wattroff(m_craftWindow, COLOR_PAIR(1));
    wrefresh(m_craftWindow);





    // Ground GPS window
    m_groundGPSWindow = newwin(12, 80, 27, 0);
    box(m_groundGPSWindow, 0, 0);
    wmove(m_groundGPSWindow, 1, 1);
    wattron(m_groundGPSWindow, A_BOLD);
    wattron(m_groundGPSWindow, COLOR_PAIR(2));
    wprintw(m_groundGPSWindow, "GROUND GPS STATUS         ");
    wattroff(m_groundGPSWindow, COLOR_PAIR(2));
    wrefresh(m_groundGPSWindow);


    wattron(m_groundGPSWindow, COLOR_PAIR(1));

    wmove(m_groundGPSWindow, 3, 5);
    wprintw(m_groundGPSWindow, "    Connection State    ");


    wmove(m_groundGPSWindow, 3, 55);
    wprintw(m_groundGPSWindow, "    Heartbeat Count     ");



    wmove(m_groundGPSWindow, 6, 5);
    wattron(m_groundGPSWindow, COLOR_PAIR(1));
    wprintw(m_groundGPSWindow, "       GPS Status       ");
    wattroff(m_groundGPSWindow, COLOR_PAIR(1));
    wmove(m_groundGPSWindow, 7, 5);
    wprintw(m_groundGPSWindow, "Fix  :");
    wmove(m_groundGPSWindow, 8, 5);
    wprintw(m_groundGPSWindow, "#Sat :");
    wmove(m_groundGPSWindow, 9, 5);
    wprintw(m_groundGPSWindow, "HDoP :");
    wmove(m_groundGPSWindow, 10, 5);
    wprintw(m_groundGPSWindow, "VDoP :");



    wmove(m_groundGPSWindow, 6, 30);
    wattron(m_groundGPSWindow, COLOR_PAIR(1));
    wprintw(m_groundGPSWindow, "        Position        ");
    wattroff(m_groundGPSWindow, COLOR_PAIR(1));
    wmove(m_groundGPSWindow, 7, 30);
    wprintw(m_groundGPSWindow, "Lat  :");
    wmove(m_groundGPSWindow, 8, 30);
    wprintw(m_groundGPSWindow, "Lon  :");
    wmove(m_groundGPSWindow, 9, 30);
    wprintw(m_groundGPSWindow, "Alt  :");
    wmove(m_groundGPSWindow, 10, 30);
    wprintw(m_groundGPSWindow, "Hdg  :");


    wmove(m_groundGPSWindow, 6, 55);
    wattron(m_groundGPSWindow, COLOR_PAIR(1));
    wprintw(m_groundGPSWindow, "          Baro          ");
    wattroff(m_groundGPSWindow, COLOR_PAIR(1));
    wmove(m_groundGPSWindow, 7, 55);
    wprintw(m_groundGPSWindow, "Prs  :");
    wmove(m_groundGPSWindow, 8, 55);
    wprintw(m_groundGPSWindow, "Tmp  :");



    wattroff(m_groundGPSWindow, COLOR_PAIR(1));
    wrefresh(m_groundGPSWindow);





    // Controller window
    m_controllerWindow = newwin(12, 80, 39, 0);
    box(m_controllerWindow, 0, 0);
    wmove(m_controllerWindow, 1, 1);
    wattron(m_controllerWindow, A_BOLD);
    wattron(m_controllerWindow, COLOR_PAIR(2));
    wprintw(m_controllerWindow, "CONTROLLER STATUS         ");
    wattroff(m_controllerWindow, COLOR_PAIR(2));
    wrefresh(m_controllerWindow);


    wattron(m_controllerWindow, COLOR_PAIR(1));

    wmove(m_controllerWindow, 3, 5);
    wprintw(m_controllerWindow, "    Connection State    ");


    wmove(m_controllerWindow, 3, 55);
    wprintw(m_controllerWindow, "    Heartbeat Count     ");


    wattroff(m_controllerWindow, COLOR_PAIR(1));


    wmove(m_controllerWindow, 6, 5);
    wattron(m_controllerWindow, COLOR_PAIR(1));
    wprintw(m_controllerWindow, "        Joystick        ");
    wattroff(m_controllerWindow, COLOR_PAIR(1));
    wmove(m_controllerWindow, 7, 5);
    wprintw(m_controllerWindow, "X    :");
    wmove(m_controllerWindow, 8, 5);
    wprintw(m_controllerWindow, "Y    :");
    wmove(m_controllerWindow, 9, 5);
    wprintw(m_controllerWindow, "Z    :");

    wrefresh(m_controllerWindow);




    // Ground baro window
    m_groundBaroWindow = newwin(12, 80, 39, 80);
    box(m_groundBaroWindow, 0, 0);
    wmove(m_groundBaroWindow, 1, 1);
    wattron(m_groundBaroWindow, A_BOLD);
    wattron(m_groundBaroWindow, COLOR_PAIR(2));
    wprintw(m_groundBaroWindow, "GROUND BARO STATUS        ");
    wattroff(m_groundBaroWindow, COLOR_PAIR(2));
    wrefresh(m_groundBaroWindow);


    wattron(m_groundBaroWindow, COLOR_PAIR(1));

    wmove(m_groundBaroWindow, 3, 5);
    wprintw(m_groundBaroWindow, "    Connection State    ");


    wmove(m_groundBaroWindow, 3, 55);
    wprintw(m_groundBaroWindow, "    Heartbeat Count     ");


    wattroff(m_groundBaroWindow, COLOR_PAIR(1));


    wmove(m_groundBaroWindow, 6, 5);
    wattron(m_groundBaroWindow, COLOR_PAIR(1));
    wprintw(m_groundBaroWindow, "          Baro          ");
    wattroff(m_groundBaroWindow, COLOR_PAIR(1));
    wmove(m_groundBaroWindow, 7, 5);
    wprintw(m_groundBaroWindow, "Press:");
    wmove(m_groundBaroWindow, 8, 5);
    wprintw(m_groundBaroWindow, "Temp :");

    wrefresh(m_groundBaroWindow);

    m_initialized = true;

}


void Display::Update()
{

    if (m_initialized == false)
        return;

    UpdateSystemStatus();
    UpdateCraft();
    UpdateGroundGps();
    UpdateController();
    UpdateCommunicationsHub();
    UpdateGroundBaro();


}

void Display::UpdateCommunicationsHub()
{
    wmove(m_communicationsHubWindow, 8, 16);
    wprintw(m_communicationsHubWindow, "                                       ");
    wmove(m_communicationsHubWindow, 9, 16);
    wprintw(m_communicationsHubWindow, "                                       ");
    wmove(m_communicationsHubWindow, 10, 16);
    wprintw(m_communicationsHubWindow, "                                       ");


    wmove(m_communicationsHubWindow, 8, 16);
    wprintw(m_communicationsHubWindow, "%d",m_communicationsHub->CraftManager.DeviceCount());
    wmove(m_communicationsHubWindow, 9, 16);
    wprintw(m_communicationsHubWindow, "%d",m_communicationsHub->ControllerManager.DeviceCount());
    wmove(m_communicationsHubWindow, 10, 16);
    wprintw(m_communicationsHubWindow, "%d",m_communicationsHub->GroundGPSManager.DeviceCount());

    wmove(m_communicationsHubWindow, 8, 23);
    wprintw(m_communicationsHubWindow, "%d",m_communicationsHub->CraftManager.ActiveCount(5));
    wmove(m_communicationsHubWindow, 9, 23);
    wprintw(m_communicationsHubWindow, "%d",m_communicationsHub->ControllerManager.ActiveCount(5));
    wmove(m_communicationsHubWindow, 10, 23);
    wprintw(m_communicationsHubWindow, "%d",m_communicationsHub->GroundGPSManager.ActiveCount(5));

    wmove(m_communicationsHubWindow, 8, 30);
    wprintw(m_communicationsHubWindow, "%d",m_communicationsHub->CraftManager.StaleCount(5));
    wmove(m_communicationsHubWindow, 9, 30);
    wprintw(m_communicationsHubWindow, "%d",m_communicationsHub->ControllerManager.StaleCount(5));
    wmove(m_communicationsHubWindow, 10, 30);
    wprintw(m_communicationsHubWindow, "%d",m_communicationsHub->GroundGPSManager.StaleCount(5));


    wrefresh(m_communicationsHubWindow);


}


void Display::UpdateSystemStatus()
{
    wmove(m_systemWindow, 1, 60);
    wattron(m_systemWindow, COLOR_PAIR(3));
    wprintw(m_systemWindow, "                   ");
    wattroff(m_systemWindow, COLOR_PAIR(3));


    time_t currentTime;
    time(&currentTime);
    wmove(m_systemWindow, 3, 60);
    wprintw(m_systemWindow, "Time : %d", (currentTime - m_systemStartTime));


    wmove(m_systemWindow, 3, 5);
    wprintw(m_systemWindow, "System State     : ");
    wprintSystemState(m_systemWindow);
    wrefresh(m_systemWindow);
}



void Display::UpdateGroundGps()
{
    generalState_t groundGpsState = uninitialized;


    wmove(m_groundGPSWindow, 4, 5);
    switch (*m_groundGpsConnectionState)
    {
        case noConnection:
            wprintw(m_groundGPSWindow, "Not detected             ");
            break;
        case socketConnected:
            wprintw(m_groundGPSWindow, "Socket connected         ");
            groundGpsState = warning;
            break;
        case requestingDataStream:
            wprintw(m_groundGPSWindow, "Requesting data stream   ");
            groundGpsState = warning;
            break;
        case waitingForValidData:
            wprintw(m_groundGPSWindow, "Waiting for valid data   ");
            groundGpsState = warning;
            break;
        case initializingChannels:
            wprintw(m_groundGPSWindow, "Initializing channels    ");
            groundGpsState = warning;
            break;
        case connected:
            wprintw(m_groundGPSWindow, "Connected                ");
            groundGpsState = good;
            break;
        case lostConnection:
            wprintw(m_groundGPSWindow, "Lost connection          ");
            groundGpsState = warning;
            break;
    }
    DisplayStatus(m_groundGPSWindow, groundGpsState);


    // Heartbeat count
    wmove(m_groundGPSWindow, 4, 55);
    wprintw(m_groundGPSWindow, "       ");
    wmove(m_groundGPSWindow, 4, 55);
    wprintw(m_groundGPSWindow, "%d", m_groundGps->GetHeartbeatCount());

    wmove(m_groundGPSWindow, 7, 12);
    wprintw(m_groundGPSWindow, "                 ");
    wmove(m_groundGPSWindow, 8, 12);
    wprintw(m_groundGPSWindow, "                 ");
    wmove(m_groundGPSWindow, 9, 12);
    wprintw(m_groundGPSWindow, "                 ");
    wmove(m_groundGPSWindow, 10, 12);
    wprintw(m_groundGPSWindow, "                 ");

    wmove(m_groundGPSWindow, 7, 12);
    wprintw(m_groundGPSWindow, "                 ");
    wmove(m_groundGPSWindow, 8, 37);
    wprintw(m_groundGPSWindow, "                 ");
    wmove(m_groundGPSWindow, 9, 37);
    wprintw(m_groundGPSWindow, "                 ");
    wmove(m_groundGPSWindow, 10, 37);
    wprintw(m_groundGPSWindow, "                 ");

    wmove(m_groundGPSWindow, 7, 60);
    wprintw(m_groundGPSWindow, "                 ");
    wmove(m_groundGPSWindow, 8, 60);
    wprintw(m_groundGPSWindow, "                 ");


    // GPS data
    wmove(m_groundGPSWindow, 7, 12);
    wprintw(m_groundGPSWindow, "%d", m_groundGps->Data.gpsFixType);
    wmove(m_groundGPSWindow, 8, 12);
    wprintw(m_groundGPSWindow, "%d", m_groundGps->Data.gpsNumberSatellites);
    wmove(m_groundGPSWindow, 9, 12);
    wprintw(m_groundGPSWindow, "%2.2f", m_groundGps->Data.gpsHdop / 100.0);
    wmove(m_groundGPSWindow, 10, 12);
    wprintw(m_groundGPSWindow, "%2.2f", m_groundGps->Data.gpsVdop / 100.0);


    // Position data
    wmove(m_groundGPSWindow, 7, 37);
    wprintw(m_groundGPSWindow, "%3.7f", m_groundGps->Data.latitude / 10000000.0);
    wmove(m_groundGPSWindow, 8, 37);
    wprintw(m_groundGPSWindow, "%3.7f", m_groundGps->Data.longitude / 10000000.0);
    wmove(m_groundGPSWindow, 9, 37);
    wprintw(m_groundGPSWindow, "%d", m_groundGps->Data.relative_alt);
    wmove(m_groundGPSWindow, 10, 37);
    wprintw(m_groundGPSWindow, "%d", m_groundGps->Data.heading);

    // GPS Baro data
    wmove(m_groundGPSWindow, 7, 60);
    wprintw(m_groundGPSWindow, "%3.7f", m_groundGps->Data.pressAbs);
    wmove(m_groundGPSWindow, 8, 60);
    wprintw(m_groundGPSWindow, "%d", m_groundGps->Data.temperature);

     wrefresh(m_groundGPSWindow);


}


void Display::UpdateController()
{
    generalState_t controllerState = uninitialized;


    wmove(m_controllerWindow, 4, 5);
    switch (*m_controllerConnectionState)
    {
        case noConnection:
            wprintw(m_controllerWindow, "Not detected             ");
            break;
        case socketConnected:
            wprintw(m_controllerWindow, "Socket connected         ");
            controllerState = warning;
            break;
        case requestingDataStream:
            wprintw(m_controllerWindow, "Requesting data stream   ");
            controllerState = warning;
            break;
        case waitingForValidData:
            wprintw(m_controllerWindow, "Waiting for valid data   ");
            controllerState = warning;
            break;
        case initializingChannels:
            wprintw(m_controllerWindow, "Initializing channels    ");
            controllerState = warning;
            break;
        case connected:
            wprintw(m_controllerWindow, "Connected                ");
            controllerState = good;
            break;
        case lostConnection:
            wprintw(m_controllerWindow, "Lost connection          ");
            controllerState = warning;
            break;
    }
    DisplayStatus(m_controllerWindow, controllerState);


        // Heartbeat count
    wmove(m_controllerWindow, 4, 55);
    wprintw(m_controllerWindow, "     ");
    wmove(m_controllerWindow, 4, 55);
    wprintw(m_controllerWindow, "%d", m_controller->GetHeartbeatCount());


    wmove(m_controllerWindow, 7, 12);
    wprintw(m_controllerWindow, "     ");
    wmove(m_controllerWindow, 8, 12);
    wprintw(m_controllerWindow, "     ");
    wmove(m_controllerWindow, 9, 12);
    wprintw(m_controllerWindow, "     ");

    wmove(m_controllerWindow, 7, 12);
    wprintw(m_controllerWindow, "%03d", m_controller->Data.JoyX);
    wmove(m_controllerWindow, 8, 12);
    wprintw(m_controllerWindow, "%03d", m_controller->Data.JoyY);
    wmove(m_controllerWindow, 9, 12);
    wprintw(m_controllerWindow, "%03d", m_controller->Data.JoyZ);

    wrefresh(m_controllerWindow);
}



void Display::UpdateGroundBaro()
{
    generalState_t groundBaroState = uninitialized;


    wmove(m_groundBaroWindow, 4, 5);
    switch (*m_groundBaroConnectionState)
    {
        case noConnection:
            wprintw(m_groundBaroWindow, "Not detected             ");
            break;
        case socketConnected:
            wprintw(m_groundBaroWindow, "Socket connected         ");
            groundBaroState = warning;
            break;
        case requestingDataStream:
            wprintw(m_groundBaroWindow, "Requesting data stream   ");
            groundBaroState = warning;
            break;
        case waitingForValidData:
            wprintw(m_groundBaroWindow, "Waiting for valid data   ");
            groundBaroState = warning;
            break;
        case initializingChannels:
            wprintw(m_groundBaroWindow, "Initializing channels    ");
            groundBaroState = warning;
            break;
        case connected:
            wprintw(m_groundBaroWindow, "Connected                ");
            groundBaroState = good;
            break;
        case lostConnection:
            wprintw(m_groundBaroWindow, "Lost connection          ");
            groundBaroState = warning;
            break;
    }
    DisplayStatus(m_groundBaroWindow, groundBaroState);


        // Heartbeat count
    wmove(m_groundBaroWindow, 4, 55);
    wprintw(m_groundBaroWindow, "     ");
    wmove(m_groundBaroWindow, 4, 55);
    wprintw(m_groundBaroWindow, "%d", m_groundBaro->GetHeartbeatCount());


    wmove(m_groundBaroWindow, 7, 12);
    wprintw(m_groundBaroWindow, "            ");
    wmove(m_groundBaroWindow, 8, 12);
    wprintw(m_groundBaroWindow, "            ");

    wmove(m_groundBaroWindow, 7, 12);
    wprintw(m_groundBaroWindow, "%3.7f", m_groundBaro->Data.pressAbs);
    wmove(m_groundBaroWindow, 8, 12);
    wprintw(m_groundBaroWindow, "%d", m_groundBaro->Data.temperature);

    wrefresh(m_groundBaroWindow);
}



void Display::UpdateCraft()
{
    generalState_t craftState = uninitialized;


    wmove(m_craftWindow, 4, 5);
    switch (*m_craftConnectionState)
    {
        case noConnection:
            wprintw(m_craftWindow, "Not detected             ");
            break;
        case socketConnected:
            wprintw(m_craftWindow, "Socket connected         ");
            craftState = warning;
            break;
        case requestingDataStream:
            wprintw(m_craftWindow, "Requesting data stream   ");
            craftState = warning;
            break;
        case waitingForValidData:
            wprintw(m_craftWindow, "Waiting for valid data   ");
            craftState = warning;
            break;
        case initializingChannels:
            wprintw(m_craftWindow, "Initializing channels    ");
            craftState = warning;
            break;
        case connected:
            wprintw(m_craftWindow, "Connected                ");
            craftState = good;
            break;
        case lostConnection:
            wprintw(m_craftWindow, "Lost connection          ");
            craftState = warning;
            break;
    }
    DisplayStatus(m_craftWindow, craftState);

    // Armed state
    wmove(m_craftWindow, 4, 30);
    if (m_craft->Data.armed == true)
        wprintw(m_craftWindow, "ARMED   ");
    else
        wprintw(m_craftWindow, "DISARMED");

    // Heartbeat count
    wmove(m_craftWindow, 4, 55);
    wprintw(m_craftWindow, "        ");
    wmove(m_craftWindow, 4, 55);
    wprintw(m_craftWindow, "%d", m_craft->GetHeartbeatCount());




    wmove(m_craftWindow, 7, 12);
    wprintw(m_craftWindow, "                 ");
    wmove(m_craftWindow, 8, 12);
    wprintw(m_craftWindow, "                 ");
    wmove(m_craftWindow, 9, 12);
    wprintw(m_craftWindow, "                 ");
    wmove(m_craftWindow, 10, 12);
    wprintw(m_craftWindow, "                 ");

    wmove(m_craftWindow, 7, 12);
    wprintw(m_craftWindow, "                 ");
    wmove(m_craftWindow, 8, 37);
    wprintw(m_craftWindow, "                 ");
    wmove(m_craftWindow, 9, 37);
    wprintw(m_craftWindow, "                 ");
    wmove(m_craftWindow, 10, 37);
    wprintw(m_craftWindow, "                 ");


    wmove(m_craftWindow, 13, 37);
    wprintw(m_craftWindow, "                 ");
    wmove(m_craftWindow, 14, 37);
    wprintw(m_craftWindow, "                 ");


    // GPS data
    wmove(m_craftWindow, 7, 12);
    wprintw(m_craftWindow, "%d", m_craft->Data.gpsFixType);
    wmove(m_craftWindow, 8, 12);
    wprintw(m_craftWindow, "%d", m_craft->Data.gpsNumberSatellites);
    wmove(m_craftWindow, 9, 12);
    wprintw(m_craftWindow, "%2.2f", m_craft->Data.gpsHdop / 100.0);
    wmove(m_craftWindow, 10, 12);
    wprintw(m_craftWindow, "%2.2f", m_craft->Data.gpsVdop / 100.0);


    // Position data
    wmove(m_craftWindow, 7, 37);
    wprintw(m_craftWindow, "%3.7f", m_craft->Data.latitude / 10000000.0);
    wmove(m_craftWindow, 8, 37);
    wprintw(m_craftWindow, "%3.7f", m_craft->Data.longitude / 10000000.0);
    wmove(m_craftWindow, 9, 37);
    wprintw(m_craftWindow, "%d", m_craft->Data.relative_alt);
    wmove(m_craftWindow, 10, 37);
    wprintw(m_craftWindow, "%d", m_craft->Data.heading);


    wmove(m_craftWindow, 13, 12);
    wprintw(m_craftWindow, "             ");
    wmove(m_craftWindow, 14, 12);
    wprintw(m_craftWindow, "             ");


    // Power data
    wmove(m_craftWindow, 13, 12);
    wprintw(m_craftWindow, "%3.1f", m_craft->Data.batteryVoltage / 1000.0);
    wmove(m_craftWindow, 14, 12);
    wprintw(m_craftWindow, "%3.1f", m_craft->Data.batteryCurrent / 100.0);

    // Baro data
    wmove(m_craftWindow, 13, 37);
    wprintw(m_craftWindow, "%3.7f", m_craft->Data.pressAbs);
    wmove(m_craftWindow, 14, 37);
    wprintw(m_craftWindow, "%d", m_craft->Data.temperature);


    // Flight mode
    wmove(m_craftWindow, 17, 12);
    switch (m_craft->Data.custom_mode)
    {
        case flightmode_stabilize:
            wprintw(m_craftWindow, "STABILIZE  ");
            break;
        case flightmode_acro:
            wprintw(m_craftWindow, "ACRO       ");
            break;
        case flightmode_alt_hold:
            wprintw(m_craftWindow, "ALT_HOLD   ");
            break;
        case flightmode_auto:
            wprintw(m_craftWindow, "AUTO       ");
            break;
        case flightmode_guided:
            wprintw(m_craftWindow, "GUIDED     ");
            break;
        case flightmode_loiter:
            wprintw(m_craftWindow, "LOITER     ");
            break;
        case flightmode_rtl:
            wprintw(m_craftWindow, "RTL        ");
            break;
        case flightmode_circle:
            wprintw(m_craftWindow, "CIRCLE     ");
            break;
        case flightmode_land:
            wprintw(m_craftWindow, "LAND       ");
            break;
        case flightmode_of_loiter:
            wprintw(m_craftWindow, "OF_LOITER  ");
            break;
        case flightmode_drift:
            wprintw(m_craftWindow, "DRIFT      ");
            break;
        case flightmode_sport:
            wprintw(m_craftWindow, "SPORT      ");
            break;
        case flightmode_flip:
            wprintw(m_craftWindow, "FLIP       ");
            break;
        case flightmode_autotune:
            wprintw(m_craftWindow, "AUTOTUNE   ");
            break;
        case flightmode_poshold:
            wprintw(m_craftWindow, "POSHOLD    ");
            break;
        default:
            wprintw(m_craftWindow, "UNKNOWN    ");
            break;
    }




    // RC channels
    for (int i = 0; i < 16; i++)
    {
        wmove(m_craftWindow, 7 + i, 58);
        wprintw(m_craftWindow, "%04d", m_craft->Data.rcChannels[i]);
    }

    for (int i = 0; i < 4; i++)
    {
        wmove(m_craftWindow, 7 + i, 70);
        wprintw(m_craftWindow, "%04d", m_craft->Data.servoOutputs[i]);
    }


    wrefresh(m_craftWindow);


}


void Display::DisplayStatus(WINDOW* win, generalState_t state)
{
    int colorPair;
    switch (state)
    {
        case uninitialized:
            colorPair = 3;
            break;
        case good:
            colorPair = 4;
            break;
        case warning:
            colorPair = 5;
            break;
        case critical:
            colorPair = 6;
            break;
        default:
            colorPair = 3;
            break;
    }

    wmove(win, 1, 60);
    wattron(win, COLOR_PAIR(colorPair));
    wprintw(win, "                   ");
    wattroff(win, COLOR_PAIR(colorPair));
}


void Display::wprintSystemState(WINDOW* win)
{
    switch (*m_systemState)
    {

        case demoMode:
            wprintw(win, "Demo mode                  ");
            break;

        case killCraft:
            wprintw(win, "KILL SWITCH ACTIVATED      ");
            break;

        case notSet:
            wprintw(win, "Not Set                    ");
            break;

        case startup:
            wprintw(win, "Startup                    ");
            break;

        case connectToController:
            wprintw(win, "Connect to Controller      ");
            break;

        case connectToSkyBox:
            wprintw(win, "Connect to SkyBox          ");
            break;

        case connectToCraft:
            wprintw(win, "Connect to Craft           ");
            break;

        case connectToGroundGPS:
            wprintw(win, "Connect to Ground          ");
            break;

        case connectToGroundBaro:
            wprintw(win, "Connect to Ground Baro     ");
            break;

        case acquiringCraftGPS:
            wprintw(win, "Acquiring Craft GPS        ");
            break;

        case acquiringControllerGPS:
            wprintw(win, "Acquiring Controller GPS   ");
            break;


        case readySkyBoxClosed:
            wprintw(win, "Ready SkyBox Closed        ");
            break;

        case closingSkyBoxFromReady:
            wprintw(win, "Closing SkyBox From Ready  ");
            break;

        case readyToArm:
            wprintw(win, "Ready to Arm               ");
            break;

        case arming:
            wprintw(win, "Arming                     ");
            break;

        case armed:
            wprintw(win, "Armed                      ");
            break;

        case launch:
            wprintw(win, "Launching                  ");
            break;

        case flightHold:
            wprintw(win, "Camera mode                ");
            break;

        case flightTranslate:
            wprintw(win, "Translate mode             ");
            break;

        case flightFollow:
            wprintw(win, "Follow Mode                ");
            break;

        case land:
            wprintw(win, "Landing                    ");
            break;

        case failsafe:
            wprintw(win, "FAILSAFE                   ");
            break;

        case safetyStart:
            wprintw(win, "Safety Start               ");
            break;

        case lowVoltage:
            wprintw(win, "Low Voltage                ");
            break;

        case error:
            wprintw(win, "ERROR                      ");
            break;


    }


}

