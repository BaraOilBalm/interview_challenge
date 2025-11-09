#include <cstdint>
#include <cstring>
#include <optional>
#include <iostream>

namespace hw
{

    // not implemented, milliseconds after boot
    uint32_t now_ms();

    // print logs
    void log(const char *msg)
    {
        std::cout << msg << std::endl;
    }
    // battery info
    struct BatteryStatus
    {
        uint8_t percent;
        bool charging;
    };

    BatteryStatus read_battery(); // not implemented but returns batter info

    // waypoint location for station point or waiting point to find station
    bool navigate_to(const char *waypoint_name);

    bool dock_start();
    bool dock_is_complete();

    // rf stuff
    bool radio_join(uint16_t pan_id, uint8_t channel);
    bool radio_send(const void *data, size_t len);
    bool radio_recv(void *data, size_t len);

    struct BeaconInfo
    {
        uint16_t pan_id;
        uint8_t channel;
        int8_t rssi_dbm;
        bool occupied; // free or taken and for start and complete
    };

    bool radio_start();
    void radio_sleep(); // not implemented but to off RF
    bool radio_set_channel(uint8_t ch);
    bool radio_scan_start();

    std::optional<BeaconInfo> radio_scan_poll(); // not implemented, returns BeaconInfo items

    constexpr uint8_t kChannelsToScan[] = {11, 15, 20, 25}; // assuming only 4 channels used by all stations

}

// robot's thresholds
namespace cfg
{
    constexpr uint8_t LOW_BATT_PERCENT = 25;
    constexpr uint32_t SCAN_WINDOW_MS = 1500;
    constexpr uint8_t MAX_ASSOC_RETRIES = 3;
    constexpr char PRECHARGE_WP[] = "pre_charging_wp";
    constexpr char DOCK_APPROACH_WP[] = "dock_approach_wp";
}

enum class RState : uint8_t
{
    Idle,        // non-charging state
    GoPrecharge, // navigate to pre-charging waypoint
    ScanInit,    // start RF scan
    Scanning,    // collecting beacons
    PickStation, // choose the station with best signal strength
    AssocInit,   // send hello msgs
    WaitAccept,  // receive session token from station
    Docking,
    RTExchange,  // RT msg exchange with 50 bytes payload and checks missed msgs
    Undocking,   // charging complete, RT exchange occurs until undocking completes
    Bye,         // end session
    ErrorBackoff // resets chosen station, rescans, else transition back to Idle state
};

struct StationChoice
{
    bool valid{false};
    uint16_t pan_id{0};
    uint8_t channel{0};
    int8_t rssi_dbm{-127};
};

struct HelloMsg
{
    uint8_t robot_id;
    uint8_t hs_payload[100]; // simulation 100 bytes for handshake message
};

struct AcceptMsg
{
    uint16_t session_token;
    uint32_t start_time_ms;
    uint16_t period_ms;
};

struct RTDataMsg
{
    uint16_t session_token;
    uint8_t seq;
    uint8_t battery_percent;
    bool charging;
    uint32_t timestamp_ms;
    uint8_t rt_payload[50]; // simulate real time payload of 50 bytes
    bool docking;
    bool docked;
};

struct ByeMsg
{
    uint16_t session_token;
};

class Robot
{
public:
    void tick();

private:
    uint16_t session_token_ = 0;
    uint32_t period_ms_ = 80;
    uint8_t rt_seq_ = 0;
    uint8_t miss_count_ = 0;
    bool docking_flag_ = false;
    bool docked_flag_ = false;
    uint8_t dock_phase_ = 0;     // 0=approach, 1=docking
    uint32_t dock_begin_ms_ = 0; // for timeout if docking fails
    bool docking_started_ = false;
    bool charging_done_ = false;
    bool undocking_ = false;
    uint32_t last_tx_ms_ = 0;
    uint32_t last_rx_ms_ = 0;
    RState st_ = RState::Idle;
    uint32_t st_enter_ms_ = 0;

    struct Seen
    {
        uint16_t pan_id;
        uint8_t channel;
        int8_t rssi_dbm;
        bool occupied;
    };
    static constexpr size_t kMaxSeen = 16;
    Seen seen_[kMaxSeen];
    size_t seen_count_ = 0;

    // retries
    uint8_t assoc_retry_ = 0;

    // chosen station
    StationChoice chosen_{};

    void rt_heartbeat_tick()
    {
        uint32_t now = hw::now_ms();
        // sends msg every 80ms defined in period_ms_, a quasi-TDMA
        // ideally need to code for TDMA with time syncronisation from station side and time slots
        if (now - last_tx_ms_ >= period_ms_)
        {
            RTDataMsg msg{};
            msg.session_token = session_token_;
            msg.seq = rt_seq_++;
            msg.battery_percent = hw::read_battery().percent;
            msg.charging = docked_flag_; // used docked_flag here but can implement charging flag when battery receives voltage
            msg.docking = docking_flag_;
            msg.docked = docked_flag_;
            msg.timestamp_ms = now;
            hw::radio_send(&msg, sizeof(msg));
            last_tx_ms_ = now;
            hw::log("RT_DATA sent");
        }

        RTDataMsg reply{};
        if (hw::radio_recv(&reply, sizeof(reply)))
        { // radio_recv returns true if msg is received. its a stub function now, assuming it returns faster than 80ms
            if (reply.session_token != session_token_)
            {
                // session token changed for some reason, robot needs to go to error state
                transition(RState::ErrorBackoff);
                return;
            }
            else
            {
                last_rx_ms_ = now;
                miss_count_ = 0;
                hw::log("RT_DATA reply received");
            }
        }
        else
        {
            if (++miss_count_ > 3 && (now - last_rx_ms_) >= 100)
            { // checks for 3 consecutive missed msgs and within 100 ms interval and sends to error
                hw::log("Too many misses within 100ms, changing to error state");
                transition(RState::ErrorBackoff);
                return;
            }
        }
    }

    void transition(RState s)
    {
        st_ = s;
        st_enter_ms_ = hw::now_ms();
    }
    uint32_t since_enter() const { return hw::now_ms() - st_enter_ms_; }

    void clear_scan_buffer() { seen_count_ = 0; }

    void handle_idle();
    void handle_go_precharge();
    void handle_scan_init();
    void handle_scanning();
    void handle_pick_station();
    void handle_assoc_init();
    void handle_wait_accept();
    void handle_docking();
    void handle_rt_exchange();
    void handle_undocking();
    void handle_bye();
    void handle_error_backoff();
};

void Robot::tick()
{
    switch (st_)
    {
    case RState::Idle:
        handle_idle();
        break;
    case RState::GoPrecharge:
        handle_go_precharge();
        break;
    case RState::ScanInit:
        handle_scan_init();
        break;
    case RState::Scanning:
        handle_scanning();
        break;
    case RState::PickStation:
        handle_pick_station();
        break;
    case RState::AssocInit:
        handle_assoc_init();
        break;
    case RState::WaitAccept:
        handle_wait_accept();
        break;
    case RState::Docking:
        handle_docking();
        break;
    case RState::RTExchange:
        handle_rt_exchange();
        break;
    case RState::Undocking:
        handle_undocking();
        break;
    case RState::Bye:
        handle_bye();
        break;
    case RState::ErrorBackoff:
        handle_error_backoff();
        break;
    }
}

void Robot::handle_idle()
{
    docking_flag_ = false;
    docked_flag_ = false;
    auto b = hw::read_battery();
    if (b.percent <= cfg::LOW_BATT_PERCENT && !b.charging)
    {
        hw::log("Low battery going to pre-charge waypoint");
        assoc_retry_ = 0;
        transition(RState::GoPrecharge);
        return;
    }
}

void Robot::handle_go_precharge()
{
    if (hw::navigate_to(cfg::PRECHARGE_WP))
    {
        hw::log("Reached pre-charge waypoint");
        transition(RState::ScanInit);
    }
}

void Robot::handle_scan_init()
{
    clear_scan_buffer();
    chosen_ = StationChoice{}; // removes last chosen station
    if (!hw::radio_start())
    {
        hw::log("Radio init failed backing off");
        transition(RState::ErrorBackoff);
        return;
    }
    hw::radio_scan_start();
    hw::log("Scanning for stations");
    transition(RState::Scanning);
}

void Robot::handle_scanning()
{
    for (;;)
    {
        auto b = hw::radio_scan_poll();
        if (!b.has_value())
            break;

        if (seen_count_ < kMaxSeen)
        {
            seen_[seen_count_++] = {b->pan_id, b->channel, b->rssi_dbm, b->occupied};
        }
    }

    if (since_enter() >= cfg::SCAN_WINDOW_MS)
    {
        transition(RState::PickStation);
    }
}

// choose strongest signal
void Robot::handle_pick_station()
{
    int best_i = -1;
    int8_t best_rssi = -127;

    for (size_t i = 0; i < seen_count_; ++i)
    {
        if (seen_[i].occupied)
            continue;
        if (seen_[i].rssi_dbm > best_rssi)
        {
            best_rssi = seen_[i].rssi_dbm;
            best_i = static_cast<int>(i);
        }
    }

    if (best_i < 0)
    {
        hw::log("No free station found, backoff and rescan");
        transition(RState::ErrorBackoff);
        return;
    }

    chosen_.valid = true;
    chosen_.pan_id = seen_[best_i].pan_id;
    chosen_.channel = seen_[best_i].channel;
    chosen_.rssi_dbm = seen_[best_i].rssi_dbm;

    hw::log("Station chosen. Proceed to association.");
    transition(RState::AssocInit);
}

void Robot::handle_error_backoff()
{
    // reset on error
    docking_flag_ = false;
    docked_flag_ = false;
    chosen_ = StationChoice{};

    if (assoc_retry_ < cfg::MAX_ASSOC_RETRIES)
    {
        if (since_enter() == 0)
        {
            hw::log("Backoff before retry");
        }
        if (since_enter() > 600)
        {
            ++assoc_retry_;
            transition(RState::ScanInit);
        }
    }
    else
    {
        hw::log("Max retries reached; wait and return to Idle (will try again later)");
        hw::radio_sleep();
        transition(RState::Idle);
    }
}

void Robot::handle_assoc_init()
{
    if (!chosen_.valid)
    {
        hw::log("No chosen station!");
        transition(RState::ErrorBackoff);
        return;
    }

    if (!hw::radio_join(chosen_.pan_id, chosen_.channel))
    {
        hw::log("Association failed");
        transition(RState::ErrorBackoff);
        return;
    }

    HelloMsg hello{.robot_id = 12}; // 100 bytes payload included
    hw::radio_send(&hello, sizeof(hello));
    hw::log("HELLO sent, waiting for ACCEPT...");
    transition(RState::WaitAccept);
}

void Robot::handle_wait_accept()
{
    AcceptMsg accept{};
    if (hw::radio_recv(&accept, sizeof(accept)))
    {
        hw::log("ACCEPT received, handshake OK");

        session_token_ = accept.session_token;
        period_ms_ = accept.period_ms;

        uint32_t now = hw::now_ms();
        last_tx_ms_ = now;
        last_rx_ms_ = now;

        // Reset docking flags
        docking_flag_ = false;
        docked_flag_ = false;
        dock_phase_ = 0;
        dock_begin_ms_ = 0;
        docking_started_ = false;

        transition(RState::Docking);
        return;
    }

    if (since_enter() > 1000)
    { // 1 second timeout
        hw::log("ACCEPT timeout");
        transition(RState::ErrorBackoff);
    }
}

void Robot::handle_rt_exchange()
{

    rt_heartbeat_tick();

    // check charge completion
    auto batt = hw::read_battery();
    if (batt.percent > 99 && !charging_done_)
    { // charging_done_ will prevent repeated undocking transition when handle_rt_exchange() used in handle_undocking()
        charging_done_ = true;
        hw::log("Charging complete, begin undocking");
        transition(RState::Undocking);
        return;
    }
}

void Robot::handle_docking()
{

    rt_heartbeat_tick();

    // move from pre-charging to in front of charger
    if (dock_phase_ == 0)
    {
        docking_flag_ = true;
        docked_flag_ = false;

        if (hw::navigate_to(cfg::DOCK_APPROACH_WP))
        {
            hw::log("Reached dock approach waypoint");
            dock_phase_ = 1;
            dock_begin_ms_ = hw::now_ms();
            return;
        }
        return;
    }

    // move to charging position
    if (dock_phase_ == 1)
    {
        if (!docking_started_)
        {
            if (hw::dock_start())
            {
                hw::log("Docking started");
                docking_started_ = true;
            }
        }

        // wait until robot in position for charge
        if (hw::dock_is_complete())
        {
            docking_flag_ = false;
            docked_flag_ = true;
            hw::log("Docked and ready to charge ");
            dock_phase_ = 2;
            transition(RState::RTExchange);
            return;
        }

        // for docking fail
        if (hw::now_ms() - dock_begin_ms_ > 20000)
        {
            hw::log("Docking timed out; backoff and rescan");
            docking_flag_ = false;
            docked_flag_ = false;
            transition(RState::ErrorBackoff);
            return;
        }

        return;
    }
}

void Robot::handle_undocking()
{

    rt_heartbeat_tick(); // ensure exchange until undock completes entirely

    static uint32_t undock_start = 0;
    if (since_enter() == 0)
        undock_start = hw::now_ms();
    if (hw::now_ms() - undock_start > 30000)
    { // undock complete after 30s
        hw::log("Undocking complete, send BYE");
        docking_flag_ = false;
        docked_flag_ = false;
        transition(RState::Bye);
    }
}

void Robot::handle_bye()
{
    ByeMsg bye{session_token_};
    hw::radio_send(&bye, sizeof(bye));
    hw::log("BYE sent, session ending");

    hw::radio_sleep();
    transition(RState::Idle);
}
