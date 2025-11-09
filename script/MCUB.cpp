#include <cstdint>
#include <cstring>
#include <optional>

namespace hw
{

    uint32_t now_ms();
    void log(const char *msg);

    void set_supply_charge(bool on); // turn on and off charging

    struct BeaconInfo
    {
        uint16_t pan_id;
        uint8_t channel;
        int8_t rssi_dbm;
        bool occupied; // 0=free, 1=busy
    };

    // start/stop RF
    bool radio_start();
    void radio_sleep();

    bool radio_set_channel(uint8_t ch);

    bool beacon_send(const BeaconInfo &b); // broadcast once

    bool radio_send(const void *data, size_t len);
    bool radio_recv(void *data, size_t len);

}

struct HelloMsg
{
    uint8_t robot_id;
    uint8_t hs_payload[100]; // 100 bytes handshake payload
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
    uint8_t rt_payload[50]; // 50 bytes RT payload
    bool docking;
    bool docked;
};

struct ByeMsg
{
    uint16_t session_token;
};

namespace cfg
{
    constexpr uint16_t PAN_ID = 0x1234;
    constexpr uint8_t CHANNEL = 11;
    constexpr uint32_t BEACON_INTERVAL = 100;  
    constexpr uint32_t ACCEPT_OFFSET_MS = 500; // start TDMA 0.5 after
    constexpr uint16_t PERIOD_MS = 80;         // < 100ms as per requirement
    constexpr uint8_t MISS_LIMIT = 3;          // >3 consecutive misses
    constexpr uint32_t MISS_WINDOW_MS = 100;   // within 100ms window
    constexpr uint32_t CLEAR_DELAY_MS = 4000;  // random delay for robot to clear station
}


enum class SState : uint8_t
{
    Boot,
    Broadcast,    // broadcast itself occupied=0 and listen for robots
    SessionStart, // send session token and start time
    RTExchange,   // receive RT data 
    ClearWait,    // stop charging, wait for robot to clear then reset
    Error
};

class Station
{
public:
    void tick();

private:
    SState st_ = SState::Boot;
    uint32_t st_enter_ms_ = 0;

    bool occupied_ = false;
    bool supply_charge_ = false;

    uint16_t session_token_ = 0;
    uint8_t robot_id_ = 0;

    uint32_t last_rx_ms_ = 0;
    uint8_t miss_count_ = 0;

    uint32_t last_beacon_ms_ = 0;
    uint32_t clear_start_ms_ = 0;

    void transition(SState s)
    {
        st_ = s;
        st_enter_ms_ = hw::now_ms();
    }
    uint32_t since_enter() const { return hw::now_ms() - st_enter_ms_; }

    // handlers
    void handle_boot();
    void handle_broadcast();
    void handle_session_start();
    void handle_rt_exchange();
    void handle_clear_wait();
    void handle_error();
};

void Station::tick()
{
    switch (st_)
    {
    case SState::Boot:
        handle_boot();
        break;
    case SState::Broadcast:
        handle_broadcast();
        break;
    case SState::SessionStart:
        handle_session_start();
        break;
    case SState::RTExchange:
        handle_rt_exchange();
        break;
    case SState::ClearWait:
        handle_clear_wait();
        break;
    case SState::Error:
        handle_error();
        break;
    }
}

void Station::handle_boot()
{
    occupied_ = false;
    supply_charge_ = false;
    hw::set_supply_charge(false);

    if (!hw::radio_start())
    {
        hw::log("RF init fail");
        transition(SState::Error);
        return;
    }
    hw::radio_set_channel(cfg::CHANNEL);

    hw::log("Station booted");
    transition(SState::Broadcast);
}

void Station::handle_broadcast()
{

    uint32_t now = hw::now_ms();
    if (now - last_beacon_ms_ >= cfg::BEACON_INTERVAL)
    {
                        //pan_id, channel, rssi, occupied
        hw::BeaconInfo b{cfg::PAN_ID, cfg::CHANNEL, 0, false};
        hw::beacon_send(b);
        last_beacon_ms_ = now;

    }

    // Listen for robots
    HelloMsg h{};
    if (hw::radio_recv(&h, sizeof(h)))
    {
        occupied_ = true;
        robot_id_ = h.robot_id;
        // Create session token using time
        session_token_ = static_cast<uint16_t>(hw::now_ms());

        hw::log("HELLO from robot received creating session");
        transition(SState::SessionStart);
    }
}

void Station::handle_session_start()
{
    AcceptMsg acc{};
    acc.session_token = session_token_;
    acc.start_time_ms = hw::now_ms() + cfg::ACCEPT_OFFSET_MS;
    acc.period_ms = cfg::PERIOD_MS;

    hw::radio_send(&acc, sizeof(acc));
    hw::log("ACCEPT sent with parameters, starting real time exchange");

    last_rx_ms_ = hw::now_ms();
    miss_count_ = 0;

    transition(SState::RTExchange);
}

void Station::handle_rt_exchange()
{
    // Receive RT data from robot
    RTDataMsg rt{};
    uint32_t now = hw::now_ms();

    if (hw::radio_recv(&rt, sizeof(rt)))
    {
        // Check session token
        if (rt.session_token == session_token_)
        {
            last_rx_ms_ = now;
            miss_count_ = 0;

            // Turn on charger only when robot says 'docked==true'
            if (rt.docked && !supply_charge_)
            {
                hw::set_supply_charge(true);
                supply_charge_ = true;
                hw::log("Charging on");
            }

            // Turn off if undocking
            if (!rt.docked && supply_charge_)
            {
                hw::set_supply_charge(false);
                supply_charge_ = false;
                hw::log("Charging off");
            }
        }
    }

    // missed > MISS_LIMIT and >= 100ms since last RX
    if ((now - last_rx_ms_) >= cfg::MISS_WINDOW_MS)
    {
        if (++miss_count_ > cfg::MISS_LIMIT)
        {
            hw::log("Missed too many times, revoke session");
            // Stop charging, free station after delay
            if (supply_charge_)
            {
                hw::set_supply_charge(false);
                supply_charge_ = false;
            }
            clear_start_ms_ = now;
            occupied_ = false;
            session_token_ = 0;
            transition(SState::ClearWait);
            return;
        }
    }

    // BYE handling
    ByeMsg bye{};
    if (hw::radio_recv(&bye, sizeof(bye)))
    {
        if (bye.session_token == session_token_)
        {
            hw::log("BYE received, finishing session");
            if (supply_charge_)
            {
                hw::set_supply_charge(false);
                supply_charge_ = false;
            }
            occupied_ = false;
            session_token_ = 0;
            clear_start_ms_ = now;
            transition(SState::ClearWait);
            return;
        }
    }
}

void Station::handle_clear_wait()
{
    // Simple delay to let the robot fully clear the bay
    if (hw::now_ms() - clear_start_ms_ >= cfg::CLEAR_DELAY_MS)
    {
        hw::log("Station clear; resume beacons");
        transition(SState::Broadcast);
    }
}

void Station::handle_error()
{
    // Try to re-init after a small pause
    if (since_enter() > 1000)
    {
        transition(SState::Boot);
    }
}
