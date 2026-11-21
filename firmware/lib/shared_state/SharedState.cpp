#include "SharedState.h"

namespace shared {

std::atomic<uint8_t>  agent_status{(uint8_t)AgentStatus::WAITING};
std::atomic<bool>     estop_external{false};
std::atomic<bool>     estop_lifted{false};
std::atomic<uint32_t> estop_clear_seq{0};
std::atomic<bool>     battery_low{false};

// Un spinlock por struct: las copias son de unas decenas de bytes
static portMUX_TYPE mux_cmd     = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE mux_ctrl    = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE mux_period  = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE mux_sensors = portMUX_INITIALIZER_UNLOCKED;

static CmdVelShared     g_cmd     = {0.0f, 0.0f, 0};
static ControlSnap      g_ctrl    = {{0, 0, 0, 0, 0}, 0};
static tl::PeriodReport g_period  = {0.0f, 0.0f, 0};
static SensorSnap       g_sensors = {
    {1.5f, 1.5f, 1.5f}, {false, false, false},
    {false, false, false, false, false},
    false, NAN, NAN, NAN
};

void setCmdVel(float v, float w, uint32_t stamp_ms)
{
    if (stamp_ms == 0) stamp_ms = 1;   // 0 esta reservado para "sin orden"
    portENTER_CRITICAL(&mux_cmd);
    g_cmd = {v, w, stamp_ms};
    portEXIT_CRITICAL(&mux_cmd);
}

void clearCmdVel()
{
    portENTER_CRITICAL(&mux_cmd);
    g_cmd = {0.0f, 0.0f, 0};
    portEXIT_CRITICAL(&mux_cmd);
}

CmdVelShared getCmdVel()
{
    portENTER_CRITICAL(&mux_cmd);
    CmdVelShared c = g_cmd;
    portEXIT_CRITICAL(&mux_cmd);
    return c;
}

void setControl(const ControlSnap& s)
{
    portENTER_CRITICAL(&mux_ctrl);
    g_ctrl = s;
    portEXIT_CRITICAL(&mux_ctrl);
}

ControlSnap getControl()
{
    portENTER_CRITICAL(&mux_ctrl);
    ControlSnap s = g_ctrl;
    portEXIT_CRITICAL(&mux_ctrl);
    return s;
}

void setPeriod(const tl::PeriodReport& r)
{
    portENTER_CRITICAL(&mux_period);
    g_period = r;
    portEXIT_CRITICAL(&mux_period);
}

tl::PeriodReport getPeriod()
{
    portENTER_CRITICAL(&mux_period);
    tl::PeriodReport r = g_period;
    portEXIT_CRITICAL(&mux_period);
    return r;
}

void setSensors(const SensorSnap& s)
{
    portENTER_CRITICAL(&mux_sensors);
    g_sensors = s;
    portEXIT_CRITICAL(&mux_sensors);
}

SensorSnap getSensors()
{
    portENTER_CRITICAL(&mux_sensors);
    SensorSnap s = g_sensors;
    portEXIT_CRITICAL(&mux_sensors);
    return s;
}

}  // namespace shared
