// Logica pura del firmware en tareas (mow_tasks): decisiones de seguridad,
// enganche de levantado, compuerta de bateria, eventos y diagnostico.
// Sin Arduino ni FreeRTOS: se testea en la PC con
//   g++ -std=c++17 -I lib/task_logic test/host/test_task_logic.cpp -o /tmp/t && /tmp/t
#ifndef TASK_LOGIC_H
#define TASK_LOGIC_H

#include <stdint.h>
#include <string.h>
#include <math.h>

namespace tl {

// ── Orden de velocidad ──────────────────────────────────────────────────────
// stamp_ms == 0 significa "no hay orden" (se usa al desconectar o en e-stop).
inline bool cmdFresh(uint32_t now_ms, uint32_t stamp_ms, uint32_t timeout_ms)
{
    if (stamp_ms == 0) return false;
    return (uint32_t)(now_ms - stamp_ms) <= timeout_ms;
}

// Igual que main.cpp: |v| y |w| < 0.01 es "orden 0" (si no, la zona muerta
// convertiria una orden de 0.009 m/s en PWM 70).
inline bool isZeroCmd(float v, float w)
{
    return fabsf(v) < 0.01f && fabsf(w) < 0.01f;
}

inline bool driveEnabled(bool agent_connected, bool cmd_fresh,
                         bool estop_external, bool estop_lifted,
                         bool battery_low)
{
    return agent_connected && cmd_fresh && !estop_external && !estop_lifted
           && !battery_low;
}

// ── Levantado (pares de bumpers opuestos, como main.cpp) ────────────────────
// b[0]=front_left b[1]=front_right b[2]=left b[3]=right b[4]=back
inline bool liftedFromBumpers(const bool b[5])
{
    return (b[2] && b[3])      // agarrado lateralmente
        || (b[0] && b[4])      // diagonal izquierda
        || (b[1] && b[4]);     // diagonal derecha
}

// Enganche de estop_lifted: solo lo escribe la tarea de sensores. Un pedido de
// liberacion (/estop false) lo limpia, pero si los bumpers siguen apretados en
// el mismo paso se vuelve a enganchar.
class LiftLatch {
public:
    // Devuelve true en el flanco de activacion (para el sonido).
    bool step(bool clear_requested, bool lifted_now)
    {
        if (clear_requested) active_ = false;
        if (lifted_now && !active_) { active_ = true; return true; }
        return false;
    }
    bool active() const { return active_; }
private:
    bool active_ = false;
};

// ── Compuerta de bateria ────────────────────────────────────────────────────
// Solo deshabilita si el INA228 MIDE tension baja. "Sin dato" no frena.
class BatteryGate {
public:
    BatteryGate(float min_v, float hyst_v) : min_v_(min_v), hyst_v_(hyst_v) {}
    void update(bool valid, float voltage)
    {
        if (!valid || isnan(voltage)) { low_ = false; return; }
        if (voltage < min_v_)                 low_ = true;
        else if (voltage >= min_v_ + hyst_v_) low_ = false;
    }
    bool low() const { return low_; }
private:
    float min_v_, hyst_v_;
    bool  low_ = false;
};

// ── Eventos de /robot/events ────────────────────────────────────────────────
enum class Event : uint8_t {
    NONE = 0,
    LIFTED,
    CUTTING_STARTED,
    NAVIGATION_STARTED,
    PERIMETER_STARTED,
    LOW_BATTERY,
    COUNT
};

// Coincidencia exacta; len es el largo sin terminador.
inline Event parseEvent(const char* s, size_t len)
{
    struct { const char* name; Event ev; } const table[] = {
        {"lifted",             Event::LIFTED},
        {"cutting_started",    Event::CUTTING_STARTED},
        {"navigation_started", Event::NAVIGATION_STARTED},
        {"perimeter_started",  Event::PERIMETER_STARTED},
        {"low_battery",        Event::LOW_BATTERY},
    };
    if (s == nullptr) return Event::NONE;
    for (const auto& e : table) {
        if (strlen(e.name) == len && memcmp(e.name, s, len) == 0) return e.ev;
    }
    return Event::NONE;
}

// El mismo evento no suena dos veces en menos de window_ms (lo detecto el
// ESP32 y ademas llego por /robot/events, o llego repetido).
class EventDedup {
public:
    explicit EventDedup(uint32_t window_ms) : window_ms_(window_ms) {}
    bool shouldPlay(Event ev, uint32_t now_ms)
    {
        uint8_t i = (uint8_t)ev;
        if (ev == Event::NONE || i >= (uint8_t)Event::COUNT) return false;
        if (played_[i] && (uint32_t)(now_ms - last_ms_[i]) < window_ms_) return false;
        played_[i]  = true;
        last_ms_[i] = now_ms;
        return true;
    }
private:
    uint32_t window_ms_;
    bool     played_[(uint8_t)Event::COUNT] = {};
    uint32_t last_ms_[(uint8_t)Event::COUNT] = {};
};

// Flanco de subida (para eventos detectados localmente)
class RisingEdge {
public:
    bool step(bool v) { bool r = v && !prev_; prev_ = v; return r; }
private:
    bool prev_ = false;
};

// ── Diezmado de la odometria ────────────────────────────────────────────────
// Publica la ultima muestra cada N ciclos de control (N = control_hz / pub_hz).
inline uint32_t odomDecimation(uint32_t control_hz, uint32_t pub_hz)
{
    if (pub_hz == 0 || pub_hz >= control_hz) return 1;
    return (control_hz + pub_hz / 2) / pub_hz;   // redondeo
}

class OdomDecimator {
public:
    explicit OdomDecimator(uint32_t n) : n_(n ? n : 1) {}
    // seq: contador de muestras de la tarea de control (crece de a 1)
    bool shouldPublish(uint32_t seq)
    {
        if (!has_ || (uint32_t)(seq - last_) >= n_) {
            has_ = true; last_ = seq; return true;
        }
        return false;
    }
    void reset() { has_ = false; }
private:
    uint32_t n_;
    uint32_t last_ = 0;
    bool     has_  = false;
};

// ── Periodo real de la tarea de control ─────────────────────────────────────
struct PeriodReport { float avg_ms; float max_ms; uint32_t samples; };

class PeriodStats {
public:
    void add(uint32_t dt_us)
    {
        sum_us_ += dt_us;
        if (dt_us > max_us_) max_us_ = dt_us;
        n_++;
    }
    uint32_t count() const { return n_; }
    // Devuelve el resumen y arranca una ventana nueva
    PeriodReport take()
    {
        PeriodReport r{0.0f, 0.0f, n_};
        if (n_ > 0) {
            r.avg_ms = (float)((double)sum_us_ / n_ / 1000.0);
            r.max_ms = max_us_ / 1000.0f;
        }
        sum_us_ = 0; max_us_ = 0; n_ = 0;
        return r;
    }
private:
    uint64_t sum_us_ = 0;
    uint32_t max_us_ = 0;
    uint32_t n_      = 0;
};

// ── Reconexion: N pings fallidos seguidos antes de desconectar ──────────────
class PingMonitor {
public:
    explicit PingMonitor(uint8_t max_fails) : max_fails_(max_fails) {}
    // Devuelve true cuando hay que desconectar
    bool onPing(bool ok)
    {
        if (ok) { fails_ = 0; return false; }
        if (fails_ < 255) fails_++;
        return fails_ >= max_fails_;
    }
    void reset() { fails_ = 0; }
    uint8_t fails() const { return fails_; }
private:
    uint8_t max_fails_;
    uint8_t fails_ = 0;
};

}  // namespace tl

#endif
