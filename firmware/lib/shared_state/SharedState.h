// Datos compartidos entre las tareas de mow_tasks.
//
// Reglas:
//  - Cada dato tiene UN solo escritor (anotado abajo).
//  - Los structs se copian en bloque dentro de una seccion critica corta
//    (portMUX, microsegundos): nadie espera a otro y la tarea de control
//    nunca se bloquea.
//  - Las banderas son std::atomic (lectura/escritura de una palabra).
#ifndef SHARED_STATE_H
#define SHARED_STATE_H

#include <Arduino.h>
#include <atomic>
#include "DiffDrive.h"   // Odometry
#include "TaskLogic.h"

// ── Estado del agente micro-ROS (escribe: microros) ─────────────────────────
enum class AgentStatus : uint8_t { WAITING = 0, CONNECTED, DISCONNECTED };

// ── Orden de velocidad (escribe: microros, lee: control) ────────────────────
struct CmdVelShared {
    float    v;
    float    w;
    uint32_t stamp_ms;   // millis() de recepcion; 0 = sin orden
};

// ── Salida de la tarea de control (escribe: control, lee: microros) ─────────
struct ControlSnap {
    Odometry odom;
    uint32_t seq;        // +1 por ciclo de control
};

// ── Periodo real del control (escribe: control, lee: microros) ──────────────
// Se actualiza una vez por segundo con la ventana anterior.

// ── Sensores (escribe: sensors, lee: microros) ──────────────────────────────
struct SensorSnap {
    float range[3];      // m (left, center, right)
    bool  range_valid[3];
    bool  bumper[5];
    bool  bat_valid;     // false = INA228 sin dato
    float bat_v;
    float bat_i;
    float bat_pct;       // 0-100
};

namespace shared {

// Banderas
extern std::atomic<uint8_t>  agent_status;     // microros
extern std::atomic<bool>     estop_external;   // microros (/estop)
extern std::atomic<bool>     estop_lifted;     // sensors (levantado, enganchado)
extern std::atomic<uint32_t> estop_clear_seq;  // microros: +1 por cada /estop false
extern std::atomic<bool>     battery_low;      // sensors (solo si se MIDE baja)

// Structs (copia en bloque)
void         setCmdVel(float v, float w, uint32_t stamp_ms);
void         clearCmdVel();
CmdVelShared getCmdVel();

void         setControl(const ControlSnap& s);
ControlSnap  getControl();

void             setPeriod(const tl::PeriodReport& r);
tl::PeriodReport getPeriod();

void         setSensors(const SensorSnap& s);
SensorSnap   getSensors();

inline bool agentConnected()
{
    return agent_status.load() == (uint8_t)AgentStatus::CONNECTED;
}
inline bool anyEstop()
{
    return estop_external.load() || estop_lifted.load();
}

}  // namespace shared

#endif
