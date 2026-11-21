#include "BatteryManager.h"
#include "BuzzerManager.h"
#include "DisplayManager.h"

// ── Instancias ───────────────────────────────────────────────────────────────
BatteryManager battery(0.002f, 20.0f);   // shunt 2mΩ, 20A máx
BuzzerManager  buzzer(2);                // buzzer GPIO 2
DisplayManager display_mgr;

// ── Estado actual del sistema ─────────────────────────────────────────────────
BuzzerPattern current_state = BuzzerPattern::NONE;

unsigned long last_print = 0;

// ════════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(100000);  // 100kHz — estable con cables Dupont

    buzzer.begin();

    // ── OLED ─────────────────────────────────────────────────────────────
    if (!display_mgr.begin()) {
        Serial.println(F("OLED no encontrada"));
    }

    // ── INA228 ───────────────────────────────────────────────────────────
    if (!battery.begin()) {
        Serial.println(F("INA228 no encontrado"));
        current_state = BuzzerPattern::DISCONNECTED;
        while (1) {
            buzzer.play(BuzzerPattern::DISCONNECTED);
            delay(2000);
        }
    }

    Serial.println(F("Sistema OK"));
}

// ════════════════════════════════════════════════════════════════════════════
void loop() {
    buzzer.update();
    battery.update();
    display_mgr.update(battery, current_state);

    // ── Log serial cada 2s ────────────────────────────────────────────────
    if (millis() - last_print >= 2000) {
        last_print = millis();
        Serial.printf("V:%.2fV  I:%.3fA  P:%.1fW  Bat:%.1f%%  Estado:%s\n",
            battery.getVoltage(),
            battery.getCurrent(),
            battery.getPower(),
            battery.getPercent(),
            // reutiliza el mismo texto que la pantalla
            (current_state == BuzzerPattern::CONNECTED)    ? "Conectado"   :
            (current_state == BuzzerPattern::DISCONNECTED) ? "Desconect."  :
            (current_state == BuzzerPattern::BATTERY_LOW)  ? "Bat.baja"    :
            (current_state == BuzzerPattern::START)        ? "Navegando"   :
            (current_state == BuzzerPattern::LIFTED)       ? "Levantado"   :
            (current_state == BuzzerPattern::CUTTING)      ? "Cortando"    :
                                                             "Iniciando"
        );

        // ── Alertas automáticas ───────────────────────────────────────────
        if (battery.isLow() && current_state != BuzzerPattern::BATTERY_LOW) {
            current_state = BuzzerPattern::BATTERY_LOW;
            buzzer.play(BuzzerPattern::BATTERY_LOW);
        }
    }
}

// ── API pública para cambiar estado desde otras partes del firmware ──────────
// Llama a esta función desde tu lógica de navegación/corte:
//   setState(BuzzerPattern::CONNECTED);
//   setState(BuzzerPattern::CUTTING);
void setState(BuzzerPattern new_state) {
    current_state = new_state;
    buzzer.play(new_state);
}
