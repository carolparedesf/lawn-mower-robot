#include "BatteryManager.h"
#include "BuzzerManager.h"

// Ajustar shunt_ohms según el valor del módulo (R010=0.01, R020=0.02)
BatteryManager battery(0.01f, 20.0f);  // 10mΩ, 20A máx
BuzzerManager  buzzer(2);

unsigned long last_print = 0;

void setup() {
    Serial.begin(115200);
    buzzer.begin();

    if (!battery.begin()) {
        Serial.println("INA228 no encontrado");
        // Parpadeo de error
        while(1) {
            buzzer.play(BuzzerPattern::DISCONNECTED);
            delay(2000);
        }
    }
    Serial.println("INA228 OK");
}

void loop() {
    battery.update();
    buzzer.update();

    if (millis() - last_print >= 1000) {
        last_print = millis();
        Serial.printf("V:%.2f  I:%.2f  P:%.1f  %%:%.1f  low:%d\n",
            battery.getVoltage(),
            battery.getCurrent(),
            battery.getPower(),
            battery.getPercent(),
            battery.isLow()
        );
    }

    // Alerta batería baja
    if (battery.isLow()) {
        buzzer.play(BuzzerPattern::BATTERY_LOW);
    }
}
