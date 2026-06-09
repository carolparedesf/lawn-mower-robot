#include "DisplayManager.h"

DisplayManager::DisplayManager()
    : display_(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET),
      ready_(false), splash_done_(false),
      splash_start_ms_(0), last_refresh_ms_(0), boot_ms_(0)
{}

bool DisplayManager::begin()
{
    if (!display_.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        return false;
    }
    boot_ms_        = millis();
    splash_start_ms_ = boot_ms_;
    ready_          = true;
    showSplash();
    return true;
}

void DisplayManager::update(const BatteryManager& battery, BuzzerPattern state)
{
    if (!ready_) return;

    // ── Splash no bloqueante ────────────────────────────────────────────────
    if (!splash_done_) {
        if (millis() - splash_start_ms_ >= SPLASH_DURATION_MS) {
            splash_done_ = true;
            display_.clearDisplay();
            display_.display();
        }
        return;  // durante splash no actualizar pantalla principal
    }

    // ── Refresco principal ──────────────────────────────────────────────────
    unsigned long now = millis();
    if (now - last_refresh_ms_ < DISPLAY_INTERVAL_MS) return;
    last_refresh_ms_ = now;

    showMain(battery, state);
}

// ── Privados ─────────────────────────────────────────────────────────────────

void DisplayManager::showSplash()
{
    display_.clearDisplay();
    display_.drawBitmap(0, 0, mow_bitmap, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
    display_.display();
}

void DisplayManager::showMain(const BatteryManager& battery, BuzzerPattern state)
{
    display_.clearDisplay();

    // ── Fila 0: MOW - Estado ─────────────────────────────────────────────
    display_.setTextSize(1);
    display_.setTextColor(WHITE);
    display_.setCursor(0, 0);
    display_.print(F("MOW"));
    display_.print(F("  -  "));
    display_.print(stateToString(state));

    // ── Separador ────────────────────────────────────────────────────────
    display_.drawFastHLine(0, 10, SCREEN_WIDTH, WHITE);

    // ── Fila 1: Voltaje y Corriente ──────────────────────────────────────
    display_.setCursor(0, 14);
    display_.print(F("V:"));
    display_.print(battery.getVoltage(), 1);
    display_.print(F("V  I:"));
    display_.print(battery.getCurrent(), 2);
    display_.print(F("A"));

    // ── Fila 2: Potencia y Carga ─────────────────────────────────────────
    display_.setCursor(0, 26);
    display_.print(F("P:"));
    display_.print(battery.getPower(), 1);
    display_.print(F("W"));

    // Barra de carga pequeña inline
    float pct = battery.getPercent();
    char pct_buf[8];
    snprintf(pct_buf, sizeof(pct_buf), "%3.0f%%", pct);
    display_.print(F("  Car:"));
    display_.print(pct_buf);

    // ── Barra gráfica de carga ────────────────────────────────────────────
    drawBatteryBar(0, 38, 128, 10, pct);

    // ── Separador ────────────────────────────────────────────────────────
    display_.drawFastHLine(0, 51, SCREEN_WIDTH, WHITE);

    // ── Fila 3: Tiempo transcurrido ───────────────────────────────────────
    char uptime_buf[16];
    formatUptime(millis() - boot_ms_, uptime_buf, sizeof(uptime_buf));
    display_.setCursor(0, 54);
    display_.print(F("T.transc: "));
    display_.print(uptime_buf);

    display_.display();
}

void DisplayManager::drawBatteryBar(int x, int y, int w, int h, float percent)
{
    // Marco exterior con símbolo de batería
    display_.drawRect(x, y, w - 4, h, WHITE);
    display_.fillRect(w - 4, y + 2, 4, h - 4, WHITE);  // terminal +

    // Relleno proporcional al porcentaje
    int fill = (int)((w - 6) * percent / 100.0f);
    if (fill > 0) {
        display_.fillRect(x + 1, y + 1, fill, h - 2, WHITE);
    }

    // Texto % centrado en la barra
    display_.setTextColor(INVERSE);
    char buf[5];
    snprintf(buf, sizeof(buf), "%2.0f%%", percent);
    int tx = x + (w - 6) / 2 - 10;
    display_.setCursor(tx, y + 1);
    display_.print(buf);
    display_.setTextColor(WHITE);
}

const char* DisplayManager::stateToString(BuzzerPattern state)
{
    switch (state) {
        case BuzzerPattern::CONNECTED:    return "Conectado";
        case BuzzerPattern::DISCONNECTED: return "Desconect.";
        case BuzzerPattern::BATTERY_LOW:  return "Bat. baja";
        case BuzzerPattern::START:        return "Navegando";
        case BuzzerPattern::LIFTED:       return "Levantado";
        case BuzzerPattern::CUTTING:      return "Cortando";
        case BuzzerPattern::NONE:
        default:                          return "Iniciando...";
    }
}

void DisplayManager::formatUptime(unsigned long ms, char* buf, size_t len)
{
    unsigned long total_s = ms / 1000;
    unsigned long h       = total_s / 3600;
    unsigned long m       = (total_s % 3600) / 60;

    if (h > 0) {
        snprintf(buf, len, "%luh %02lum", h, m);
    } else {
        unsigned long s = total_s % 60;
        snprintf(buf, len, "%02lum %02lus", m, s);
    }
}
