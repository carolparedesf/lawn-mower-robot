// Tests de lib/task_logic en la PC (no en el ESP32). Desde firmware/:
//   g++ -std=c++17 -Wall -Wextra -I lib/task_logic test/host/test_task_logic.cpp -o /tmp/t && /tmp/t
#include "TaskLogic.h"
#include <cstdio>

static int g_fail = 0, g_count = 0;
#define CHECK(c) do { g_count++; if (!(c)) { g_fail++; \
    std::printf("FALLA %s:%d  %s\n", __FILE__, __LINE__, #c); } } while (0)

using namespace tl;

static void test_cmd()
{
    CHECK(!cmdFresh(1000, 0, 500));            // sin orden
    CHECK(cmdFresh(1000, 600, 500));           // 400 ms
    CHECK(cmdFresh(1000, 500, 500));           // justo 500 ms
    CHECK(!cmdFresh(1001, 500, 500));          // > 500 ms
    CHECK(cmdFresh(5, 0xFFFFFFF0u, 500));      // vuelta de millis()
    CHECK(isZeroCmd(0.0f, 0.0f));
    CHECK(isZeroCmd(0.009f, -0.009f));
    CHECK(!isZeroCmd(0.01f, 0.0f));
    CHECK(!isZeroCmd(0.0f, -0.2f));
}

static void test_drive_enabled()
{
    CHECK(driveEnabled(true, true, false, false, false));
    CHECK(!driveEnabled(false, true, false, false, false));  // sin agente
    CHECK(!driveEnabled(true, false, false, false, false));  // orden vieja
    CHECK(!driveEnabled(true, true, true, false, false));    // estop_external
    CHECK(!driveEnabled(true, true, false, true, false));    // estop_lifted
    CHECK(!driveEnabled(true, true, false, false, true));    // bateria baja medida
}

static void test_lifted()
{
    bool none[5] = {false, false, false, false, false};
    bool side[5] = {false, false, true, true, false};
    bool diagL[5] = {true, false, false, false, true};
    bool diagR[5] = {false, true, false, false, true};
    bool front[5] = {true, true, false, false, false};     // choque frontal
    bool one[5] = {false, false, true, false, false};
    CHECK(!liftedFromBumpers(none));
    CHECK(liftedFromBumpers(side));
    CHECK(liftedFromBumpers(diagL));
    CHECK(liftedFromBumpers(diagR));
    CHECK(!liftedFromBumpers(front));
    CHECK(!liftedFromBumpers(one));
}

static void test_lift_latch()
{
    LiftLatch l;
    CHECK(!l.step(false, false) && !l.active());
    CHECK(l.step(false, true) && l.active());       // flanco
    CHECK(!l.step(false, true) && l.active());      // sin flanco repetido
    CHECK(!l.step(false, false) && l.active());     // enganchado al soltar
    CHECK(!l.step(true, false) && !l.active());     // /estop false lo limpia
    // Liberacion con los bumpers todavia apretados: se re-engancha
    CHECK(l.step(false, true) && l.active());
    CHECK(l.step(true, true) && l.active());
}

static void test_battery_gate()
{
    BatteryGate g(15.0f, 1.0f);
    g.update(false, NAN);   CHECK(!g.low());        // sin dato: no frena
    g.update(true, 14.9f);  CHECK(g.low());         // mide baja
    g.update(true, 15.5f);  CHECK(g.low());         // histeresis
    g.update(true, 16.0f);  CHECK(!g.low());
    g.update(true, 14.0f);  CHECK(g.low());
    g.update(false, NAN);   CHECK(!g.low());        // se pierde el dato: no frena
    g.update(true, NAN);    CHECK(!g.low());
}

static void test_events()
{
    CHECK(parseEvent("lifted", 6) == Event::LIFTED);
    CHECK(parseEvent("cutting_started", 15) == Event::CUTTING_STARTED);
    CHECK(parseEvent("navigation_started", 18) == Event::NAVIGATION_STARTED);
    CHECK(parseEvent("perimeter_started", 17) == Event::PERIMETER_STARTED);
    CHECK(parseEvent("low_battery", 11) == Event::LOW_BATTERY);
    CHECK(parseEvent("lifted ", 7) == Event::NONE);
    CHECK(parseEvent("lift", 4) == Event::NONE);
    CHECK(parseEvent("", 0) == Event::NONE);
    CHECK(parseEvent(nullptr, 0) == Event::NONE);
    CHECK(parseEvent("LIFTED", 6) == Event::NONE);

    EventDedup d(2000);
    CHECK(d.shouldPlay(Event::LIFTED, 1000));
    CHECK(!d.shouldPlay(Event::LIFTED, 2999));      // < 2 s: no repite
    CHECK(d.shouldPlay(Event::LOW_BATTERY, 2999));  // otro evento: si
    CHECK(d.shouldPlay(Event::LIFTED, 3000));       // 2 s: suena
    CHECK(!d.shouldPlay(Event::NONE, 9000));
    CHECK(d.shouldPlay(Event::CUTTING_STARTED, 0)); // primer evento en t=0

    RisingEdge e;
    CHECK(!e.step(false));
    CHECK(e.step(true));
    CHECK(!e.step(true));
    CHECK(!e.step(false));
    CHECK(e.step(true));
}

static void test_beep_sequencer()
{
    static const Beep two[] = { {100, 100, 2} };                 // 2 beeps
    static const Beep nav[] = { {100, 100, 2}, {400, 0, 1} };    // corto-corto-largo
    BeepSequencer s;
    CHECK(!s.playing() && !s.step(0));

    s.play(two, 1, 1000);
    CHECK(s.playing());
    CHECK(s.step(1000));        // on
    CHECK(s.step(1099));
    CHECK(!s.step(1100));       // off
    CHECK(!s.step(1199));
    CHECK(s.step(1200));        // 2do beep
    CHECK(!s.step(1300));
    CHECK(!s.step(1400) && !s.playing());   // termino (400 ms en total)

    // Muestreo grueso (20 Hz): no se traba y termina
    s.play(nav, 2, 0);
    int on = 0;
    for (uint32_t t = 0; t <= 1000; t += 50) on += s.step(t) ? 1 : 0;
    CHECK(!s.playing());
    CHECK(on == 2 + 2 + 8);     // 100 ms + 100 ms + 400 ms encendido

    // Salto de tiempo largo: sale apagado y terminado
    s.play(nav, 2, 0);
    CHECK(!s.step(10000) && !s.playing());

    // Uno nuevo corta al anterior
    s.play(nav, 2, 0);
    s.play(two, 1, 50);
    CHECK(s.step(50) && s.playing());
    CHECK(!s.step(450) && !s.playing());

    s.play(nullptr, 1, 0);  CHECK(!s.playing());
    s.play(two, 0, 0);      CHECK(!s.playing());
}

static void test_odom_decimation()
{
    CHECK(odomDecimation(50, 50) == 1);
    CHECK(odomDecimation(50, 100) == 1);
    CHECK(odomDecimation(50, 0) == 1);
    CHECK(odomDecimation(50, 25) == 2);
    CHECK(odomDecimation(50, 10) == 5);
    CHECK(odomDecimation(50, 20) == 3);   // 2.5 redondeado

    OdomDecimator all(1);
    CHECK(all.shouldPublish(7));
    CHECK(all.shouldPublish(8));
    CHECK(!all.shouldPublish(8));         // misma muestra: no

    OdomDecimator five(5);
    int n = 0;
    for (uint32_t s = 100; s < 150; s++) n += five.shouldPublish(s) ? 1 : 0;
    CHECK(n == 10);                       // 50 muestras -> 10 publicaciones
    OdomDecimator skip(5);
    CHECK(skip.shouldPublish(0));
    CHECK(skip.shouldPublish(9));         // saltos de seq: publica la ultima
    CHECK(!skip.shouldPublish(10));
    CHECK(skip.shouldPublish(0xFFFFFFFFu) == true); // vuelta del contador
}

static void test_period_stats()
{
    PeriodStats p;
    PeriodReport r0 = p.take();
    CHECK(r0.samples == 0 && r0.avg_ms == 0.0f && r0.max_ms == 0.0f);
    p.add(20000); p.add(20000); p.add(23000); p.add(17000);
    CHECK(p.count() == 4);
    PeriodReport r = p.take();
    CHECK(r.samples == 4);
    CHECK(fabsf(r.avg_ms - 20.0f) < 1e-4f);
    CHECK(fabsf(r.max_ms - 23.0f) < 1e-4f);
    CHECK(p.count() == 0);
}

static void test_ping_monitor()
{
    PingMonitor m(3);
    CHECK(!m.onPing(false));
    CHECK(!m.onPing(false));
    CHECK(!m.onPing(true));               // uno bueno reinicia la cuenta
    CHECK(!m.onPing(false));
    CHECK(!m.onPing(false));
    CHECK(m.onPing(false));               // 3 seguidos: desconectar
    m.reset();
    CHECK(m.fails() == 0);
}

int main()
{
    test_cmd();
    test_drive_enabled();
    test_lifted();
    test_lift_latch();
    test_battery_gate();
    test_events();
    test_beep_sequencer();
    test_odom_decimation();
    test_period_stats();
    test_ping_monitor();
    std::printf("%d checks, %d fallas\n", g_count, g_fail);
    return g_fail ? 1 : 0;
}
