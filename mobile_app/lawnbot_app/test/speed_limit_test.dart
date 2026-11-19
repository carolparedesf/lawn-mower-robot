import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:provider/provider.dart';
import 'package:robotmapapp/providers/connection_provider.dart';
import 'package:robotmapapp/providers/speed_limit_provider.dart';
import 'package:robotmapapp/widgets/joystick_widget.dart';
import 'package:robotmapapp/widgets/speed_limit_selector.dart';

class _Connected extends ConnectionProvider {
  @override
  bool get isConnected => true;
}

void main() {
  group('joystickCommand (same output as before at full speed)', () {
    test('dominant axis gives the direction, its deflection the speed', () {
      expect(joystickCommand(0.0, -0.8), (dir: 'fwd', speed: 0.8));
      expect(joystickCommand(0.1, 0.5), (dir: 'back', speed: 0.5));
      expect(joystickCommand(0.9, -0.3), (dir: 'right', speed: 0.9));
      expect(joystickCommand(-0.6, 0.2), (dir: 'left', speed: 0.6));
    });

    test('dead zone: both axes under 0.2 is stop with speed 0', () {
      expect(joystickDeadZone, 0.2);
      expect(joystickCommand(0.19, -0.19), (dir: 'stop', speed: 0.0));
      expect(joystickCommand(0.0, 0.0, factor: 1.0), (dir: 'stop', speed: 0.0));
      expect(joystickCommand(0.2, 0.0).dir, 'right'); // the limit itself is not dead
    });

    test('a tie between axes goes to forward/back; speed never above 1', () {
      expect(joystickCommand(0.5, -0.5).dir, 'fwd');
      expect(joystickCommand(0.0, -1.7).speed, 1.0);
    });
  });

  group('speed limit scales what is published', () {
    test('levels: slow 30 %, medium 60 %, fast 100 %; default slow', () {
      expect(SpeedLevel.slow.factor, 0.3);
      expect(SpeedLevel.medium.factor, 0.6);
      expect(SpeedLevel.fast.factor, 1.0);
      expect(SpeedLimitProvider().level, SpeedLevel.slow);
      expect(SpeedLimitProvider().factor, 0.3);
    });

    test('speed is the deflection times the factor; direction does not change', () {
      for (final level in SpeedLevel.values) {
        final c = joystickCommand(0.0, -0.8, factor: level.factor);
        expect(c.dir, 'fwd');
        expect(c.speed, closeTo(0.8 * level.factor, 1e-12));
      }
      expect(joystickCommand(0.0, -1.0, factor: 0.3).speed, closeTo(0.3, 1e-12));
      expect(joystickCommand(0.0, -1.0, factor: 0.6).speed, closeTo(0.6, 1e-12));
      expect(joystickCommand(0.0, -1.0, factor: 1.0).speed, 1.0);
    });

    test('stop stays at 0 whatever the level', () {
      for (final level in SpeedLevel.values) {
        expect(joystickCommand(0.05, 0.05, factor: level.factor).speed, 0.0);
      }
    });
  });

  testWidgets('the selector shows the three levels, slow selected, and changes the level',
      (tester) async {
    final limit = SpeedLimitProvider();
    await tester.pumpWidget(ChangeNotifierProvider.value(
      value: limit,
      child: const MaterialApp(home: Scaffold(body: Center(child: SpeedLimitSelector()))),
    ));
    expect(find.text('Lento 30 %'), findsOneWidget);
    expect(find.text('Medio 60 %'), findsOneWidget);
    expect(find.text('Rápido 100 %'), findsOneWidget);
    expect(tester.widget<SegmentedButton<SpeedLevel>>(find.byKey(const Key('speed_limit'))).selected,
        {SpeedLevel.slow});
    await tester.tap(find.text('Medio 60 %'));
    await tester.pump();
    expect(limit.level, SpeedLevel.medium);
    await tester.tap(find.text('Rápido 100 %'));
    await tester.pump();
    expect(limit.factor, 1.0);
  });

  group('turning is sized in rad/s: the level scales only linear speed', () {
    double omega(({String dir, double speed}) c) => c.speed * turnMaxRadS;

    test('constants: minimum 0.8 rad/s, 1.5 rad/s at speed 1 (as in the bridge)', () {
      expect(turnMinRadS, 0.8);
      expect(turnMaxRadS, 1.5);
    });

    test('Lento and Medio never turn below 0.8 rad/s, even at a small deflection', () {
      for (final level in [SpeedLevel.slow, SpeedLevel.medium, SpeedLevel.fast]) {
        for (final x in [0.2, 0.5, 1.0]) {
          final c = joystickCommand(x, 0.0, factor: level.factor);
          expect(omega(c), greaterThanOrEqualTo(0.8 - 1e-9), reason: '${level.name} x=$x');
        }
      }
      expect(omega(joystickCommand(1.0, 0.0, factor: SpeedLevel.slow.factor)), closeTo(0.8, 1e-9));
      expect(omega(joystickCommand(-1.0, 0.0, factor: SpeedLevel.slow.factor)), closeTo(0.8, 1e-9));
    });

    test('Medio gains with the deflection up to 0.9 rad/s; Rápido reaches 1.5', () {
      expect(omega(joystickCommand(1.0, 0.0, factor: SpeedLevel.medium.factor)), closeTo(0.9, 1e-9));
      expect(omega(joystickCommand(1.0, 0.0, factor: SpeedLevel.fast.factor)), closeTo(1.5, 1e-9));
      expect(omega(joystickCommand(0.7, 0.0, factor: SpeedLevel.fast.factor)), closeTo(1.05, 1e-9));
    });

    test('a turn is never above 1.5 rad/s and speed stays in 0..1', () {
      final c = joystickCommand(3.0, 0.0);
      expect(c.speed, 1.0);
      expect(joystickCommand(-3.0, 0.0, factor: 1.0).dir, 'left');
    });

    test('the minimum is a parameter', () {
      expect(omega(joystickCommand(1.0, 0.0, factor: 0.3, minTurnRadS: 1.0)), closeTo(1.0, 1e-9));
    });

    test('the in-place turn at Lento moves each wheel above 0.14 m/s (was 0.053)', () {
      final c = joystickCommand(1.0, 0.0, factor: SpeedLevel.slow.factor);
      expect(omega(c) * 0.355 / 2, greaterThan(0.14));
    });

    test('forward/back are still scaled by the level only', () {
      expect(joystickCommand(0.0, -1.0, factor: 0.3).speed, closeTo(0.3, 1e-12));
      expect(joystickCommand(0.0, 1.0, factor: 0.6).speed, closeTo(0.6, 1e-12));
    });
  });

  testWidgets('robot/cmd/move carries the scaled speed, and zero on release', (tester) async {
    final limit = SpeedLimitProvider();
    final sent = <(String, Map<String, dynamic>)>[];
    await tester.pumpWidget(MultiProvider(
      providers: [
        ChangeNotifierProvider<ConnectionProvider>.value(value: _Connected()),
        ChangeNotifierProvider.value(value: limit),
      ],
      child: MaterialApp(
        home: Scaffold(
          body: Center(
            child: JoystickWidget(
              publishOverride: (topic, payload) =>
                  sent.add((topic, jsonDecode(payload) as Map<String, dynamic>)),
            ),
          ),
        ),
      ),
    ));
    await tester.pump(const Duration(seconds: 10)); // the package's intro animation
    sent.clear();

    Future<List<double>> dragUp(SpeedLevel level) async {
      limit.setLevel(level);
      sent.clear();
      final gesture = await tester.startGesture(tester.getCenter(find.byType(JoystickStick)));
      await gesture.moveBy(const Offset(0, -30)); // past the touch slop: the pan starts here
      await gesture.moveBy(const Offset(0, -170)); // fully forward
      await tester.pump(const Duration(milliseconds: 100)); // the package's 100 ms callback...
      await tester.pump(const Duration(milliseconds: 60)); // ...then the 50 ms publish throttle
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 120));
      return [for (final (_, m) in sent) if (m['dir'] == 'fwd') (m['speed'] as num).toDouble()];
    }

    for (final level in SpeedLevel.values) {
      final speeds = await dragUp(level);
      expect(speeds, isNotEmpty, reason: level.name);
      expect(speeds.last, closeTo(level.factor, 0.02), reason: level.name);
      expect(speeds.every((s) => s <= level.factor + 1e-9), isTrue, reason: level.name);
      expect(sent.every((e) => e.$1 == 'robot/cmd/move'), isTrue);
      expect(sent.last.$2, {'dir': 'stop', 'speed': 0.0}, reason: 'release sends stop with speed 0');
    }
    await tester.pumpWidget(const SizedBox());
    await tester.pump(const Duration(seconds: 10));
  });
}
