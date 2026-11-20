import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:mqtt_client/mqtt_client.dart';
import 'package:provider/provider.dart';
import 'package:robotmapapp/models/robot_status.dart';
import 'package:robotmapapp/providers/connection_provider.dart';
import 'package:robotmapapp/providers/robot_state_provider.dart';
import 'package:robotmapapp/providers/speed_limit_provider.dart';
import 'package:robotmapapp/screens/control_screen.dart';
import 'package:robotmapapp/theme/robo_theme.dart';

class _Robot extends RobotStateProvider {
  RobotStatus st;
  _Robot(this.st);
  @override
  RobotStatus get currentStatus => st;
  void report(RobotStatus s) {
    st = s;
    notifyListeners();
  }
}

class _Conn extends ConnectionProvider {
  final bool up;
  int releases = 0;
  _Conn({this.up = true});
  @override
  bool get isConnected => up;
  @override
  void releaseEstop() => releases++;
}

RobotStatus _status({String mode = 'manual', String alert = 'none', bool estop = false}) =>
    RobotStatus(mode: mode, battery: 80, alert: alert, estop: estop);

Future<void> _pump(WidgetTester tester, _Conn conn, _Robot robot) async {
  tester.view.physicalSize = const Size(1080, 2400);
  tester.view.devicePixelRatio = 2.75;
  addTearDown(tester.view.reset);
  await tester.pumpWidget(MultiProvider(
    providers: [
      ChangeNotifierProvider<RobotStateProvider>.value(value: robot),
      ChangeNotifierProvider<ConnectionProvider>.value(value: conn),
      ChangeNotifierProvider(create: (_) => SpeedLimitProvider()),
    ],
    child: MaterialApp(
      theme: roboTheme(),
      home: Scaffold(body: ControlScreen(onSettingsTap: () {}, onNavigate: (_) {})),
    ),
  ));
  await tester.pump(const Duration(seconds: 10)); // joystick intro animation
}

Future<void> _unmount(WidgetTester tester) async {
  await tester.pumpWidget(const SizedBox());
  await tester.pump(const Duration(seconds: 10));
}

void main() {
  group('RobotStatus.estop', () {
    test('robot/status "estop" is read; missing means false', () {
      expect(RobotStatus.fromJson({'mode': 'stop', 'estop': true}).estop, isTrue);
      expect(RobotStatus.fromJson({'mode': 'stop', 'estop': false}).estop, isFalse);
      expect(RobotStatus.fromJson({'mode': 'stop'}).estop, isFalse);
    });

    test('toJson round-trips estop', () {
      final s = RobotStatus.fromJson({'mode': 'stop', 'estop': true});
      expect(RobotStatus.fromJson(s.toJson()).estop, isTrue);
    });

    test('e-stop shows for estop true, alert estop or alert lifted only', () {
      expect(_status(estop: true).isEmergencyStopped, isTrue);
      expect(_status(alert: 'estop').isEmergencyStopped, isTrue);
      expect(_status(alert: 'lifted').isEmergencyStopped, isTrue);
      expect(_status().isEmergencyStopped, isFalse);
      expect(_status(alert: 'low_battery').isEmergencyStopped, isFalse);
      expect(_status(alert: 'geofence_breach').isEmergencyStopped, isFalse);
    });
  });

  group('robot/cmd/estop', () {
    test('payload is {"active": false} (release only), as the bridge reads it', () {
      expect(estopReleasePayload(), '{"active":false}');
    });

    test('releaseEstop publishes on robot/cmd/estop with QoS 1', () {
      final sent = <(String, String, MqttQos)>[];
      final conn = ConnectionProvider(
        connectionStream: const Stream<bool>.empty(),
        publisher: (topic, payload, {MqttQos qos = MqttQos.atMostOnce}) =>
            sent.add((topic, payload, qos)),
      );
      conn.releaseEstop();
      expect(sent, [('robot/cmd/estop', '{"active":false}', MqttQos.atLeastOnce)]);
      conn.dispose();
    });
  });

  group('Liberar parada de emergencia', () {
    testWidgets('hidden while the robot reports no e-stop', (tester) async {
      await _pump(tester, _Conn(), _Robot(_status()));
      expect(find.byKey(const Key('estop_release_button')), findsNothing);
      await _unmount(tester);
    });

    testWidgets('hidden for other alerts', (tester) async {
      await _pump(tester, _Conn(), _Robot(_status(alert: 'low_battery')));
      expect(find.byKey(const Key('estop_release_button')), findsNothing);
      await _unmount(tester);
    });

    for (final (name, st) in [
      ('estop true', _status(mode: 'stop', estop: true)),
      ('alert lifted', _status(mode: 'stop', alert: 'lifted')),
      ('alert estop', _status(mode: 'stop', alert: 'estop')),
    ]) {
      testWidgets('visible with $name', (tester) async {
        await _pump(tester, _Conn(), _Robot(st));
        expect(find.byKey(const Key('estop_release_button')), findsOneWidget);
        expect(find.text('Liberar parada de emergencia'), findsOneWidget);
        await _unmount(tester);
      });
    }

    testWidgets('appears and disappears with robot/status', (tester) async {
      final robot = _Robot(_status());
      await _pump(tester, _Conn(), robot);
      expect(find.byKey(const Key('estop_release_button')), findsNothing);
      robot.report(_status(mode: 'stop', estop: true));
      await tester.pump();
      expect(find.byKey(const Key('estop_release_button')), findsOneWidget);
      robot.report(_status(mode: 'stop'));
      await tester.pump();
      expect(find.byKey(const Key('estop_release_button')), findsNothing);
      await _unmount(tester);
    });

    testWidgets('asks for confirmation first; Cancelar sends nothing', (tester) async {
      final conn = _Conn();
      await _pump(tester, conn, _Robot(_status(mode: 'stop', estop: true)));
      await tester.tap(find.byKey(const Key('estop_release_button')));
      await tester.pump(const Duration(milliseconds: 400)); // dialog animation
      expect(find.text('¿El robot está apoyado y sin nadie cerca?'), findsOneWidget);
      expect(conn.releases, 0, reason: 'nothing is sent before confirming');

      await tester.tap(find.byKey(const Key('estop_cancel')));
      await tester.pump(const Duration(milliseconds: 400));
      expect(find.byKey(const Key('estop_dialog')), findsNothing);
      expect(conn.releases, 0);
      await _unmount(tester);
    });

    testWidgets('dismissing the dialog (tap outside) sends nothing', (tester) async {
      final conn = _Conn();
      await _pump(tester, conn, _Robot(_status(mode: 'stop', estop: true)));
      await tester.tap(find.byKey(const Key('estop_release_button')));
      await tester.pump(const Duration(milliseconds: 400)); // dialog animation
      await tester.tapAt(const Offset(5, 5)); // the barrier
      await tester.pump(const Duration(milliseconds: 400));
      expect(find.byKey(const Key('estop_dialog')), findsNothing);
      expect(conn.releases, 0);
      await _unmount(tester);
    });

    testWidgets('confirming releases the e-stop exactly once', (tester) async {
      final conn = _Conn();
      await _pump(tester, conn, _Robot(_status(mode: 'stop', estop: true)));
      await tester.tap(find.byKey(const Key('estop_release_button')));
      await tester.pump(const Duration(milliseconds: 400)); // dialog animation
      await tester.tap(find.byKey(const Key('estop_confirm')));
      await tester.pump(const Duration(milliseconds: 400));
      expect(conn.releases, 1);
      expect(find.byKey(const Key('estop_dialog')), findsNothing);
      await _unmount(tester);
    });

    testWidgets('releasing does not change the mode by itself', (tester) async {
      // The app never sends manual: the bridge leaves "stop" and the user taps Manual.
      final modes = <String>[];
      final conn = _ModeSpy(modes);
      await _pump(tester, conn, _Robot(_status(mode: 'manual', estop: true)));
      await tester.tap(find.byKey(const Key('estop_release_button')));
      await tester.pump(const Duration(milliseconds: 400)); // dialog animation
      await tester.tap(find.byKey(const Key('estop_confirm')));
      await tester.pump(const Duration(milliseconds: 400));
      expect(modes, isEmpty);
      await _unmount(tester);
    });

    testWidgets('disabled without connection', (tester) async {
      final conn = _Conn(up: false);
      await _pump(tester, conn, _Robot(_status(mode: 'stop', estop: true)));
      final button = tester.widget<ElevatedButton>(
          find.byKey(const Key('estop_release_button')));
      expect(button.onPressed, isNull);
      await _unmount(tester);
    });
  });
}

class _ModeSpy extends _Conn {
  final List<String> modes;
  _ModeSpy(this.modes);
  @override
  void setMode(String mode) => modes.add(mode);
}
