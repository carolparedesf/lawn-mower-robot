import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:provider/provider.dart';
import 'package:robotmapapp/models/robot_status.dart';
import 'package:robotmapapp/providers/connection_provider.dart';
import 'package:robotmapapp/providers/robot_state_provider.dart';
import 'package:robotmapapp/providers/speed_limit_provider.dart';
import 'package:robotmapapp/screens/control_screen.dart';
import 'package:robotmapapp/screens/mode_screen.dart';
import 'package:robotmapapp/theme/robo_theme.dart';
import 'package:robotmapapp/widgets/joystick_widget.dart';

class _Robot extends RobotStateProvider {
  RobotStatus st;
  _Robot(String mode) : st = RobotStatus(mode: mode, battery: null, alert: 'none');
  @override
  RobotStatus get currentStatus => st;
  void report(String mode) {
    st = RobotStatus(mode: mode, battery: null, alert: 'none');
    notifyListeners();
  }
}

/// Records every robot/cmd/mode request (setMode is what publishes it).
class _Conn extends ConnectionProvider {
  final List<String> modes = [];
  @override
  bool get isConnected => true;
  @override
  void setMode(String mode) {
    modes.add(mode);
    super.setMode(mode);
  }
}

Future<(_Conn, _Robot)> _pumpControl(WidgetTester tester, String mode) async {
  tester.view.physicalSize = const Size(1080, 2400);
  tester.view.devicePixelRatio = 2.75;
  addTearDown(tester.view.reset);
  final conn = _Conn();
  final robot = _Robot(mode);
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
  await tester.pump(const Duration(seconds: 10));
  return (conn, robot);
}

Future<void> _unmount(WidgetTester tester) async {
  await tester.pumpWidget(const SizedBox());
  await tester.pump(const Duration(seconds: 10));
}

Color _bg(WidgetTester tester, String key) => tester
    .widget<ElevatedButton>(find.descendant(
        of: find.byKey(Key(key)), matching: find.byType(ElevatedButton)))
    .style!
    .backgroundColor!
    .resolve({})!;

void main() {
  test('robot/cmd/mode payload is {"mode": ...}, as the bridge reads it', () {
    expect(modePayload('manual'), '{"mode":"manual"}');
    expect(modePayload('stop'), '{"mode":"stop"}');
  });

  testWidgets('1. Manual publishes the manual mode', (tester) async {
    final (conn, _) = await _pumpControl(tester, 'stop');
    await tester.tap(find.byKey(const Key('mode_manual')));
    expect(conn.modes, ['manual']);
    await _unmount(tester);
  });

  testWidgets('1. Autónomo starts the mode chosen in the selector', (tester) async {
    final (conn, _) = await _pumpControl(tester, 'manual');
    await tester.tap(find.byKey(const Key('mode_auto')));
    expect(conn.modes, ['auto_parallel']); // default until another is picked

    conn.setMode('auto_random'); // what a card in the Modo tab does
    conn.modes.clear();
    await tester.tap(find.byKey(const Key('mode_auto')));
    expect(conn.modes, ['auto_random']);
    await _unmount(tester);
  });

  testWidgets('1. Stop does what the big STOP does (move stop + mode stop)', (tester) async {
    final (conn, _) = await _pumpControl(tester, 'manual');
    await tester.tap(find.byKey(const Key('mode_stop')));
    expect(conn.modes, ['stop']);
    await _unmount(tester);
  });

  testWidgets('2. joystick is gray with a hint unless the robot reports manual', (tester) async {
    for (final mode in ['idle', 'stop', 'auto_random']) {
      final (_, _) = await _pumpControl(tester, mode);
      expect(find.text('Elegí Manual para mover el robot'), findsOneWidget, reason: mode);
      expect(tester.widget<JoystickWidget>(find.byType(JoystickWidget)).modeEnabled, isFalse);
      expect(tester.widget<Opacity>(find.descendant(
              of: find.byType(JoystickWidget), matching: find.byType(Opacity)))
          .opacity, lessThan(1));
      await _unmount(tester);
    }
    final (_, robot) = await _pumpControl(tester, 'manual');
    expect(find.text('Elegí Manual para mover el robot'), findsNothing);
    expect(tester.widget<JoystickWidget>(find.byType(JoystickWidget)).modeEnabled, isTrue);
    expect(tester.widget<Opacity>(find.descendant(
            of: find.byType(JoystickWidget), matching: find.byType(Opacity)))
        .opacity, 1.0);

    robot.report('stop'); // the robot leaves manual: the stick locks again
    await tester.pump();
    expect(find.text('Elegí Manual para mover el robot'), findsOneWidget);
    await _unmount(tester);
  });

  testWidgets('3. the active button follows robot/status, not the last tap', (tester) async {
    final (_, robot) = await _pumpControl(tester, 'stop');
    expect(_bg(tester, 'mode_stop'), RoboColors.error);
    expect(_bg(tester, 'mode_manual'), Colors.white);

    await tester.tap(find.byKey(const Key('mode_manual')));
    await tester.pump();
    expect(_bg(tester, 'mode_manual'), Colors.white); // robot has not confirmed yet
    expect(_bg(tester, 'mode_stop'), RoboColors.error);

    robot.report('manual');
    await tester.pump();
    expect(_bg(tester, 'mode_manual'), RoboColors.primary);
    expect(_bg(tester, 'mode_stop'), Colors.white);

    robot.report('auto_perimeter');
    await tester.pump();
    expect(_bg(tester, 'mode_auto'), RoboColors.primary);
    expect(_bg(tester, 'mode_manual'), Colors.white);
    await _unmount(tester);
  });

  testWidgets('4. Aleatorio, Paralelas and Perimetral cards publish their mode', (tester) async {
    tester.view.physicalSize = const Size(1080, 2400);
    tester.view.devicePixelRatio = 2.75;
    addTearDown(tester.view.reset);
    final conn = _Conn();
    await tester.pumpWidget(MultiProvider(
      providers: [
        ChangeNotifierProvider<RobotStateProvider>.value(value: _Robot('idle')),
        ChangeNotifierProvider<ConnectionProvider>.value(value: conn),
      ],
      child: MaterialApp(
        theme: roboTheme(),
        home: Scaffold(body: ModeScreen(onSettingsTap: () {}, onNavigate: (_) {})),
      ),
    ));
    for (final t in ['Aleatorio', 'Líneas paralelas', 'Perimetral']) {
      await tester.tap(find.text(t));
    }
    expect(conn.modes, ['auto_random', 'auto_parallel', 'auto_perimeter']);
    expect(conn.selectedAutoMode, 'auto_perimeter');
    await _unmount(tester);
  });

  test('4. sendStop publishes the stop mode', () {
    final conn = _Conn();
    conn.sendStop();
    expect(conn.modes, ['stop']);
  });
}
