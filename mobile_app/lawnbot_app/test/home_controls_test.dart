import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:provider/provider.dart';
import 'package:robotmapapp/models/robot_status.dart';
import 'package:robotmapapp/providers/actuator_provider.dart';
import 'package:robotmapapp/providers/connection_provider.dart';
import 'package:robotmapapp/providers/coverage_map_provider.dart';
import 'package:robotmapapp/providers/geofence_provider.dart';
import 'package:robotmapapp/providers/robot_state_provider.dart';
import 'package:robotmapapp/providers/speed_limit_provider.dart';
import 'package:robotmapapp/screens/home_screen.dart';
import 'package:robotmapapp/theme/robo_theme.dart';
import 'package:robotmapapp/utils/mode_labels.dart';

class _Robot extends RobotStateProvider {
  final RobotStatus st;
  _Robot(this.st);
  @override
  RobotStatus get currentStatus => st;
}

class _Conn extends ConnectionProvider {
  int stops = 0;
  @override
  bool get isConnected => true;
  @override
  void sendStop() => stops++;
}

Future<_Conn> _pump(WidgetTester tester, RobotStatus status) async {
  tester.view.physicalSize = const Size(1080, 2400);
  tester.view.devicePixelRatio = 2.75;
  addTearDown(tester.view.reset);
  final conn = _Conn();
  await tester.pumpWidget(MultiProvider(
    providers: [
      ChangeNotifierProvider<RobotStateProvider>.value(value: _Robot(status)),
      ChangeNotifierProvider<ConnectionProvider>.value(value: conn),
      ChangeNotifierProvider(create: (_) => GeofenceProvider()),
      ChangeNotifierProvider(create: (_) => ActuatorProvider()),
      ChangeNotifierProvider(create: (_) => CoverageMapProvider()),
      ChangeNotifierProvider(create: (_) => SpeedLimitProvider()),
    ],
    child: MaterialApp(theme: roboTheme(), home: const HomeScreen()),
  ));
  await tester.pump(const Duration(seconds: 10)); // the joystick's intro animation
  return conn;
}

Future<void> _unmount(WidgetTester tester) async {
  await tester.pumpWidget(const SizedBox());
  await tester.pump(const Duration(seconds: 10));
}

void main() {
  test('mode names in Spanish', () {
    expect(modeLabel('idle'), 'Detenido');
    expect(modeLabel('stop'), 'Detenido');
    expect(modeLabel('manual'), 'Manual');
    expect(modeLabel('auto_random'), 'Aleatorio');
    expect(modeLabel('auto_parallel'), 'Líneas paralelas');
    expect(modeLabel('auto_perimeter'), 'Perimetral');
    expect(modeLabel('algo_nuevo'), 'algo_nuevo'); // unknown: shown as received
  });

  test('STOP sends the same "no movement" payload the joystick sends on release', () {
    expect(stopMovePayload(), '{"dir":"stop","speed":0.0}');
  });

  testWidgets('STOP is visible on every screen and sends the stop command', (tester) async {
    final conn = await _pump(
        tester, const RobotStatus(mode: 'manual', battery: 50, alert: 'none'));
    for (final tab in ['Control', 'Modo', 'Cerco', 'Actuadores']) {
      await tester.tap(find.text(tab));
      await tester.pump(const Duration(milliseconds: 600));
      expect(find.byKey(const Key('stop_button')), findsOneWidget, reason: tab);
      final size = tester.getSize(find.byKey(const Key('stop_button')));
      expect(size.height, greaterThanOrEqualTo(56), reason: '$tab: big');
      expect(size.width, greaterThan(300), reason: '$tab: wide');
    }
    expect(conn.stops, 0);
    await tester.tap(find.byKey(const Key('stop_button')));
    await tester.pump();
    expect(conn.stops, 1);
    await _unmount(tester);
  });

  testWidgets('status bar: Spanish mode names, "--" without battery', (tester) async {
    const modes = {
      'idle': 'Detenido',
      'manual': 'Manual',
      'auto_random': 'Aleatorio',
      'auto_parallel': 'Líneas paralelas',
      'auto_perimeter': 'Perimetral',
      'stop': 'Detenido',
    };
    for (final e in modes.entries) {
      await _pump(tester, RobotStatus(mode: e.key, battery: null, alert: 'none'));
      expect(tester.widget<Text>(find.byKey(const Key('status_mode'))).data, 'Modo: ${e.value}', reason: e.key);
      expect(tester.widget<Text>(find.byKey(const Key('status_battery'))).data, '--', reason: 'no data: not 0 %');
      await _unmount(tester);
    }
    await _pump(tester, const RobotStatus(mode: 'manual', battery: 0, alert: 'none'));
    expect(tester.widget<Text>(find.byKey(const Key('status_battery'))).data, '0%'); // a real 0 is shown
    await _unmount(tester);
    await _pump(tester, const RobotStatus(mode: 'manual', battery: 62, alert: 'none'));
    expect(tester.widget<Text>(find.byKey(const Key('status_battery'))).data, '62%');
    await _unmount(tester);
  });

  testWidgets('pink theme: sampled colors and Roboto', (tester) async {
    final theme = roboTheme();
    expect(theme.colorScheme.primary, const Color(0xFFE91D8C));
    expect(RoboColors.appBar, const Color(0xFFE42189));
    expect(RoboColors.background, const Color(0xFFFEF8F8));
    expect(RoboColors.statusBar, const Color(0xFFFBE4EC));
    expect(theme.scaffoldBackgroundColor, RoboColors.background);
    expect(theme.textTheme.bodyMedium!.fontFamily, 'Roboto');
    await _pump(tester, const RobotStatus(mode: 'idle', battery: null, alert: 'none'));
    expect((tester.widget<Container>(find.byKey(const Key('top_bar'))).color), RoboColors.appBar);
    final strip = tester.widget<Container>(find.byKey(const Key('status_strip')));
    expect((strip.decoration as BoxDecoration).color, RoboColors.statusBar);
    final nav = tester.widget<BottomNavigationBar>(find.byKey(const Key('bottom_nav')));
    expect((nav.selectedItemColor, nav.unselectedItemColor), (RoboColors.primary, RoboColors.muted));
    await _unmount(tester);
  });
}
