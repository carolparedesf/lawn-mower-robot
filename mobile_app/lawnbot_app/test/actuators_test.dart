import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:provider/provider.dart';
import 'package:robotmapapp/models/actuator_command.dart';
import 'package:robotmapapp/models/blade_status.dart';
import 'package:robotmapapp/models/robot_status.dart';
import 'package:robotmapapp/providers/actuator_provider.dart';
import 'package:robotmapapp/providers/connection_provider.dart';
import 'package:robotmapapp/providers/robot_state_provider.dart';
import 'package:robotmapapp/screens/actuators_screen.dart';

class _FakeState extends RobotStateProvider {
  final RobotStatus status;
  _FakeState(this.status);

  @override
  RobotStatus get currentStatus => status;
}

class _Call {
  final bool? blade, trimmer, forceCut;
  _Call(this.blade, this.trimmer, this.forceCut);
}

class _FakeActuators extends ActuatorProvider {
  final calls = <_Call>[];

  @override
  void setActuators(RobotStatus current,
      {bool? blade, bool? trimmer, bool? charging, bool? forceCut}) {
    calls.add(_Call(blade, trimmer, forceCut));
  }
}

Future<_FakeActuators> _pump(WidgetTester tester, RobotStatus status) async {
  final actuators = _FakeActuators();
  tester.view.physicalSize = const Size(800, 2400);
  tester.view.devicePixelRatio = 1.0;
  addTearDown(tester.view.reset);
  await tester.pumpWidget(MultiProvider(
    providers: [
      ChangeNotifierProvider<RobotStateProvider>.value(value: _FakeState(status)),
      ChangeNotifierProvider<ActuatorProvider>.value(value: actuators),
      ChangeNotifierProvider(create: (_) => ConnectionProvider()),
    ],
    child: MaterialApp(home: Scaffold(body: ActuatorsScreen(onSettingsTap: () {}))),
  ));
  return actuators;
}

List<Switch> _switches(WidgetTester tester) =>
    tester.widgetList<Switch>(find.byType(Switch)).toList();

void main() {
  group('ActuatorCommand contract', () {
    test('existing payload keys are unchanged', () {
      const c = ActuatorCommand(blade: true, trimmer: false, charging: false);
      expect(c.toJson(), {'blade': true, 'trimmer': false, 'charging': false});
    });

    test('force_cut is an optional key, only sent when true', () {
      const on = ActuatorCommand(
          blade: true, trimmer: false, charging: false, forceCut: true);
      expect(on.toJson()['force_cut'], isTrue);
      const off = ActuatorCommand(
          blade: true, trimmer: false, charging: false, forceCut: false);
      expect(off.toJson().containsKey('force_cut'), isFalse);
    });
  });

  group('RobotStatus actuator keys', () {
    test('new optional keys parse; absent keys default to off', () {
      final s = RobotStatus.fromJson({
        'mode': 'manual',
        'blade': true,
        'blade_running': false,
        'trimmer_running': true,
        'force_cut': true,
        'blade_status': 'cutter=OFF(sin_pasto) | trimmer=ON duty=0.30',
      });
      expect(s.blade, isTrue);
      expect(s.bladeRunning, isFalse);
      expect(s.trimmerRunning, isTrue);
      expect(s.forceCut, isTrue);
      expect(s.bladeStatus, contains('sin_pasto'));
      final old = RobotStatus.fromJson({'mode': 'idle', 'blade': true});
      expect(old.bladeRunning, isFalse);
      expect(old.forceCut, isFalse);
      expect(old.bladeStatus, isNull);
    });

    test('actuators are usable in manual and automatic modes, not idle/stop', () {
      bool usable(String m) => RobotStatus.fromJson({'mode': m}).canUseActuators;
      expect(usable('manual'), isTrue);
      expect(usable('auto_parallel'), isTrue);
      expect(usable('auto_random'), isTrue);
      expect(usable('auto_perimeter'), isTrue);
      expect(usable('idle'), isFalse);
      expect(usable('stop'), isFalse);
    });
  });

  group('parseBladeStatus', () {
    test('parses on/off, duty and reasons', () {
      final m = parseBladeStatus(
          'cutter=OFF(sin_pasto,levantado) | trimmer=ON duty=0.30');
      expect(m['cutter']!.on, isFalse);
      expect(m['cutter']!.reasons, ['sin_pasto', 'levantado']);
      expect(m['trimmer']!.on, isTrue);
      expect(m['trimmer']!.duty, 0.30);
    });

    test('null, empty and garbage give an empty map', () {
      expect(parseBladeStatus(null), isEmpty);
      expect(parseBladeStatus(''), isEmpty);
      expect(parseBladeStatus('???'), isEmpty);
    });

    test('whyNotRunning ignores "sin_pedido" and translates the rest', () {
      final m = parseBladeStatus('cutter=OFF(sin_pedido) | trimmer=OFF(fuera_del_cerco)');
      expect(whyNotRunning(m['cutter']), isNull);
      expect(whyNotRunning(m['trimmer']), 'fuera del cerco');
      expect(whyNotRunning(null), isNull);
    });
  });

  group('ActuatorsScreen', () {
    testWidgets('switches are enabled in manual mode', (tester) async {
      await _pump(tester, RobotStatus.fromJson({'mode': 'manual'}));
      final sw = _switches(tester);
      expect(sw.length, greaterThanOrEqualTo(3));
      for (final s in sw.take(3)) {
        expect(s.onChanged, isNotNull);
      }
      expect(find.textContaining('solo en modo manual o'), findsNothing);
    });

    testWidgets('switches are disabled in idle with an explanation',
        (tester) async {
      await _pump(tester, RobotStatus.fromJson({'mode': 'idle'}));
      for (final s in _switches(tester).take(3)) {
        expect(s.onChanged, isNull);
      }
      expect(find.textContaining('solo en modo manual o'), findsOneWidget);
    });

    testWidgets('requested and running are shown separately, with the reason',
        (tester) async {
      await _pump(
          tester,
          RobotStatus.fromJson({
            'mode': 'auto_random',
            'blade': true,
            'blade_running': false,
            'blade_status': 'cutter=OFF(sin_pasto) | trimmer=OFF(sin_pedido)',
          }));
      expect(find.text('Pedido: sí  ·  Corriendo: no'), findsOneWidget);
      expect(find.text('No corre: no detecta pasto'), findsOneWidget);
    });

    testWidgets('blade switch sends only the blade change', (tester) async {
      final a = await _pump(tester, RobotStatus.fromJson({'mode': 'manual'}));
      await tester.tap(find.byType(Switch).at(0));
      await tester.pump();
      expect(a.calls.length, 1);
      expect(a.calls.single.blade, isTrue);
      expect(a.calls.single.trimmer, isNull);
      expect(a.calls.single.forceCut, isNull);
    });

    testWidgets('force cut asks for confirmation; cancel sends nothing',
        (tester) async {
      final a = await _pump(tester, RobotStatus.fromJson({'mode': 'manual'}));
      await tester.tap(find.byType(Switch).at(2));
      await tester.pumpAndSettle();
      expect(find.text('¿Cortar sin detectar pasto?'), findsOneWidget);
      await tester.tap(find.text('Cancelar'));
      await tester.pumpAndSettle();
      expect(a.calls, isEmpty);
    });

    testWidgets('force cut is sent only after confirming', (tester) async {
      final a = await _pump(tester, RobotStatus.fromJson({'mode': 'manual'}));
      await tester.tap(find.byType(Switch).at(2));
      await tester.pumpAndSettle();
      await tester.tap(find.text('Forzar corte'));
      await tester.pumpAndSettle();
      expect(a.calls.length, 1);
      expect(a.calls.single.forceCut, isTrue);
    });

    testWidgets('turning force cut off needs no confirmation', (tester) async {
      final a = await _pump(
          tester, RobotStatus.fromJson({'mode': 'manual', 'force_cut': true}));
      await tester.tap(find.byType(Switch).at(2));
      await tester.pump();
      expect(find.text('¿Cortar sin detectar pasto?'), findsNothing);
      expect(a.calls.single.forceCut, isFalse);
    });

    testWidgets('battery "--" when missing and no estimated hours',
        (tester) async {
      await _pump(tester, RobotStatus.fromJson({'mode': 'manual'}));
      expect(find.text('--'), findsNWidgets(2)); // top bar + battery card
      expect(find.textContaining('Autonomía'), findsNothing);
      expect(find.textContaining('h aprox'), findsNothing);
    });
  });
}
