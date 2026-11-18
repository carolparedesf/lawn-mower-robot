import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:latlong2/latlong.dart';
import 'package:provider/provider.dart';
import 'package:robotmapapp/models/robot_position.dart';
import 'package:robotmapapp/providers/connection_provider.dart';
import 'package:robotmapapp/providers/coverage_map_provider.dart';
import 'package:robotmapapp/providers/geofence_provider.dart';
import 'package:robotmapapp/providers/robot_state_provider.dart';
import 'package:robotmapapp/screens/geofence_screen.dart';
import 'package:robotmapapp/utils/geo.dart';

const _corner = LatLng(-25.2867, -57.6470);

class _FakeRobot extends RobotStateProvider {
  final RobotPosition? pos;
  _FakeRobot(this.pos);
  @override
  RobotPosition? get currentPosition => pos;
}

Future<GeofenceProvider> _pump(WidgetTester tester, RobotPosition? pos) async {
  tester.view.physicalSize = const Size(900, 2200);
  tester.view.devicePixelRatio = 1.0;
  addTearDown(tester.view.reset);
  final fence = GeofenceProvider();
  await tester.pumpWidget(MultiProvider(
    providers: [
      ChangeNotifierProvider<RobotStateProvider>.value(value: _FakeRobot(pos)),
      ChangeNotifierProvider(create: (_) => ConnectionProvider()),
      ChangeNotifierProvider<GeofenceProvider>.value(value: fence),
      ChangeNotifierProvider(create: (_) => CoverageMapProvider()),
    ],
    child: MaterialApp(home: Scaffold(body: GeofenceScreen(onSettingsTap: () {}))),
  ));
  await tester.pump();
  return fence;
}

Future<void> _unmount(WidgetTester tester) async {
  await tester.pumpWidget(const SizedBox());
}

void main() {
  group('polygonSize', () {
    test('null without points, zero area with fewer than 3', () {
      expect(polygonSize([]), isNull);
      expect(polygonSize([_corner])!.area, 0);
    });

    test('axis-aligned rectangle', () {
      final s = polygonSize([
        _corner,
        localToLatLng(0, 20, _corner.latitude, _corner.longitude),
        localToLatLng(10, 20, _corner.latitude, _corner.longitude),
        localToLatLng(10, 0, _corner.latitude, _corner.longitude),
      ])!;
      expect(s.eastWest, closeTo(10, 1e-6));
      expect(s.northSouth, closeTo(20, 1e-6));
      expect(s.area, closeTo(200, 1e-6));
    });
  });

  test('RobotPosition.isFresh uses a 3 s limit', () {
    final now = DateTime(2026, 1, 1, 12);
    RobotPosition at(Duration ago) => RobotPosition(
        lat: 0, lng: 0, heading: 0, timestamp: now.subtract(ago));
    expect(at(const Duration(seconds: 2)).isFresh(now), isTrue);
    expect(at(const Duration(seconds: 3)).isFresh(now), isTrue);
    expect(at(const Duration(milliseconds: 3001)).isFresh(now), isFalse);
  });

  testWidgets('the rectangle-by-measures option is gone; tap-to-draw and the robot vertex stay',
      (tester) async {
    await _pump(tester, null);
    expect(find.text('Rectángulo por medidas'), findsNothing);
    expect(find.byKey(const Key('rectangle_by_measures')), findsNothing);
    expect(find.text('Vértice en el robot'), findsOneWidget);
    expect(find.text('Tocá el mapa para agregar puntos'), findsOneWidget);
    await _unmount(tester);
  });

  testWidgets('add-at-robot is disabled without a position', (tester) async {
    await _pump(tester, null);
    final chip =
        tester.widget<ActionChip>(find.byKey(const Key('add_vertex_at_robot')));
    expect(chip.onPressed, isNull);
    await _unmount(tester);
  });

  testWidgets('add-at-robot is disabled when the position is stale', (tester) async {
    await _pump(
        tester,
        RobotPosition(
            lat: -25.33,
            lng: -57.51,
            heading: 0,
            timestamp: DateTime.now().subtract(const Duration(seconds: 10))));
    final chip =
        tester.widget<ActionChip>(find.byKey(const Key('add_vertex_at_robot')));
    expect(chip.onPressed, isNull);
    await _unmount(tester);
  });

  testWidgets('add-at-robot adds the robot position as a vertex', (tester) async {
    final fence = await _pump(
        tester,
        RobotPosition(
            lat: -25.33, lng: -57.51, heading: 0, timestamp: DateTime.now()));
    await tester.tap(find.byKey(const Key('add_vertex_at_robot')));
    await tester.pump();
    expect(fence.points, [const LatLng(-25.33, -57.51)]);
    expect(find.byKey(const Key('geofence_size')), findsOneWidget);
    await _unmount(tester);
  });
}
