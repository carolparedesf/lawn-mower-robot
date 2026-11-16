import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:latlong2/latlong.dart';
import 'package:provider/provider.dart';
import 'package:robotmapapp/models/geofence.dart';
import 'package:robotmapapp/models/robot_position.dart';
import 'package:robotmapapp/providers/connection_provider.dart';
import 'package:robotmapapp/providers/coverage_map_provider.dart';
import 'package:robotmapapp/providers/geofence_provider.dart';
import 'package:robotmapapp/providers/robot_state_provider.dart';
import 'package:robotmapapp/screens/geofence_screen.dart';
import 'package:robotmapapp/utils/geo.dart';
import 'package:robotmapapp/widgets/rectangle_dialog.dart';

const _corner = LatLng(-25.2867, -57.6470);

double _dist(LatLng a, LatLng b) {
  final p = latLngToLocal(b, a.latitude, a.longitude);
  return (p.x * p.x + p.y * p.y).abs() == 0 ? 0 : _hyp(p.x, p.y);
}

double _hyp(double x, double y) => (x * x + y * y) <= 0 ? 0 : _sqrt(x * x + y * y);
double _sqrt(double v) {
  var r = v;
  for (var i = 0; i < 60; i++) {
    r = (r + v / r) / 2;
  }
  return r;
}

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
  group('rectangleFromCorner', () {
    test('bearing 0: length goes north, width goes east', () {
      final v = rectangleFromCorner(_corner, 10, 20, 0);
      expect(v.length, 4);
      expect(v[0], _corner);
      final p1 = latLngToLocal(v[1], _corner.latitude, _corner.longitude);
      final p3 = latLngToLocal(v[3], _corner.latitude, _corner.longitude);
      expect(p1.x, closeTo(0, 1e-6));
      expect(p1.y, closeTo(20, 1e-6));
      expect(p3.x, closeTo(10, 1e-6));
      expect(p3.y, closeTo(0, 1e-6));
    });

    test('bearing 90: length goes east, width goes south', () {
      final v = rectangleFromCorner(_corner, 10, 20, 90);
      final p1 = latLngToLocal(v[1], _corner.latitude, _corner.longitude);
      final p3 = latLngToLocal(v[3], _corner.latitude, _corner.longitude);
      expect(p1.x, closeTo(20, 1e-6));
      expect(p1.y, closeTo(0, 1e-6));
      expect(p3.x, closeTo(0, 1e-6));
      expect(p3.y, closeTo(-10, 1e-6));
    });

    test('any bearing gives a valid (simple, convex) fence with exact sides', () {
      for (final b in [0.0, 17.0, 45.0, 135.0, 270.0, 359.0]) {
        final v = rectangleFromCorner(_corner, 8, 15, b);
        expect(Geofence(points: v).isValid, isTrue, reason: 'bearing $b');
        expect(_dist(v[0], v[1]), closeTo(15, 1e-3));
        expect(_dist(v[1], v[2]), closeTo(8, 1e-3));
        final s = polygonSize(v)!;
        expect(s.area, closeTo(120, 1e-3));
      }
    });
  });

  group('polygonSize', () {
    test('null without points, zero area with fewer than 3', () {
      expect(polygonSize([]), isNull);
      expect(polygonSize([_corner])!.area, 0);
    });

    test('axis-aligned rectangle', () {
      final s = polygonSize(rectangleFromCorner(_corner, 10, 20, 0))!;
      expect(s.eastWest, closeTo(10, 1e-6));
      expect(s.northSouth, closeTo(20, 1e-6));
      expect(s.area, closeTo(200, 1e-6));
    });
  });

  test('parseMeasure accepts comma decimals and rejects garbage', () {
    expect(parseMeasure('12,5'), 12.5);
    expect(parseMeasure(' 3 '), 3);
    expect(parseMeasure('abc'), isNull);
    expect(parseMeasure(''), isNull);
  });

  test('RobotPosition.isFresh uses a 3 s limit', () {
    final now = DateTime(2026, 1, 1, 12);
    RobotPosition at(Duration ago) => RobotPosition(
        lat: 0, lng: 0, heading: 0, timestamp: now.subtract(ago));
    expect(at(const Duration(seconds: 2)).isFresh(now), isTrue);
    expect(at(const Duration(seconds: 3)).isFresh(now), isTrue);
    expect(at(const Duration(milliseconds: 3001)).isFresh(now), isFalse);
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

  testWidgets('rectangle dialog shows the vertices and returns them', (tester) async {
    List<LatLng>? result;
    await tester.pumpWidget(MaterialApp(
      home: Builder(
        builder: (context) => TextButton(
          onPressed: () async =>
              result = await showRectangleDialog(context, _corner),
          child: const Text('open'),
        ),
      ),
    ));
    await tester.tap(find.text('open'));
    await tester.pumpAndSettle();

    expect(find.byKey(const Key('rect_invalid')), findsOneWidget);
    expect(
        tester.widget<FilledButton>(find.byKey(const Key('rect_accept'))).onPressed,
        isNull);

    await tester.enterText(find.byKey(const Key('rect_width')), '10');
    await tester.enterText(find.byKey(const Key('rect_length')), '20,5');
    await tester.pump();
    for (var i = 0; i < 4; i++) {
      expect(find.byKey(Key('rect_vertex_$i')), findsOneWidget);
    }

    await tester.tap(find.byKey(const Key('rect_accept')));
    await tester.pumpAndSettle();
    expect(result, isNotNull);
    expect(result!.length, 4);
    final s = polygonSize(result!)!;
    expect(s.eastWest, closeTo(10, 1e-6));
    expect(s.northSouth, closeTo(20.5, 1e-6));
  });

  testWidgets('rectangle flow: pick a corner on the map, fill the dialog',
      (tester) async {
    final fence = await _pump(tester, null);
    await tester.tap(find.byKey(const Key('rectangle_by_measures')));
    await tester.pump();
    expect(find.text('Tocá el mapa en una esquina del rectángulo'), findsOneWidget);

    await tester.tapAt(const Offset(450, 900));
    await tester.pumpAndSettle();
    expect(find.byType(RectangleDialog), findsOneWidget);
    expect(fence.points, isEmpty); // nothing added before accepting

    await tester.enterText(find.byKey(const Key('rect_width')), '5');
    await tester.enterText(find.byKey(const Key('rect_length')), '5');
    await tester.pump();
    await tester.tap(find.byKey(const Key('rect_accept')));
    await tester.pumpAndSettle();

    expect(fence.points.length, 4);
    expect(fence.canSend, isTrue);
    await _unmount(tester);
  });
}
