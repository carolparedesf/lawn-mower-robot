import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:latlong2/latlong.dart';
import 'package:provider/provider.dart';
import 'package:robotmapapp/models/geofence.dart';
import 'package:robotmapapp/models/robot_status.dart';
import 'package:robotmapapp/providers/geofence_provider.dart';
import 'package:robotmapapp/providers/robot_state_provider.dart';
import 'package:robotmapapp/utils/polygon_checks.dart';
import 'package:robotmapapp/widgets/geofence_notices.dart';

// About 0.0002 degrees = 22 m around Asuncion.
const double _lat = -25.3000;
const double _lng = -57.6000;
const double _d = 0.0002;

LatLng _p(double dLat, double dLng) => LatLng(_lat + dLat, _lng + dLng);

final List<LatLng> _square = [
  _p(0, 0),
  _p(0, _d),
  _p(_d, _d),
  _p(_d, 0),
];

void main() {
  group('Geofence.problem', () {
    test('square is valid', () {
      expect(Geofence(points: _square).problem, isNull);
      expect(Geofence(points: _square).isValid, isTrue);
    });

    test('clockwise square is valid too', () {
      expect(Geofence(points: _square.reversed.toList()).isValid, isTrue);
    });

    test('fewer than 4 points', () {
      expect(const Geofence(points: []).problem, GeofenceProblem.tooFewPoints);
      expect(Geofence(points: _square.sublist(0, 3)).problem,
          GeofenceProblem.tooFewPoints);
    });

    test('bow-tie is self intersecting', () {
      final bowTie = [_p(0, 0), _p(_d, _d), _p(0, _d), _p(_d, 0)];
      expect(Geofence(points: bowTie).problem, GeofenceProblem.selfIntersecting);
    });

    test('vertex touching another edge is self intersecting', () {
      final touching = [
        _p(0, 0),
        _p(0, _d),
        _p(_d, _d),
        _p(0, _d / 2),
        _p(_d, 0),
      ];
      expect(
          Geofence(points: touching).problem, GeofenceProblem.selfIntersecting);
    });

    test('edge folding back over the previous one is self intersecting', () {
      final spike = [_p(0, 0), _p(0, _d), _p(0, _d / 2), _p(_d, _d / 2), _p(_d, 0)];
      expect(Geofence(points: spike).problem, GeofenceProblem.selfIntersecting);
    });

    test('L shape is not convex', () {
      final l = [
        _p(0, 0),
        _p(0, _d),
        _p(_d / 2, _d),
        _p(_d / 2, _d / 2),
        _p(_d, _d / 2),
        _p(_d, 0),
      ];
      expect(Geofence(points: l).problem, GeofenceProblem.notConvex);
    });

    test('convex pentagon is valid', () {
      final pentagon = [
        _p(0, _d / 2),
        _p(_d / 3, _d),
        _p(_d, _d * 0.8),
        _p(_d, _d * 0.2),
        _p(_d / 3, 0),
      ];
      expect(Geofence(points: pentagon).isValid, isTrue);
    });

    test('a vertex in the middle of a side is allowed', () {
      final withMid = [_p(0, 0), _p(0, _d / 2), _p(0, _d), _p(_d, _d), _p(_d, 0)];
      expect(Geofence(points: withMid).isValid, isTrue);
    });

    test('self intersection is reported before convexity', () {
      expect(isConvex([_p(0, 0), _p(_d, _d), _p(0, _d), _p(_d, 0)]), isFalse);
      expect(
        Geofence(points: [_p(0, 0), _p(_d, _d), _p(0, _d), _p(_d, 0)]).problem,
        GeofenceProblem.selfIntersecting,
      );
    });

    test('payload format is unchanged', () {
      expect(Geofence(points: _square).toJson(), {
        'points': [
          {'lat': _lat, 'lng': _lng},
          {'lat': _lat, 'lng': _lng + _d},
          {'lat': _lat + _d, 'lng': _lng + _d},
          {'lat': _lat + _d, 'lng': _lng},
        ],
      });
    });
  });

  group('GeofenceProvider', () {
    test('canSend follows the geometry as points are added', () {
      final provider = GeofenceProvider();
      expect(provider.canSend, isFalse);
      for (final p in _square.sublist(0, 3)) {
        provider.addPoint(p);
        expect(provider.canSend, isFalse);
      }
      provider.addPoint(_square[3]);
      expect(provider.canSend, isTrue);

      provider.undo();
      expect(provider.canSend, isFalse);

      provider.clear();
      for (final p in [_p(0, 0), _p(_d, _d), _p(0, _d), _p(_d, 0)]) {
        provider.addPoint(p); // bow-tie
      }
      expect(provider.problem, GeofenceProblem.selfIntersecting);
      expect(provider.canSend, isFalse);

      provider.clear();
      expect(provider.problem, GeofenceProblem.tooFewPoints);
    });

    test('sendGeofence refuses an invalid polygon without publishing', () {
      final provider = GeofenceProvider();
      for (final p in [_p(0, 0), _p(_d, _d), _p(0, _d), _p(_d, 0)]) {
        provider.addPoint(p);
      }
      expect(provider.sendGeofence(), isFalse);
    });
  });

  group('RobotStatus geofence field', () {
    test('absent key stays null and is not serialized', () {
      final status = RobotStatus.fromJson({'mode': 'idle', 'battery': 80, 'alert': 'none'});
      expect(status.geofenceStatus, isNull);
      expect(status.toJson().containsKey('geofence'), isFalse);
    });

    test('reads the status sent by the robot', () {
      final status = RobotStatus.fromJson({
        'mode': 'idle',
        'battery': 80,
        'alert': 'none',
        'geofence': 'INVALID:autointerseccion entre las aristas 1 y 3',
      });
      expect(status.geofenceStatus, 'INVALID:autointerseccion entre las aristas 1 y 3');
    });
  });

  group('GeofenceProblemBanner', () {
    Future<GeofenceProvider> pump(WidgetTester tester) async {
      final provider = GeofenceProvider();
      await tester.pumpWidget(
        ChangeNotifierProvider<GeofenceProvider>.value(
          value: provider,
          child: const MaterialApp(home: Scaffold(body: GeofenceProblemBanner())),
        ),
      );
      return provider;
    }

    testWidgets('hidden with no points, hint with few points', (tester) async {
      final provider = await pump(tester);
      expect(find.byType(Text), findsNothing);

      provider.addPoint(_square[0]);
      await tester.pump();
      expect(find.textContaining('al menos 4'), findsOneWidget);
    });

    testWidgets('explains a crossing polygon and a non convex one', (tester) async {
      final provider = await pump(tester);
      for (final p in [_p(0, 0), _p(_d, _d), _p(0, _d), _p(_d, 0)]) {
        provider.addPoint(p);
      }
      await tester.pump();
      expect(find.textContaining('se cruza a sí mismo'), findsOneWidget);

      provider.clear();
      for (final p in [
        _p(0, 0),
        _p(0, _d),
        _p(_d / 2, _d),
        _p(_d / 2, _d / 2),
        _p(_d, _d / 2),
        _p(_d, 0),
      ]) {
        provider.addPoint(p);
      }
      await tester.pump();
      expect(find.textContaining('no es convexo'), findsOneWidget);
    });

    testWidgets('hidden once the polygon is valid', (tester) async {
      final provider = await pump(tester);
      for (final p in _square) {
        provider.addPoint(p);
      }
      await tester.pump();
      expect(find.byType(Text), findsNothing);
    });
  });

  group('GeofenceRobotStatus', () {
    Future<void> pump(WidgetTester tester, String? geofenceStatus) async {
      await tester.pumpWidget(
        ChangeNotifierProvider<RobotStateProvider>.value(
          value: _FakeRobotState(geofenceStatus),
          child: const MaterialApp(home: Scaffold(body: GeofenceRobotStatus())),
        ),
      );
    }

    testWidgets('hidden while the robot does not report it', (tester) async {
      await pump(tester, null);
      expect(find.byType(Text), findsNothing);
    });

    testWidgets('OK', (tester) async {
      await pump(tester, 'OK');
      expect(find.text('Cerco activo en el robot'), findsOneWidget);
    });

    testWidgets('INVALID shows the reason from the robot', (tester) async {
      await pump(tester, 'INVALID:autointerseccion entre las aristas 1 y 3');
      expect(
        find.text('El robot rechazó el cerco: autointerseccion entre las aristas 1 y 3'),
        findsOneWidget,
      );
    });

    testWidgets('waiting for the GPS transform', (tester) async {
      await pump(tester, 'WAITING_CONVERSION');
      expect(find.textContaining('esperando GPS'), findsOneWidget);
    });
  });
}

class _FakeRobotState extends RobotStateProvider {
  final String? _geofence;
  _FakeRobotState(this._geofence);

  @override
  RobotStatus get currentStatus => RobotStatus(
        mode: 'idle',
        battery: 0,
        alert: 'none',
        geofenceStatus: _geofence,
      );
}
