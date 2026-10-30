import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:latlong2/latlong.dart';
import 'package:provider/provider.dart';
import 'package:robotmapapp/models/coverage_map.dart';
import 'package:robotmapapp/providers/coverage_map_provider.dart';
import 'package:robotmapapp/utils/geo.dart';
import 'package:robotmapapp/widgets/coverage_overlay.dart';

/// test/fixtures/robot_map.json was produced by the robot's own code
/// (CoverageGrid.to_dict + origin_ll): 12 x 10 cells of 0.5 m, origin (-1, -1).
Map<String, dynamic> fixture() =>
    jsonDecode(File('test/fixtures/robot_map.json').readAsStringSync())
        as Map<String, dynamic>;

int alphaAt(CoverageRaster r, int px, int py) => r.rgba[(py * r.width + px) * 4 + 3];

void main() {
  group('CoverageMapData.fromJson (payload from the robot)', () {
    test('decodes geometry, layers and origin_ll', () {
      final m = CoverageMapData.fromJson(fixture());
      expect(m.width, 12);
      expect(m.height, 10);
      expect(m.resolution, 0.5);
      expect(m.originX, -1.0);
      expect(m.originY, -1.0);
      expect(m.passes.length, 120);
      expect(m.obstaclesSonar.length, 120);
      expect(m.obstaclesCamera.length, 120);
      expect(m.canPlace, isTrue);
      expect(m.originLat, -25.2867);
      expect(m.originLng, -57.647);
      expect(m.robot, [1.5, 1.25, 0.3]);
    });

    test('cell values match what the robot wrote (row 0 = south)', () {
      final m = CoverageMapData.fromJson(fixture());
      // y = 1.25 -> row 4; y = 2.25 -> row 6; x = 3.25 -> col 8
      expect(m.passes[4 * 12 + 4], greaterThan(1));
      expect(m.passes[6 * 12 + 4], 1);
      expect(m.passes[0], 0);
      expect(m.obstaclesSonar[4 * 12 + 9], 60);
      expect(m.obstaclesCamera[7 * 12 + 4], 50);
      expect(m.maxPasses, greaterThan(1));
    });

    test('a map without origin_ll still parses but cannot be placed', () {
      final j = fixture()..remove('origin_ll');
      final m = CoverageMapData.fromJson(j);
      expect(m.canPlace, isFalse);
      expect(m.bounds, isNull);
    });

    test('rejects an unknown encoding', () {
      final j = fixture()..['encoding'] = 'zlib';
      expect(() => CoverageMapData.fromJson(j), throwsFormatException);
    });

    test('rejects a layer whose size does not match width x height', () {
      final j = fixture()..['passes'] = base64Encode([1, 2, 3]);
      expect(() => CoverageMapData.fromJson(j), throwsFormatException);
    });

    test('rejects missing or invalid geometry', () {
      expect(() => CoverageMapData.fromJson(fixture()..remove('origin')), throwsFormatException);
      expect(() => CoverageMapData.fromJson(fixture()..['width'] = 0), throwsFormatException);
      expect(() => CoverageMapData.fromJson(fixture()..['resolution'] = 0), throwsFormatException);
    });

    test('ignores a malformed origin_ll instead of failing', () {
      final m = CoverageMapData.fromJson(fixture()..['origin_ll'] = ['x']);
      expect(m.canPlace, isFalse);
    });
  });

  group('placement on the map', () {
    test('local meters to lat/lng matches the robot formula', () {
      // Expected values computed with geofence_logic.lat_lon_to_local_xy's
      // formula (inverse): the Python side maps them back to (100, 50), etc.
      final a = localToLatLng(100, 50, -25.2867, -57.6470);
      expect(a.latitude, closeTo(-25.28624864341762, 1e-9));
      expect(a.longitude, closeTo(-57.646007094734024, 1e-9));
      final b = localToLatLng(-30, 200, -25.2867, -57.6470);
      expect(b.latitude, closeTo(-25.284894573670474, 1e-9));
      expect(b.longitude, closeTo(-57.64729787157979, 1e-9));
      final o = localToLatLng(0, 0, -25.2867, -57.6470);
      expect(o.latitude, -25.2867);
      expect(o.longitude, -57.6470);
    });

    test('grid bounds span the real size in meters, north is up', () {
      final m = CoverageMapData.fromJson(fixture());
      final b = m.bounds!;
      const d = Distance();
      // 12 cells x 0.5 m = 6 m wide, 10 x 0.5 = 5 m tall
      expect(d(LatLng(b.south, b.west), LatLng(b.south, b.east)), closeTo(6.0, 0.05));
      expect(d(LatLng(b.south, b.west), LatLng(b.north, b.west)), closeTo(5.0, 0.05));
      expect(b.north, greaterThan(b.south));
      expect(b.east, greaterThan(b.west));
      // origin (-1,-1) is the south-west corner: slightly south-west of origin_ll
      expect(b.south, lessThan(-25.2867));
      expect(b.west, lessThan(-57.6470));
    });
  });

  group('raster', () {
    test('size is cells x scale', () {
      final r = CoverageMapData.fromJson(fixture()).toRaster(const CoverageLayers(), scale: 4);
      expect(r.width, 48);
      expect(r.height, 40);
      expect(r.rgba.length, 48 * 40 * 4);
    });

    test('north is up: the cut row (y = 1.25, data row 4) is drawn at image row 5', () {
      final m = CoverageMapData.fromJson(fixture());
      final r = m.toRaster(const CoverageLayers(sonar: false, camera: false), scale: 1);
      // col 4, data row 4 -> image row (10 - 1 - 4) = 5
      expect(alphaAt(r, 4, 5), greaterThan(0));
      expect(alphaAt(r, 4, 4), 0);
      expect(alphaAt(r, 0, 9), 0, reason: 'data row 0 (south) is the bottom, untouched');
    });

    test('more passes are drawn more opaque', () {
      final m = CoverageMapData.fromJson(fixture());
      final r = m.toRaster(const CoverageLayers(sonar: false, camera: false), scale: 1);
      final once = alphaAt(r, 4, 10 - 1 - 6); // 1 pass
      final many = alphaAt(r, 4, 10 - 1 - 4); // several
      expect(many, greaterThan(once));
    });

    test('each layer can be switched off', () {
      final m = CoverageMapData.fromJson(fixture());
      final none = m.toRaster(const CoverageLayers(passes: false, sonar: false, camera: false));
      expect(none.rgba.every((v) => v == 0), isTrue);
      final sonarOnly = m.toRaster(const CoverageLayers(passes: false, camera: false), scale: 1);
      expect(alphaAt(sonarOnly, 9, 5), greaterThan(0)); // sonar obstacle at data row 4, col 9
      expect(alphaAt(sonarOnly, 4, 5), 0);
      final cameraOnly = m.toRaster(const CoverageLayers(passes: false, sonar: false), scale: 1);
      expect(alphaAt(cameraOnly, 4, 10 - 1 - 7), greaterThan(0));
      expect(alphaAt(cameraOnly, 9, 5), 0);
    });

    test('higher certainty is more opaque', () {
      final j = fixture();
      final m1 = CoverageMapData.fromJson(j);
      final low = m1.toRaster(const CoverageLayers(passes: false, camera: false), scale: 1);
      final sonar = List<int>.from(m1.obstaclesSonar);
      sonar[4 * 12 + 9] = 100;
      final j2 = fixture()..['obstacles_sonar'] = base64Encode(sonar);
      final high = CoverageMapData.fromJson(j2)
          .toRaster(const CoverageLayers(passes: false, camera: false), scale: 1);
      expect(alphaAt(high, 9, 5), greaterThan(alphaAt(low, 9, 5)));
    });
  });

  group('CoverageMapProvider and controls', () {
    testWidgets('keeps the latest map, toggles layers and renders an overlay image',
        (tester) async {
      final ctrl = StreamController<CoverageMapData>();
      final provider = CoverageMapProvider(stream: ctrl.stream);
      addTearDown(() {
        provider.dispose();
        ctrl.close();
      });
      expect(provider.data, isNull);
      ctrl.add(CoverageMapData.fromJson(fixture()));
      await tester.pump(); // deliver the stream event inside the test zone
      // The PNG is encoded by the engine (real async); the continuation runs in
      // the test's fake zone, so alternate real waiting with pumping.
      for (var i = 0; i < 50 && provider.image == null; i++) {
        await tester.runAsync(() => Future<void>.delayed(const Duration(milliseconds: 50)));
        await tester.pump();
      }
      expect(provider.data, isNotNull);
      expect(provider.image, isNotNull, reason: 'overlay PNG rendered');
      provider.setLayers(provider.layers.copyWith(sonar: false));
      expect(provider.layers.sonar, isFalse);
      expect(provider.layers.passes, isTrue);
    });

    testWidgets('controls are hidden until a map arrives, then show chips and the trend caption',
        (tester) async {
      final ctrl = StreamController<CoverageMapData>();
      final provider = CoverageMapProvider(stream: ctrl.stream);
      addTearDown(() {
        provider.dispose();
        ctrl.close();
      });
      await tester.pumpWidget(ChangeNotifierProvider.value(
        value: provider,
        child: const MaterialApp(home: Scaffold(body: CoverageControls())),
      ));
      expect(find.byKey(const Key('coverage_controls')), findsNothing);

      ctrl.add(CoverageMapData.fromJson(fixture()));
      await tester.pump();
      expect(find.byKey(const Key('coverage_controls')), findsOneWidget);
      expect(find.text('Pasadas'), findsOneWidget);
      expect(find.text('Sonar'), findsOneWidget);
      expect(find.text('Cámara'), findsOneWidget);
      expect(find.textContaining('tendencia'), findsOneWidget);
      expect(find.textContaining('2-3 m'), findsOneWidget);

      await tester.tap(find.text('Sonar'));
      await tester.pump();
      expect(provider.layers.sonar, isFalse);
      expect(provider.layers.passes, isTrue);
      await tester.tap(find.text('Pasadas'));
      await tester.pump();
      expect(provider.layers.passes, isFalse);
    });

    testWidgets('a map without origin_ll explains why it is not drawn', (tester) async {
      final ctrl = StreamController<CoverageMapData>();
      final provider = CoverageMapProvider(stream: ctrl.stream);
      addTearDown(() {
        provider.dispose();
        ctrl.close();
      });
      await tester.pumpWidget(ChangeNotifierProvider.value(
        value: provider,
        child: const MaterialApp(home: Scaffold(body: CoverageControls())),
      ));
      ctrl.add(CoverageMapData.fromJson(fixture()..remove('origin_ll')));
      await tester.pump();
      expect(find.textContaining('origen del mapa'), findsOneWidget);
      expect(provider.image, isNull);
    });
  });
}
