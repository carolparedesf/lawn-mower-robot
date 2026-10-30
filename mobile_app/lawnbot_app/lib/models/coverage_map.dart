import 'dart:convert';
import 'dart:typed_data';
import 'package:flutter_map/flutter_map.dart';
import '../utils/geo.dart';

/// Which layers of the coverage map to draw.
class CoverageLayers {
  final bool passes;
  final bool sonar;
  final bool camera;

  const CoverageLayers({
    this.passes = true,
    this.sonar = true,
    this.camera = true,
  });

  CoverageLayers copyWith({bool? passes, bool? sonar, bool? camera}) =>
      CoverageLayers(
        passes: passes ?? this.passes,
        sonar: sonar ?? this.sonar,
        camera: camera ?? this.camera,
      );
}

/// Pixels of the rendered overlay (RGBA, north up).
class CoverageRaster {
  final int width;
  final int height;
  final Uint8List rgba;
  const CoverageRaster(this.width, this.height, this.rgba);
}

/// `robot/map` payload (published by the robot's coverage_map node).
///
/// Grid of `width x height` cells of `resolution` m, row-major starting at
/// `origin` (x = east, y = north, in the robot's `map` frame): row 0 is the
/// southernmost. Layers are base64 u8: `passes` (completed passes 0..255),
/// `obstacles_sonar` and `obstacles_camera` (certainty 0..100).
/// `origin_ll` [lat, lng] (optional) is where the map frame's (0, 0) is on
/// Earth; without it the grid cannot be placed on the map.
class CoverageMapData {
  final double resolution;
  final double originX;
  final double originY;
  final int width;
  final int height;
  final Uint8List passes;
  final Uint8List obstaclesSonar;
  final Uint8List obstaclesCamera;
  final double? originLat;
  final double? originLng;

  /// Robot pose in the map frame [x, y, yaw], if known.
  final List<double>? robot;

  const CoverageMapData({
    required this.resolution,
    required this.originX,
    required this.originY,
    required this.width,
    required this.height,
    required this.passes,
    required this.obstaclesSonar,
    required this.obstaclesCamera,
    this.originLat,
    this.originLng,
    this.robot,
  });

  /// Throws [FormatException] if the payload is not a valid map.
  factory CoverageMapData.fromJson(Map<String, dynamic> json) {
    final encoding = json['encoding'];
    if (encoding != 'base64-u8-row-major') {
      throw FormatException('unsupported map encoding: $encoding');
    }
    final width = (json['width'] as num?)?.toInt() ?? 0;
    final height = (json['height'] as num?)?.toInt() ?? 0;
    final resolution = (json['resolution'] as num?)?.toDouble() ?? 0.0;
    final origin = json['origin'];
    if (width <= 0 || height <= 0 || resolution <= 0 || origin is! List || origin.length != 2) {
      throw const FormatException('invalid map geometry');
    }
    // Defensive cap: a 200 x 200 m fence at 0.5 m is 400 x 400 cells.
    if (width * height > 1000000) {
      throw const FormatException('map too large');
    }
    Uint8List layer(String key) {
      final bytes = base64Decode(json[key] as String);
      if (bytes.length != width * height) {
        throw FormatException(
            'layer $key has ${bytes.length} bytes, expected ${width * height}');
      }
      return bytes;
    }

    final ll = json['origin_ll'];
    double? lat, lng;
    if (ll is List && ll.length == 2 && ll[0] is num && ll[1] is num) {
      lat = (ll[0] as num).toDouble();
      lng = (ll[1] as num).toDouble();
    }
    final robot = json['robot'];
    return CoverageMapData(
      resolution: resolution,
      originX: (origin[0] as num).toDouble(),
      originY: (origin[1] as num).toDouble(),
      width: width,
      height: height,
      passes: layer('passes'),
      obstaclesSonar: layer('obstacles_sonar'),
      obstaclesCamera: layer('obstacles_camera'),
      originLat: lat,
      originLng: lng,
      robot: robot is List && robot.length == 3
          ? robot.map((e) => (e as num).toDouble()).toList()
          : null,
    );
  }

  /// True when the robot reported where the map frame is on Earth.
  bool get canPlace => originLat != null && originLng != null;

  /// Geographic rectangle of the grid, or null without `origin_ll`.
  LatLngBounds? get bounds {
    if (!canPlace) return null;
    final sw = localToLatLng(originX, originY, originLat!, originLng!);
    final ne = localToLatLng(originX + width * resolution,
        originY + height * resolution, originLat!, originLng!);
    return LatLngBounds(sw, ne);
  }

  int get maxPasses => passes.fold(0, (m, v) => v > m ? v : m);

  /// Renders the enabled layers to RGBA, north up, each cell [scale] x
  /// [scale] pixels (nearest-neighbor, so cells stay square and crisp).
  /// Transparent where nothing is drawn. Passes in green (darker = more),
  /// sonar obstacles in red and camera obstacles in orange on top.
  CoverageRaster toRaster(CoverageLayers layers, {int scale = 4}) {
    final w = width * scale, h = height * scale;
    final out = Uint8List(w * h * 4);
    for (var row = 0; row < height; row++) {
      final py0 = (height - 1 - row) * scale; // row 0 is the south edge
      for (var col = 0; col < width; col++) {
        final i = row * width + col;
        int r = 0, g = 0, b = 0, a = 0;
        if (layers.passes && passes[i] > 0) {
          final n = passes[i] > 3 ? 3 : passes[i];
          r = 76 - n * 14; // 4CAF50 -> darker green with more passes
          g = 175 - n * 28;
          b = 80 - n * 14;
          a = 60 + n * 45;
        }
        if (layers.sonar && obstaclesSonar[i] > 0) {
          r = 239; g = 83; b = 80;
          a = 90 + (obstaclesSonar[i] * 1.5).round().clamp(0, 150);
        }
        if (layers.camera && obstaclesCamera[i] > 0) {
          r = 255; g = 152; b = 0;
          a = 90 + (obstaclesCamera[i] * 1.5).round().clamp(0, 150);
        }
        if (a == 0) continue;
        for (var dy = 0; dy < scale; dy++) {
          var o = ((py0 + dy) * w + col * scale) * 4;
          for (var dx = 0; dx < scale; dx++, o += 4) {
            out[o] = r;
            out[o + 1] = g;
            out[o + 2] = b;
            out[o + 3] = a;
          }
        }
      }
    }
    return CoverageRaster(w, h, out);
  }
}
