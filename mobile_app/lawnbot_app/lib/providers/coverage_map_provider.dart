import 'dart:async';
import 'dart:ui' as ui;
import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';
import '../models/coverage_map.dart';
import '../services/mqtt_service.dart';

/// Latest `robot/map` plus the layer toggles and the rendered overlay image.
class CoverageMapProvider extends ChangeNotifier {
  CoverageMapData? _data;
  DateTime? _received;
  CoverageLayers _layers = const CoverageLayers();
  ImageProvider? _image;
  int _renderSeq = 0;
  bool _disposed = false;
  StreamSubscription<CoverageMapData>? _sub;

  CoverageMapData? get data => _data;
  DateTime? get receivedAt => _received;
  CoverageLayers get layers => _layers;

  /// Rendered overlay (null until the first map is drawn).
  ImageProvider? get image => _image;

  CoverageMapProvider({Stream<CoverageMapData>? stream}) {
    _sub = (stream ?? MqttService.instance.mapStream).listen(_onMap);
  }

  void _onMap(CoverageMapData data) {
    _data = data;
    _received = DateTime.now();
    notifyListeners();
    _render();
  }

  void setLayers(CoverageLayers layers) {
    _layers = layers;
    notifyListeners();
    _render();
  }

  Future<void> _render() async {
    final data = _data;
    if (data == null || !data.canPlace) return;
    final seq = ++_renderSeq;
    final raster = data.toRaster(_layers);
    final png = await encodeRasterPng(raster);
    if (_disposed || seq != _renderSeq) return; // disposed, or a newer map/toggle superseded this one
    _image = MemoryImage(png);
    notifyListeners();
  }

  @override
  void dispose() {
    _disposed = true;
    _sub?.cancel();
    super.dispose();
  }
}

/// PNG bytes of a raster (RGBA, 8 bits).
Future<Uint8List> encodeRasterPng(CoverageRaster raster) async {
  final completer = Completer<ui.Image>();
  ui.decodeImageFromPixels(raster.rgba, raster.width, raster.height,
      ui.PixelFormat.rgba8888, completer.complete);
  final image = await completer.future;
  final bytes = await image.toByteData(format: ui.ImageByteFormat.png);
  image.dispose();
  return bytes!.buffer.asUint8List();
}
