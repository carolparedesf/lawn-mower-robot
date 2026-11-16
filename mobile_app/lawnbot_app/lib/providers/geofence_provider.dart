import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'package:latlong2/latlong.dart';
import 'package:mqtt_client/mqtt_client.dart';
import '../config/mqtt_config.dart';
import '../models/geofence.dart';
import '../services/mqtt_service.dart';

class GeofenceProvider extends ChangeNotifier {
  final List<LatLng> _points = [];

  List<LatLng> get points => List.unmodifiable(_points);

  /// Why the current polygon cannot be sent, or null if it can.
  GeofenceProblem? get problem => Geofence(points: _points).problem;
  bool get canSend => problem == null;

  void addPoint(LatLng point) {
    _points.add(point);
    notifyListeners();
  }

  /// Replaces the whole polygon (used by the rectangle-by-measures flow).
  void setPoints(List<LatLng> points) {
    _points
      ..clear()
      ..addAll(points);
    notifyListeners();
  }

  void undo() {
    if (_points.isEmpty) return;
    _points.removeLast();
    notifyListeners();
  }

  void clear() {
    _points.clear();
    notifyListeners();
  }

  bool sendGeofence() {
    final fence = Geofence(points: List.from(_points));
    if (!fence.isValid) return false;

    final payload = jsonEncode(fence.toJson());
    MqttService.instance.publish(
      MqttConfig.topicCmdGeofence,
      payload,
      qos: MqttQos.atLeastOnce,
    );
    return true;
  }
}
