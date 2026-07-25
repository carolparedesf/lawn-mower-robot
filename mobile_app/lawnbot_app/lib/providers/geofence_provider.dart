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
  bool get canSend => _points.length >= 3;

  void addPoint(LatLng point) {
    _points.add(point);
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
