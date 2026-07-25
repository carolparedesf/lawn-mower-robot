import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:latlong2/latlong.dart';
import '../models/robot_position.dart';
import '../models/robot_status.dart';
import '../services/mqtt_service.dart';

const int _maxHistory = 200;

class RobotStateProvider extends ChangeNotifier {
  RobotPosition? _currentPosition;
  RobotStatus _currentStatus = const RobotStatus(
    mode: 'idle',
    battery: 0,
    alert: 'none',
  );
  final List<LatLng> _positionHistory = [];

  RobotPosition? get currentPosition => _currentPosition;
  RobotStatus get currentStatus => _currentStatus;
  List<LatLng> get positionHistory => List.unmodifiable(_positionHistory);

  StreamSubscription<RobotPosition>? _posSub;
  StreamSubscription<RobotStatus>? _statusSub;

  RobotStateProvider() {
    _posSub = MqttService.instance.positionStream.listen(_onPosition);
    _statusSub = MqttService.instance.statusStream.listen(_onStatus);
  }

  void _onPosition(RobotPosition pos) {
    _currentPosition = pos;
    _positionHistory.add(LatLng(pos.lat, pos.lng));
    if (_positionHistory.length > _maxHistory) {
      _positionHistory.removeAt(0);
    }
    notifyListeners();
  }

  void _onStatus(RobotStatus status) {
    _currentStatus = status;
    notifyListeners();
  }

  @override
  void dispose() {
    _posSub?.cancel();
    _statusSub?.cancel();
    super.dispose();
  }
}
