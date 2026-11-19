import 'dart:async';
import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'package:mqtt_client/mqtt_client.dart';
import '../config/mqtt_config.dart';
import '../services/mqtt_service.dart';

/// Payload of robot/cmd/move for "no movement" (what the joystick sends on release).
String stopMovePayload() => jsonEncode({'dir': 'stop', 'speed': 0.0});

/// Payload of robot/cmd/mode (the bridge reads the "mode" key).
String modePayload(String mode) => jsonEncode({'mode': mode});

/// Autonomous mode started by the "Autónomo" button until the user picks another.
const String defaultAutoMode = 'auto_parallel';

class ConnectionProvider extends ChangeNotifier {
  String _selectedAutoMode = defaultAutoMode;
  bool _connected = false;
  bool _connecting = false;
  String _broker = MqttConfig.broker;

  /// Last autonomous mode chosen in the "Modo" tab (what "Autónomo" starts).
  String get selectedAutoMode => _selectedAutoMode;

  bool get isConnected => _connected;
  bool get isConnecting => _connecting;
  String get broker => _broker;

  StreamSubscription<bool>? _sub;

  ConnectionProvider() {
    _sub = MqttService.instance.connectionStream.listen((connected) {
      _connected = connected;
      notifyListeners();
    });
  }

  Future<void> connect({String? broker}) async {
    if (_connecting) return;
    if (broker != null) _broker = broker;
    _connecting = true;
    notifyListeners();

    final ok = await MqttService.instance.connect(_broker);
    _connected = ok;
    _connecting = false;
    notifyListeners();
  }

  void disconnect() {
    MqttService.instance.disconnect();
  }

  /// STOP: speed 0 on robot/cmd/move (same format as the joystick) and the
  /// "stop" mode on robot/cmd/mode (same as the Detener button).
  void sendStop() {
    MqttService.instance.publish(
      MqttConfig.topicCmdMove,
      stopMovePayload(),
    );
    setMode('stop');
  }

  /// Starts the autonomous mode picked in the "Modo" tab.
  void startAutonomous() => setMode(_selectedAutoMode);

  /// "Manual": the only mode in which the bridge forwards robot/cmd/move.
  void startManual() => setMode('manual');

  void setMode(String mode) {
    if (mode.startsWith('auto_') && mode != _selectedAutoMode) {
      _selectedAutoMode = mode;
      notifyListeners();
    }
    final payload = modePayload(mode);
    MqttService.instance.publish(
      MqttConfig.topicCmdMode,
      payload,
      qos: MqttQos.atLeastOnce,
    );
  }

  @override
  void dispose() {
    _sub?.cancel();
    super.dispose();
  }
}
