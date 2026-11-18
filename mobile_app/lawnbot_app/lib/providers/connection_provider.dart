import 'dart:async';
import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'package:mqtt_client/mqtt_client.dart';
import '../config/mqtt_config.dart';
import '../services/mqtt_service.dart';

/// Payload of robot/cmd/move for "no movement" (what the joystick sends on release).
String stopMovePayload() => jsonEncode({'dir': 'stop', 'speed': 0.0});

class ConnectionProvider extends ChangeNotifier {
  bool _connected = false;
  bool _connecting = false;
  String _broker = MqttConfig.broker;

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

  void setMode(String mode) {
    final payload = jsonEncode({'mode': mode});
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
