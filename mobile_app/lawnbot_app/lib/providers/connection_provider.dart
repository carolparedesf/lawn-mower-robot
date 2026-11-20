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

/// Payload of robot/cmd/estop. The bridge only accepts the release.
String estopReleasePayload() => jsonEncode({'active': false});

/// Autonomous mode started by the "Autónomo" button until the user picks another.
const String defaultAutoMode = 'auto_parallel';

/// Time between reconnection attempts after the link drops.
const Duration reconnectInterval = Duration(seconds: 2);

class ConnectionProvider extends ChangeNotifier {
  String _selectedAutoMode = defaultAutoMode;
  bool _connected = false;
  bool _connecting = false;
  String _broker = MqttConfig.broker;

  // Automatic reconnection: after a drop (never after the user's own
  // disconnect() or if there was no connection before), retry every
  // [retryEvery] with no user action, until it comes back.
  final Future<bool> Function(String broker) _connector;
  final void Function() _requestState;
  final void Function(String topic, String payload, {MqttQos qos}) _publish;
  /// Time between attempts (2 s; tests shorten it).
  final Duration retryEvery;
  Timer? _retryTimer;
  bool _everConnected = false;
  bool _userDisconnected = false;
  bool _reconnecting = false;
  bool _attemptRunning = false;

  /// Last autonomous mode chosen in the "Modo" tab (what "Autónomo" starts).
  String get selectedAutoMode => _selectedAutoMode;

  bool get isConnected => _connected;
  bool get isConnecting => _connecting;

  /// True while the app retries by itself after a dropped connection (the chip
  /// shows "Reconectando...").
  bool get isReconnecting => _reconnecting;
  String get broker => _broker;

  StreamSubscription<bool>? _sub;

  /// The optional parameters are for tests (a fake link and a short interval).
  ConnectionProvider({
    Stream<bool>? connectionStream,
    Future<bool> Function(String broker)? connector,
    void Function()? requestState,
    void Function(String topic, String payload, {MqttQos qos})? publisher,
    this.retryEvery = reconnectInterval,
  })  : _connector = connector ?? MqttService.instance.connect,
        _requestState = requestState ?? MqttService.instance.requestState,
        _publish = publisher ?? MqttService.instance.publish {
    _sub = (connectionStream ?? MqttService.instance.connectionStream)
        .listen(_onLinkChanged);
  }

  void _onLinkChanged(bool connected) {
    _connected = connected;
    if (connected) {
      _everConnected = true;
    } else if (_everConnected && !_userDisconnected) {
      _startRetrying();
    }
    notifyListeners();
  }

  Future<void> connect({String? broker}) async {
    if (_connecting) return;
    if (broker != null) _broker = broker;
    _userDisconnected = false;
    _stopRetrying();
    _connecting = true;
    notifyListeners();

    final ok = await _connector(_broker);
    _connected = ok;
    if (ok) {
      _everConnected = true;
      _requestState();
    }
    _connecting = false;
    notifyListeners();
  }

  void disconnect() {
    _userDisconnected = true;
    _stopRetrying();
    MqttService.instance.disconnect();
    notifyListeners();
  }

  void _startRetrying() {
    if (_retryTimer != null) return;
    _reconnecting = true;
    _retryTimer = Timer.periodic(retryEvery, (_) => _retryOnce());
  }

  void _stopRetrying() {
    _retryTimer?.cancel();
    _retryTimer = null;
    _reconnecting = false;
  }

  /// One attempt; attempts never overlap (a slow one skips the next ticks).
  Future<void> _retryOnce() async {
    if (_attemptRunning || _connecting || _connected) return;
    _attemptRunning = true;
    final ok = await _connector(_broker);
    _attemptRunning = false;
    if (_retryTimer == null) return; // stopped meanwhile (user / dispose)
    if (ok) {
      _stopRetrying();
      _connected = true;
      // The session is clean: ask the robot for its state again.
      _requestState();
      notifyListeners();
    }
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

  /// Releases the robot's e-stop (robot/cmd/estop, QoS 1). The bridge leaves
  /// the mode in "stop": the user taps Manual afterwards, it does not go back
  /// to manual by itself.
  void releaseEstop() {
    _publish(
      MqttConfig.topicCmdEstop,
      estopReleasePayload(),
      qos: MqttQos.atLeastOnce,
    );
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
    _retryTimer?.cancel();
    _retryTimer = null;
    _sub?.cancel();
    super.dispose();
  }
}
