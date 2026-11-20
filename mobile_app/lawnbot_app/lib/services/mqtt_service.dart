import 'dart:async';
import 'dart:convert';
import 'package:mqtt_client/mqtt_client.dart';
import 'package:mqtt_client/mqtt_server_client.dart';
import '../config/mqtt_config.dart';
import '../models/coverage_map.dart';
import '../models/robot_position.dart';
import '../models/robot_status.dart';

class MqttService {
  MqttService._();
  static final MqttService instance = MqttService._();

  MqttServerClient? _client;

  final _positionController = StreamController<RobotPosition>.broadcast();
  final _statusController = StreamController<RobotStatus>.broadcast();
  final _mapController = StreamController<CoverageMapData>.broadcast();
  final _connectionController = StreamController<bool>.broadcast();

  Stream<RobotPosition> get positionStream => _positionController.stream;
  Stream<RobotStatus> get statusStream => _statusController.stream;
  Stream<CoverageMapData> get mapStream => _mapController.stream;
  Stream<bool> get connectionStream => _connectionController.stream;

  bool _connected = false;
  bool get isConnected => _connected;

  /// Connects (a new client each time, so it also serves to reconnect). It does
  /// NOT subscribe: call [requestState] after it returns true.
  Future<bool> connect(String broker) async {
    // Drop the previous client's callbacks: a late onDisconnected from it must
    // not flip the state of the new connection.
    final old = _client;
    if (old != null) {
      old.onDisconnected = null;
      old.onConnected = null;
      old.disconnect();
    }

    final client = MqttServerClient(broker, MqttConfig.clientId);
    _client = client;
    client.port = MqttConfig.port;
    // Short keep-alive: a cut link is noticed in ~10 s, not ~30 s.
    client.keepAlivePeriod = 8;
    client.onDisconnected = () {
      if (identical(client, _client)) _onDisconnected();
    };
    client.onConnected = () {
      if (identical(client, _client)) _onConnected();
    };
    client.logging(on: false);

    final connMsg = MqttConnectMessage()
        .withClientIdentifier(MqttConfig.clientId)
        .startClean()
        .withWillQos(MqttQos.atLeastOnce);
    client.connectionMessage = connMsg;

    try {
      await client.connect();
    } catch (_) {
      client.disconnect();
      return false;
    }

    if (client.connectionStatus?.state != MqttConnectionState.connected) {
      return false;
    }

    client.updates!.listen(_onMessage);
    return true;
  }

  /// Asks the robot for its state again: (re)subscribes to its topics. The
  /// broker re-sends the retained map right away; the status arrives with the
  /// bridge's next periodic robot/status. Must run after every (re)connect:
  /// the session is clean, so the broker forgets the subscriptions.
  void requestState() {
    if (!_connected) return;
    _subscribe(MqttConfig.topicPosition, MqttQos.atMostOnce);
    _subscribe(MqttConfig.topicStatus, MqttQos.atLeastOnce);
    // Retained by the broker: the last map arrives right after subscribing.
    _subscribe(MqttConfig.topicMap, MqttQos.atMostOnce);
  }

  void _subscribe(String topic, MqttQos qos) {
    _client?.subscribe(topic, qos);
  }

  void _onConnected() {
    _connected = true;
    _connectionController.add(true);
  }

  void _onDisconnected() {
    _connected = false;
    _connectionController.add(false);
  }

  void _onMessage(List<MqttReceivedMessage<MqttMessage>> messages) {
    for (final msg in messages) {
      final payload = MqttPublishPayload.bytesToStringAsString(
          (msg.payload as MqttPublishMessage).payload.message);

      try {
        final json = jsonDecode(payload) as Map<String, dynamic>;
        if (msg.topic == MqttConfig.topicPosition) {
          _positionController.add(RobotPosition.fromJson(json));
        } else if (msg.topic == MqttConfig.topicStatus) {
          _statusController.add(RobotStatus.fromJson(json));
        } else if (msg.topic == MqttConfig.topicMap) {
          _mapController.add(CoverageMapData.fromJson(json));
        }
      } catch (_) {}
    }
  }

  void publish(String topic, String payload, {MqttQos qos = MqttQos.atMostOnce}) {
    if (!_connected) return;
    final builder = MqttClientPayloadBuilder()..addString(payload);
    _client?.publishMessage(topic, qos, builder.payload!);
  }

  void disconnect() {
    _client?.disconnect();
  }

  void dispose() {
    _positionController.close();
    _statusController.close();
    _mapController.close();
    _connectionController.close();
  }
}
