import 'dart:async';
import 'dart:convert';
import 'package:mqtt_client/mqtt_client.dart';
import 'package:mqtt_client/mqtt_server_client.dart';
import '../config/mqtt_config.dart';
import '../models/robot_position.dart';
import '../models/robot_status.dart';

class MqttService {
  MqttService._();
  static final MqttService instance = MqttService._();

  late MqttServerClient _client;

  final _positionController = StreamController<RobotPosition>.broadcast();
  final _statusController = StreamController<RobotStatus>.broadcast();
  final _connectionController = StreamController<bool>.broadcast();

  Stream<RobotPosition> get positionStream => _positionController.stream;
  Stream<RobotStatus> get statusStream => _statusController.stream;
  Stream<bool> get connectionStream => _connectionController.stream;

  bool _connected = false;
  bool get isConnected => _connected;

  Future<bool> connect(String broker) async {
    _client = MqttServerClient(broker, MqttConfig.clientId);
    _client.port = MqttConfig.port;
    _client.keepAlivePeriod = 20;
    _client.onDisconnected = _onDisconnected;
    _client.onConnected = _onConnected;
    _client.logging(on: false);

    final connMsg = MqttConnectMessage()
        .withClientIdentifier(MqttConfig.clientId)
        .startClean()
        .withWillQos(MqttQos.atLeastOnce);
    _client.connectionMessage = connMsg;

    try {
      await _client.connect();
    } catch (_) {
      _client.disconnect();
      return false;
    }

    if (_client.connectionStatus?.state != MqttConnectionState.connected) {
      return false;
    }

    _subscribe(MqttConfig.topicPosition, MqttQos.atMostOnce);
    _subscribe(MqttConfig.topicStatus, MqttQos.atLeastOnce);

    _client.updates!.listen(_onMessage);
    return true;
  }

  void _subscribe(String topic, MqttQos qos) {
    _client.subscribe(topic, qos);
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
        }
      } catch (_) {}
    }
  }

  void publish(String topic, String payload, {MqttQos qos = MqttQos.atMostOnce}) {
    if (!_connected) return;
    final builder = MqttClientPayloadBuilder()..addString(payload);
    _client.publishMessage(topic, qos, builder.payload!);
  }

  void disconnect() {
    _client.disconnect();
  }

  void dispose() {
    _positionController.close();
    _statusController.close();
    _connectionController.close();
  }
}
