class MqttConfig {
  static const String broker = '10.183.212.234';
  static const int port = 1883;
  static const String clientId = 'robotcontrol_app';

  static const String topicCmdMove = 'robot/cmd/move';
  static const String topicCmdMode = 'robot/cmd/mode';
  static const String topicCmdGeofence = 'robot/cmd/geofence';
  static const String topicCmdActuator = 'robot/cmd/actuator';

  static const String topicPosition = 'robot/position';
  static const String topicStatus = 'robot/status';
}
