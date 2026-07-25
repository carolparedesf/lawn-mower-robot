import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'package:mqtt_client/mqtt_client.dart';
import '../config/mqtt_config.dart';
import '../models/actuator_command.dart';
import '../models/robot_status.dart';
import '../services/mqtt_service.dart';

/// Publishes actuator commands. Holds no local state — the confirmed
/// on/off state always comes back from RobotStateProvider.currentStatus,
/// same pattern as ConnectionProvider.setMode.
class ActuatorProvider extends ChangeNotifier {
  void setActuators(
    RobotStatus current, {
    bool? blade,
    bool? trimmer,
    bool? charging,
  }) {
    final command = ActuatorCommand(
      blade: blade ?? current.blade,
      trimmer: trimmer ?? current.trimmer,
      charging: charging ?? current.charging,
    );
    MqttService.instance.publish(
      MqttConfig.topicCmdActuator,
      jsonEncode(command.toJson()),
      qos: MqttQos.atLeastOnce,
    );
  }

  void emergencyStop() {
    MqttService.instance.publish(
      MqttConfig.topicCmdActuator,
      jsonEncode(
        const ActuatorCommand(
          blade: false,
          trimmer: false,
          charging: false,
        ).toJson(),
      ),
      qos: MqttQos.atLeastOnce,
    );
  }
}
