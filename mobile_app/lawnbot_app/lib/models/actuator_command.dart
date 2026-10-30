/// Reference model documenting the `robot/cmd/actuator` payload shape.
/// Mirrors the role of [RobotCommand]: the actual serialization is done
/// directly in ActuatorProvider, this exists as a documented contract.
class ActuatorCommand {
  final bool blade;
  final bool trimmer;
  final bool charging;

  /// Optional `force_cut`: overrides only the blade's grass condition. Sent
  /// only when true; a missing key means false for the robot.
  final bool forceCut;

  const ActuatorCommand({
    required this.blade,
    required this.trimmer,
    required this.charging,
    this.forceCut = false,
  });

  Map<String, dynamic> toJson() => {
        'blade': blade,
        'trimmer': trimmer,
        'charging': charging,
        if (forceCut) 'force_cut': true,
      };
}
