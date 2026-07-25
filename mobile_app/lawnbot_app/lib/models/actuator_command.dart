/// Reference model documenting the `robot/cmd/actuator` payload shape.
/// Mirrors the role of [RobotCommand]: the actual serialization is done
/// directly in ActuatorProvider, this exists as a documented contract.
class ActuatorCommand {
  final bool blade;
  final bool trimmer;
  final bool charging;

  const ActuatorCommand({
    required this.blade,
    required this.trimmer,
    required this.charging,
  });

  Map<String, dynamic> toJson() => {
        'blade': blade,
        'trimmer': trimmer,
        'charging': charging,
      };
}
