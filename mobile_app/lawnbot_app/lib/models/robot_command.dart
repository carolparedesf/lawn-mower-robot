class RobotCommand {
  final String direction;
  final double speed;
  final String mode;

  const RobotCommand({
    required this.direction,
    required this.speed,
    required this.mode,
  });

  factory RobotCommand.fromJson(Map<String, dynamic> json) {
    return RobotCommand(
      direction: json['dir'] as String? ?? 'stop',
      speed: (json['speed'] as num?)?.toDouble() ?? 0.0,
      mode: json['mode'] as String? ?? 'manual',
    );
  }

  Map<String, dynamic> toJson() => {
        'dir': direction,
        'speed': speed,
        'mode': mode,
      };
}
