import 'dart:math' as math;
import 'package:flutter/material.dart';
import '../theme/robo_theme.dart';

/// Robot position marker: 🤖 emoji on a highlighted badge while connected,
/// a dimmed neutral icon otherwise. Rotates with `hdg` to hint heading.
class RobotMarker extends StatelessWidget {
  final bool isConnected;
  final double headingDegrees;

  const RobotMarker({
    super.key,
    required this.isConnected,
    required this.headingDegrees,
  });

  @override
  Widget build(BuildContext context) {
    return Transform.rotate(
      angle: headingDegrees * math.pi / 180,
      child: Container(
        decoration: BoxDecoration(
          color: isConnected ? RoboColors.amberAccent : Colors.grey.shade400,
          shape: BoxShape.circle,
          border: const Border.fromBorderSide(
            BorderSide(color: Colors.white, width: 3),
          ),
        ),
        child: Center(
          child: isConnected
              ? const Text('🤖', style: TextStyle(fontSize: 18))
              : const Icon(Icons.navigation, color: Colors.white, size: 18),
        ),
      ),
    );
  }
}
