import 'package:flutter/material.dart';
import '../theme/robo_theme.dart';

/// Text for a battery percentage; "--" when the robot reports none.
String batteryLabel(int? battery) => battery == null ? '--' : '$battery%';

/// Bar/label color. Neutral grey when unknown (not the red of a dead battery).
Color batteryColorFor(int? battery) {
  if (battery == null) return RoboColors.muted;
  if (battery <= 20) return RoboColors.error;
  if (battery <= 45) return RoboColors.warning;
  return RoboColors.success;
}
