import 'package:flutter/foundation.dart';

/// Joystick speed limit: the share of the full speed that is published.
enum SpeedLevel {
  slow('Lento', 0.3),
  medium('Medio', 0.6),
  fast('Rápido', 1.0);

  final String label;
  final double factor;
  const SpeedLevel(this.label, this.factor);

  /// "Lento 30 %"
  String get labelWithPercent => '$label ${(factor * 100).round()} %';
}

/// Selected joystick speed level. Starts on [SpeedLevel.slow] (safest).
class SpeedLimitProvider extends ChangeNotifier {
  SpeedLevel _level = SpeedLevel.slow;

  SpeedLevel get level => _level;
  double get factor => _level.factor;

  void setLevel(SpeedLevel level) {
    if (level == _level) return;
    _level = level;
    notifyListeners();
  }
}
