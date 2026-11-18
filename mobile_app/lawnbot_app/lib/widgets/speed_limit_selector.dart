import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/speed_limit_provider.dart';
import '../theme/robo_theme.dart';

/// Three-level joystick speed limit: slow 30 %, medium 60 %, fast 100 %.
class SpeedLimitSelector extends StatelessWidget {
  const SpeedLimitSelector({super.key});

  @override
  Widget build(BuildContext context) {
    final limit = context.watch<SpeedLimitProvider>();
    return SegmentedButton<SpeedLevel>(
      key: const Key('speed_limit'),
      showSelectedIcon: false,
      style: SegmentedButton.styleFrom(
        visualDensity: VisualDensity.compact,
        selectedBackgroundColor: RoboColors.primary,
        selectedForegroundColor: Colors.white,
        foregroundColor: RoboColors.textSecondary,
        side: const BorderSide(color: RoboColors.borderStrong),
        textStyle: const TextStyle(
            fontFamily: 'Roboto', fontSize: 12, fontWeight: FontWeight.w500),
      ),
      segments: [
        for (final level in SpeedLevel.values)
          ButtonSegment(
            value: level,
            label: Text(level.labelWithPercent, key: Key('speed_${level.name}')),
          ),
      ],
      selected: {limit.level},
      onSelectionChanged: (s) => limit.setLevel(s.first),
    );
  }
}
