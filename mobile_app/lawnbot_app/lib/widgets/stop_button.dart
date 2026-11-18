import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/connection_provider.dart';
import '../theme/robo_theme.dart';

/// Big STOP button shown on every screen (above the bottom navigation). Sends
/// speed 0 on robot/cmd/move and the "stop" mode on robot/cmd/mode. Always
/// enabled: with no connection nothing is sent, but it never looks inactive.
class StopButton extends StatelessWidget {
  const StopButton({super.key});

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.fromLTRB(16, 8, 16, 8),
      child: SizedBox(
        width: double.infinity,
        height: 56,
        child: ElevatedButton.icon(
          key: const Key('stop_button'),
          onPressed: () => context.read<ConnectionProvider>().sendStop(),
          icon: const Icon(Icons.stop_circle, size: 30),
          label: const Text('STOP'),
          style: ElevatedButton.styleFrom(
            backgroundColor: RoboColors.error,
            foregroundColor: Colors.white,
            textStyle: const TextStyle(
              fontFamily: 'Roboto',
              fontWeight: FontWeight.w700,
              fontSize: 22,
              letterSpacing: 1.5,
            ),
          ),
        ),
      ),
    );
  }
}
