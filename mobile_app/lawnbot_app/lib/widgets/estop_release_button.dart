import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/connection_provider.dart';
import '../providers/robot_state_provider.dart';
import '../theme/robo_theme.dart';

const String estopConfirmText = '¿El robot está apoyado y sin nadie cerca?';

/// "Liberar parada de emergencia": only while the robot reports an e-stop (or
/// the "lifted" alert). It asks for confirmation before publishing the release;
/// the robot then stays in "stop" and the user taps Manual.
class EstopReleaseButton extends StatelessWidget {
  const EstopReleaseButton({super.key});

  Future<void> _ask(BuildContext context) async {
    final connection = context.read<ConnectionProvider>();
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        key: const Key('estop_dialog'),
        title: const Text('Liberar parada de emergencia'),
        content: const Text(estopConfirmText),
        actions: [
          TextButton(
            key: const Key('estop_cancel'),
            onPressed: () => Navigator.of(ctx).pop(false),
            child: const Text('Cancelar'),
          ),
          ElevatedButton(
            key: const Key('estop_confirm'),
            onPressed: () => Navigator.of(ctx).pop(true),
            style: ElevatedButton.styleFrom(
              backgroundColor: RoboColors.error,
              foregroundColor: Colors.white,
            ),
            child: const Text('Liberar'),
          ),
        ],
      ),
    );
    if (confirmed == true) connection.releaseEstop();
  }

  @override
  Widget build(BuildContext context) {
    final stopped =
        context.watch<RobotStateProvider>().currentStatus.isEmergencyStopped;
    if (!stopped) return const SizedBox.shrink();
    final connected = context.watch<ConnectionProvider>().isConnected;
    return Padding(
      padding: const EdgeInsets.only(bottom: 8),
      child: SizedBox(
        width: double.infinity,
        child: ElevatedButton.icon(
          key: const Key('estop_release_button'),
          onPressed: connected ? () => _ask(context) : null,
          icon: const Icon(Icons.lock_open),
          label: const FittedBox(
            fit: BoxFit.scaleDown,
            child: Text('Liberar parada de emergencia'),
          ),
          style: ElevatedButton.styleFrom(
            backgroundColor: RoboColors.error,
            foregroundColor: Colors.white,
            padding: const EdgeInsets.symmetric(vertical: 12),
            textStyle: const TextStyle(
              fontFamily: 'Roboto',
              fontWeight: FontWeight.w700,
              fontSize: 14,
            ),
          ),
        ),
      ),
    );
  }
}
