import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/connection_provider.dart';
import '../providers/robot_state_provider.dart';
import '../theme/robo_theme.dart';
import '../widgets/mode_card.dart';
import '../widgets/robo_bottom_nav.dart';
import '../widgets/robo_top_bar.dart';

/// "Modo" tab — autonomous submode picker. Tapping a card publishes
/// immediately via ConnectionProvider.setMode, same as today's
/// mode_selector.dart submode buttons; "Confirmar" just returns to Inicio.
class ModeScreen extends StatelessWidget {
  final VoidCallback onSettingsTap;
  final ValueChanged<RoboTab> onNavigate;

  const ModeScreen({
    super.key,
    required this.onSettingsTap,
    required this.onNavigate,
  });

  @override
  Widget build(BuildContext context) {
    final connection = context.watch<ConnectionProvider>();
    final currentMode =
        context.watch<RobotStateProvider>().currentStatus.mode;

    return Column(
      children: [
        RoboTopBar(onSettingsTap: onSettingsTap),
        const Padding(
          padding: EdgeInsets.fromLTRB(20, 14, 20, 4),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                'Modo autónomo',
                style: TextStyle(
                  fontWeight: FontWeight.w700,
                  fontSize: 18,
                  color: RoboColors.darkGreen,
                ),
              ),
              SizedBox(height: 2),
              Text(
                'Elegí cómo va a cortar el robot',
                style: TextStyle(fontSize: 13, color: RoboColors.textSecondary),
              ),
            ],
          ),
        ),
        Expanded(
          child: ListView(
            padding: const EdgeInsets.fromLTRB(20, 14, 20, 14),
            children: [
              ModeCard(
                icon: Icons.view_stream,
                iconBg: RoboColors.blueChip,
                iconColor: RoboColors.blueAccent,
                title: 'Líneas paralelas',
                subtitle: 'Corte prolijo en franjas rectas',
                selected: currentMode == 'auto_parallel',
                onTap: () => connection.setMode('auto_parallel'),
              ),
              const SizedBox(height: 14),
              ModeCard(
                icon: Icons.shuffle,
                iconBg: RoboColors.amberChip,
                iconColor: RoboColors.amberAccent,
                title: 'Aleatorio',
                subtitle: 'Recorrido libre, ideal jardines chicos',
                selected: currentMode == 'auto_random',
                onTap: () => connection.setMode('auto_random'),
              ),
              const SizedBox(height: 14),
              ModeCard(
                icon: Icons.border_outer,
                iconBg: RoboColors.selectedSurfaceAlt,
                iconColor: RoboColors.primaryGreen,
                title: 'Perimetral',
                subtitle: 'Bordea el contorno del cerco',
                selected: currentMode == 'auto_perimeter',
                onTap: () => connection.setMode('auto_perimeter'),
              ),
            ],
          ),
        ),
        Padding(
          padding: const EdgeInsets.fromLTRB(20, 0, 20, 14),
          child: SizedBox(
            width: double.infinity,
            child: ElevatedButton(
              onPressed: () => onNavigate(RoboTab.inicio),
              child: const Text('Confirmar modo'),
            ),
          ),
        ),
      ],
    );
  }
}
