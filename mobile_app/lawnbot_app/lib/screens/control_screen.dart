import 'dart:math' as math;
import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:provider/provider.dart';
import '../providers/connection_provider.dart';
import '../providers/robot_state_provider.dart';
import '../theme/robo_theme.dart';
import '../widgets/dpad_widget.dart';
import '../widgets/joystick_widget.dart';
import '../widgets/robo_bottom_nav.dart';
import '../widgets/robo_top_bar.dart';

const LatLng _defaultCenter = LatLng(-25.2867, -57.6470);

const Map<String, String> _modeLabels = {
  'manual': 'Manual',
  'auto_parallel': 'Líneas paralelas',
  'auto_random': 'Aleatorio',
  'auto_perimeter': 'Perimetral',
  'stop': 'Detenido',
  'idle': 'Inactivo',
};

/// "Inicio" tab. Real GPS map (tiles, robot marker+heading, polyline trail,
/// follow-toggle) replaces the mockup's fake SVG yard box; everything else
/// (joystick, mode banner, autonomous CTA) ports the mockup's layout.
class ControlScreen extends StatefulWidget {
  final VoidCallback onSettingsTap;
  final ValueChanged<RoboTab> onNavigate;

  const ControlScreen({
    super.key,
    required this.onSettingsTap,
    required this.onNavigate,
  });

  @override
  State<ControlScreen> createState() => _ControlScreenState();
}

class _ControlScreenState extends State<ControlScreen> {
  final MapController _mapController = MapController();
  bool _followRobot = true;
  bool _useDpad = false;

  void _selectStick(bool useDpad) {
    setState(() => _useDpad = useDpad);
    // Entering manual control must explicitly request manual mode: the
    // robot only honors robot/cmd/move while mode == "manual".
    context.read<ConnectionProvider>().setMode('manual');
  }

  void _toggleAutonomous(bool isAutonomous) {
    final connection = context.read<ConnectionProvider>();
    if (isAutonomous) {
      connection.setMode('stop');
    } else {
      connection.setMode('auto_parallel');
    }
  }

  @override
  Widget build(BuildContext context) {
    final robotState = context.watch<RobotStateProvider>();
    final status = robotState.currentStatus;
    final robotPos = robotState.currentPosition;
    final center = robotPos != null
        ? LatLng(robotPos.lat, robotPos.lng)
        : _defaultCenter;
    final isAutonomous = status.isAutonomous;

    if (_followRobot && robotPos != null) {
      WidgetsBinding.instance.addPostFrameCallback((_) {
        _mapController.move(center, _mapController.camera.zoom);
      });
    }

    return Column(
      children: [
        RoboTopBar(onSettingsTap: widget.onSettingsTap),
        Expanded(
          child: SingleChildScrollView(
            padding: const EdgeInsets.fromLTRB(20, 10, 20, 10),
            child: Column(
              children: [
                ClipRRect(
                  borderRadius: BorderRadius.circular(24),
                  child: SizedBox(
                    height: 260,
                    child: Stack(
                      children: [
                        FlutterMap(
                          mapController: _mapController,
                          options: MapOptions(
                            initialCenter: center,
                            initialZoom: 18,
                            onPositionChanged: (_, hasGesture) {
                              if (hasGesture && _followRobot) {
                                setState(() => _followRobot = false);
                              }
                            },
                          ),
                          children: [
                            TileLayer(
                              urlTemplate:
                                  'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
                              userAgentPackageName:
                                  'com.example.lawnbot_app',
                            ),
                            if (robotState.positionHistory.length > 1)
                              PolylineLayer(
                                polylines: [
                                  Polyline(
                                    points: robotState.positionHistory,
                                    color: RoboColors.blueAccent
                                        .withValues(alpha: 0.6),
                                    strokeWidth: 3,
                                  ),
                                ],
                              ),
                            if (robotPos != null)
                              MarkerLayer(
                                markers: [
                                  Marker(
                                    point: LatLng(robotPos.lat, robotPos.lng),
                                    width: 36,
                                    height: 36,
                                    child: Transform.rotate(
                                      angle: robotPos.heading * math.pi / 180,
                                      child: Container(
                                        decoration: const BoxDecoration(
                                          color: RoboColors.amberAccent,
                                          shape: BoxShape.circle,
                                          border: Border.fromBorderSide(
                                            BorderSide(
                                                color: Colors.white,
                                                width: 3),
                                          ),
                                        ),
                                        child: const Icon(Icons.navigation,
                                            color: Colors.white, size: 18),
                                      ),
                                    ),
                                  ),
                                ],
                              ),
                          ],
                        ),
                        Positioned(
                          bottom: 12,
                          right: 12,
                          child: FloatingActionButton.small(
                            heroTag: 'follow-robot',
                            backgroundColor: _followRobot
                                ? RoboColors.primaryGreen
                                : Colors.white,
                            foregroundColor: _followRobot
                                ? Colors.white
                                : RoboColors.primaryGreen,
                            onPressed: () =>
                                setState(() => _followRobot = !_followRobot),
                            child: Icon(_followRobot
                                ? Icons.gps_fixed
                                : Icons.gps_not_fixed),
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
                const SizedBox(height: 12),
                Container(
                  padding: const EdgeInsets.symmetric(
                      horizontal: 14, vertical: 10),
                  decoration: BoxDecoration(
                    color: Colors.white,
                    borderRadius: BorderRadius.circular(16),
                    boxShadow: const [
                      BoxShadow(
                        color: Color(0x0D000000),
                        blurRadius: 8,
                        offset: Offset(0, 2),
                      ),
                    ],
                  ),
                  child: Row(
                    mainAxisAlignment: MainAxisAlignment.spaceBetween,
                    children: [
                      Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          const Text(
                            'MODO SELECCIONADO',
                            style: TextStyle(
                              fontSize: 10,
                              fontWeight: FontWeight.w700,
                              color: RoboColors.textSecondary,
                              letterSpacing: 0.4,
                            ),
                          ),
                          Text(
                            _modeLabels[status.mode] ?? status.mode,
                            style: const TextStyle(
                              fontSize: 14,
                              fontWeight: FontWeight.w800,
                              color: RoboColors.darkGreen,
                            ),
                          ),
                        ],
                      ),
                      TextButton(
                        onPressed: () => widget.onNavigate(RoboTab.modo),
                        style: TextButton.styleFrom(
                          backgroundColor: RoboColors.blueChip,
                          foregroundColor: RoboColors.blueAccent,
                          padding: const EdgeInsets.symmetric(
                              horizontal: 14, vertical: 8),
                          shape: RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(14),
                          ),
                        ),
                        child: const Text('Cambiar',
                            style: TextStyle(
                                fontSize: 12, fontWeight: FontWeight.w700)),
                      ),
                    ],
                  ),
                ),
                const SizedBox(height: 14),
                Container(
                  padding: const EdgeInsets.all(4),
                  decoration: BoxDecoration(
                    color: Colors.white,
                    borderRadius: BorderRadius.circular(16),
                    boxShadow: const [
                      BoxShadow(
                        color: Color(0x0D000000),
                        blurRadius: 8,
                        offset: Offset(0, 2),
                      ),
                    ],
                  ),
                  child: Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      _SegButton(
                        label: 'Joystick',
                        active: !_useDpad,
                        onTap: () => _selectStick(false),
                      ),
                      _SegButton(
                        label: 'Flechas',
                        active: _useDpad,
                        onTap: () => _selectStick(true),
                      ),
                    ],
                  ),
                ),
                const SizedBox(height: 14),
                SizedBox(
                  height: 138,
                  child: Center(
                    child: _useDpad
                        ? const DpadWidget()
                        : const JoystickWidget(),
                  ),
                ),
                const SizedBox(height: 14),
                SizedBox(
                  width: double.infinity,
                  child: ElevatedButton(
                    onPressed: () => _toggleAutonomous(isAutonomous),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: isAutonomous
                          ? RoboColors.error
                          : RoboColors.amberAccent,
                      shadowColor: (isAutonomous
                              ? RoboColors.error
                              : RoboColors.amberAccent)
                          .withValues(alpha: 0.4),
                    ),
                    child: Text(isAutonomous
                        ? '⏸ Detener modo autónomo'
                        : '▶ Iniciar modo autónomo'),
                  ),
                ),
              ],
            ),
          ),
        ),
      ],
    );
  }
}

class _SegButton extends StatelessWidget {
  final String label;
  final bool active;
  final VoidCallback onTap;

  const _SegButton({
    required this.label,
    required this.active,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    return InkWell(
      onTap: onTap,
      borderRadius: BorderRadius.circular(12),
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
        decoration: BoxDecoration(
          color: active ? RoboColors.primaryGreen : Colors.transparent,
          borderRadius: BorderRadius.circular(12),
        ),
        child: Text(
          label,
          style: TextStyle(
            fontSize: 12,
            fontWeight: FontWeight.w700,
            color: active ? Colors.white : RoboColors.textSecondary,
          ),
        ),
      ),
    );
  }
}
