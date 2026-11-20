import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:provider/provider.dart';
import '../providers/connection_provider.dart';
import '../providers/robot_state_provider.dart';
import '../theme/robo_theme.dart';
import '../utils/mode_labels.dart';
import '../widgets/estop_release_button.dart';
import '../widgets/joystick_widget.dart';
import '../widgets/mode_buttons.dart';
import '../widgets/robo_bottom_nav.dart';
import '../widgets/robo_top_bar.dart';
import '../widgets/robot_marker.dart';
import '../widgets/speed_limit_selector.dart';

const LatLng _defaultCenter = LatLng(-25.2867, -57.6470);

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

  @override
  Widget build(BuildContext context) {
    final robotState = context.watch<RobotStateProvider>();
    final status = robotState.currentStatus;
    final robotPos = robotState.currentPosition;
    final center = robotPos != null
        ? LatLng(robotPos.lat, robotPos.lng)
        : _defaultCenter;
    final manual = isManualMode(status.mode);
    final isConnected = context.watch<ConnectionProvider>().isConnected;

    if (_followRobot && robotPos != null) {
      WidgetsBinding.instance.addPostFrameCallback((_) {
        _mapController.move(center, _mapController.camera.zoom);
      });
    }

    return Column(
      children: [
        RoboTopBar(onSettingsTap: widget.onSettingsTap),
        Expanded(
          child: Padding(
            padding: const EdgeInsets.fromLTRB(20, 10, 20, 0),
            child: Column(
              children: [
                Expanded(
                  child: ClipRRect(
                    borderRadius: BorderRadius.circular(24),
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
                              userAgentPackageName: 'com.example.lawnbot_app',
                            ),
                            if (robotState.positionHistory.length > 1)
                              PolylineLayer(
                                polylines: [
                                  Polyline(
                                    points: robotState.positionHistory,
                                    color: RoboColors.primary.withValues(
                                      alpha: 0.6,
                                    ),
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
                                    child: RobotMarker(
                                      isConnected: isConnected,
                                      headingDegrees: robotPos.heading,
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
                                ? RoboColors.primary
                                : Colors.white,
                            foregroundColor: _followRobot
                                ? Colors.white
                                : RoboColors.primary,
                            onPressed: () =>
                                setState(() => _followRobot = !_followRobot),
                            child: Icon(
                              _followRobot
                                  ? Icons.gps_fixed
                                  : Icons.gps_not_fixed,
                            ),
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
                const SizedBox(height: 10),
                Container(
                  padding: const EdgeInsets.symmetric(
                    horizontal: 14,
                    vertical: 10,
                  ),
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
                      Expanded(
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            const Text(
                              'MODO SELECCIONADO',
                              style: TextStyle(
                                fontSize: 10,
                                fontWeight: FontWeight.w700,
                                color: RoboColors.textMuted,
                                letterSpacing: 0.4,
                              ),
                            ),
                            Text(
                              modeLabel(status.mode),
                              maxLines: 1,
                              overflow: TextOverflow.ellipsis,
                              style: const TextStyle(
                                fontSize: 14,
                                fontWeight: FontWeight.w800,
                                color: RoboColors.text,
                              ),
                            ),
                          ],
                        ),
                      ),
                      TextButton(
                        onPressed: () => widget.onNavigate(RoboTab.modo),
                        style: TextButton.styleFrom(
                          backgroundColor: RoboColors.statusBar,
                          foregroundColor: RoboColors.primary,
                          padding: const EdgeInsets.symmetric(
                            horizontal: 14,
                            vertical: 8,
                          ),
                          shape: RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(14),
                          ),
                        ),
                        child: const Text(
                          'Cambiar',
                          style: TextStyle(
                            fontSize: 12,
                            fontWeight: FontWeight.w700,
                          ),
                        ),
                      ),
                    ],
                  ),
                ),
                const SizedBox(height: 8),
                const SpeedLimitSelector(),
                const SizedBox(height: 6),
                SizedBox(
                  height: 156,
                  child: Stack(
                    alignment: Alignment.center,
                    children: [
                      Center(child: JoystickWidget(modeEnabled: manual)),
                      if (!manual)
                        const Text(
                          'Elegí Manual para mover el robot',
                          key: Key('joystick_hint'),
                          textAlign: TextAlign.center,
                          style: TextStyle(
                            fontWeight: FontWeight.w700,
                            color: RoboColors.textSecondary,
                          ),
                        ),
                    ],
                  ),
                ),
              ],
            ),
          ),
        ),
        // Fixed outside any scroll view so it's always reachable without
        // scrolling, regardless of how tall the content above ends up being.
        const Padding(
          padding: EdgeInsets.fromLTRB(20, 10, 20, 14),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              EstopReleaseButton(),
              ModeButtons(),
            ],
          ),
        ),
      ],
    );
  }
}
