import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:provider/provider.dart';
import '../providers/connection_provider.dart';
import '../providers/geofence_provider.dart';
import '../providers/robot_state_provider.dart';
import '../theme/robo_theme.dart';
import '../models/geofence.dart';
import '../widgets/coverage_overlay.dart';
import '../widgets/geofence_notices.dart';
import '../widgets/geofence_toolbar.dart';
import '../widgets/robot_marker.dart';
import '../widgets/robo_top_bar.dart';

const LatLng _defaultCenter = LatLng(-25.2867, -57.6470);

/// "Cerco" tab — dedicated geofence definition screen. Tap the map to add
/// points; unchanged GeofenceProvider add/undo/clear/send logic underneath.
class GeofenceScreen extends StatefulWidget {
  final VoidCallback onSettingsTap;

  const GeofenceScreen({super.key, required this.onSettingsTap});

  @override
  State<GeofenceScreen> createState() => _GeofenceScreenState();
}

class _GeofenceScreenState extends State<GeofenceScreen> {
  final MapController _mapController = MapController();
  bool _centeredOnRobot = false;

  @override
  Widget build(BuildContext context) {
    final geofence = context.watch<GeofenceProvider>();
    final robotState = context.watch<RobotStateProvider>();
    final robotPos = robotState.currentPosition;
    final isConnected = context.watch<ConnectionProvider>().isConnected;
    final center = robotPos != null
        ? LatLng(robotPos.lat, robotPos.lng)
        : _defaultCenter;
    final geofenceBreach = robotState.currentStatus.hasGeofenceBreach;
    // Invalid shapes (not just "too few points") are drawn with the error color.
    final problem = geofence.problem;
    final badShape = problem == GeofenceProblem.selfIntersecting ||
        problem == GeofenceProblem.notConvex;

    // La posición del robot suele llegar después del primer build (viene
    // por GPS/MQTT) — initialCenter del mapa ya quedó fijo en ese momento,
    // así que la primera vez que tenemos una posición real hay que mover
    // la cámara a mano, una sola vez (no interrumpe el paneo del usuario
    // mientras marca los puntos del cerco después de eso).
    if (robotPos != null && !_centeredOnRobot) {
      _centeredOnRobot = true;
      WidgetsBinding.instance.addPostFrameCallback((_) {
        _mapController.move(center, 18);
      });
    }

    return Column(
      children: [
        RoboTopBar(onSettingsTap: widget.onSettingsTap),
        const Padding(
          padding: EdgeInsets.fromLTRB(20, 14, 20, 4),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                'Cerco virtual',
                style: TextStyle(
                  fontWeight: FontWeight.w700,
                  fontSize: 18,
                  color: RoboColors.darkGreen,
                ),
              ),
              SizedBox(height: 2),
              Text(
                'Marcá el área donde puede cortar el robot',
                style: TextStyle(fontSize: 13, color: RoboColors.textSecondary),
              ),
            ],
          ),
        ),
        Expanded(
          child: Padding(
            padding: const EdgeInsets.fromLTRB(20, 10, 20, 20),
            child: ClipRRect(
              borderRadius: BorderRadius.circular(24),
              child: Stack(
                children: [
                  FlutterMap(
                    mapController: _mapController,
                    options: MapOptions(
                      initialCenter: center,
                      initialZoom: 18,
                      onTap: (_, latlng) => geofence.addPoint(latlng),
                    ),
                    children: [
                      TileLayer(
                        urlTemplate:
                            'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
                        userAgentPackageName:
                            'com.example.lawnbot_app',
                      ),
                      const CoverageOverlayLayer(),
                      if (geofence.points.length >= 2)
                        PolygonLayer(
                          polygons: [
                            Polygon(
                              points: geofence.points,
                              color: RoboColors.primaryGreen
                                  .withValues(alpha: 0.35),
                              borderColor: (geofenceBreach || badShape)
                                  ? RoboColors.error
                                  : RoboColors.darkGreen,
                              borderStrokeWidth: geofenceBreach ? 3 : 3,
                            ),
                          ],
                        ),
                      if (robotPos != null)
                        MarkerLayer(
                          key: const Key('robot_marker_layer'),
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
                      MarkerLayer(
                        markers: [
                          for (var i = 0; i < geofence.points.length; i++)
                            Marker(
                              point: geofence.points[i],
                              width: 26,
                              height: 26,
                              child: Container(
                                decoration: const BoxDecoration(
                                  color: RoboColors.amberAccent,
                                  shape: BoxShape.circle,
                                  border:
                                      Border.fromBorderSide(BorderSide(
                                    color: Colors.white,
                                    width: 2,
                                  )),
                                ),
                                child: Center(
                                  child: Text(
                                    '${i + 1}',
                                    style: const TextStyle(
                                      color: RoboColors.darkGreen,
                                      fontWeight: FontWeight.w700,
                                      fontSize: 10,
                                    ),
                                  ),
                                ),
                              ),
                            ),
                        ],
                      ),
                    ],
                  ),
                  Positioned(
                    top: 10,
                    left: 10,
                    right: 10,
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Container(
                          padding: const EdgeInsets.symmetric(
                              horizontal: 10, vertical: 6),
                          decoration: BoxDecoration(
                            color: Colors.white.withValues(alpha: 0.92),
                            borderRadius: BorderRadius.circular(14),
                          ),
                          child: const Text(
                            'Tocá el mapa para agregar puntos',
                            style: TextStyle(
                              fontSize: 11,
                              fontWeight: FontWeight.w700,
                              color: RoboColors.darkGreen,
                            ),
                          ),
                        ),
                        const SizedBox(height: 6),
                        const GeofenceProblemBanner(),
                        const SizedBox(height: 6),
                        const GeofenceRobotStatus(),
                        const SizedBox(height: 6),
                        const CoverageControls(),
                      ],
                    ),
                  ),
                  const Positioned(
                    bottom: 12,
                    left: 0,
                    right: 0,
                    child: Center(child: GeofenceToolbar()),
                  ),
                  Positioned(
                    right: 12,
                    bottom: 76,
                    child: FloatingActionButton.small(
                      key: const Key('center_on_robot'),
                      heroTag: 'center_on_robot',
                      tooltip: 'Centrar en el robot',
                      backgroundColor: Colors.white,
                      foregroundColor: RoboColors.darkGreen,
                      onPressed: robotPos == null
                          ? () => ScaffoldMessenger.of(context).showSnackBar(
                                const SnackBar(
                                  content: Text(
                                      'Todavía no hay posición del robot'),
                                  duration: Duration(seconds: 2),
                                ),
                              )
                          : () => _mapController.move(center, 19),
                      child: const Icon(Icons.my_location),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      ],
    );
  }
}
