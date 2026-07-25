import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'providers/actuator_provider.dart';
import 'providers/connection_provider.dart';
import 'providers/geofence_provider.dart';
import 'providers/robot_state_provider.dart';
import 'screens/home_screen.dart';
import 'theme/robo_theme.dart';

void main() {
  runApp(const RobotControlApp());
}

class RobotControlApp extends StatelessWidget {
  const RobotControlApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MultiProvider(
      providers: [
        ChangeNotifierProvider(create: (_) => ConnectionProvider()),
        ChangeNotifierProvider(create: (_) => RobotStateProvider()),
        ChangeNotifierProvider(create: (_) => GeofenceProvider()),
        ChangeNotifierProvider(create: (_) => ActuatorProvider()),
      ],
      child: MaterialApp(
        title: 'RobotControl',
        debugShowCheckedModeBanner: false,
        theme: roboTheme(),
        home: const HomeScreen(),
      ),
    );
  }
}
