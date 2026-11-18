import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/connection_provider.dart';
import '../widgets/robo_bottom_nav.dart';
import '../widgets/stop_button.dart';
import 'actuators_screen.dart';
import 'control_screen.dart';
import 'geofence_screen.dart';
import 'mode_screen.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  RoboTab _tab = RoboTab.inicio;

  void _onNavigate(RoboTab tab) => setState(() => _tab = tab);

  @override
  Widget build(BuildContext context) {
    late final Widget body;
    switch (_tab) {
      case RoboTab.inicio:
        body = ControlScreen(
          onSettingsTap: () => _showConnectionDialog(context),
          onNavigate: _onNavigate,
        );
        break;
      case RoboTab.modo:
        body = ModeScreen(
          onSettingsTap: () => _showConnectionDialog(context),
          onNavigate: _onNavigate,
        );
        break;
      case RoboTab.cerco:
        body = GeofenceScreen(
          onSettingsTap: () => _showConnectionDialog(context),
        );
        break;
      case RoboTab.actuadores:
        body = ActuatorsScreen(
          onSettingsTap: () => _showConnectionDialog(context),
        );
        break;
    }

    return Scaffold(
      // The top bar paints the status-bar area itself (pink, edge to edge)
      body: SafeArea(top: false, child: body),
      // STOP is on every screen, right above the navigation
      bottomNavigationBar: SafeArea(
        top: false,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            const StopButton(),
            RoboBottomNav(active: _tab, onTap: _onNavigate),
          ],
        ),
      ),
    );
  }

  void _showConnectionDialog(BuildContext context) {
    final provider = context.read<ConnectionProvider>();
    final controller = TextEditingController(text: provider.broker);

    showDialog(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('Conexión MQTT'),
        content: TextField(
          controller: controller,
          decoration: const InputDecoration(
            labelText: 'IP del broker (Raspberry Pi)',
            hintText: '192.168.1.100',
          ),
          keyboardType: TextInputType.url,
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx),
            child: const Text('Cancelar'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.pop(ctx);
              provider.connect(broker: controller.text.trim());
            },
            child: const Text('Conectar'),
          ),
        ],
      ),
    );
  }
}
