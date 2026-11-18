import 'package:flutter/material.dart';
import '../theme/robo_theme.dart';

enum RoboTab { inicio, modo, cerco, actuadores }

/// Bottom navigation of the first (pink) version: the plain Material bar with a
/// pink selected item and grey unselected ones, now with the four screens.
class RoboBottomNav extends StatelessWidget {
  final RoboTab active;
  final ValueChanged<RoboTab> onTap;

  const RoboBottomNav({super.key, required this.active, required this.onTap});

  static const _items = [
    (RoboTab.inicio, 'Control', Icons.gamepad),
    (RoboTab.modo, 'Modo', Icons.tune),
    (RoboTab.cerco, 'Cerco', Icons.map),
    (RoboTab.actuadores, 'Actuadores', Icons.power),
  ];

  @override
  Widget build(BuildContext context) {
    return BottomNavigationBar(
      key: const Key('bottom_nav'),
      currentIndex: RoboTab.values.indexOf(active),
      onTap: (i) => onTap(RoboTab.values[i]),
      backgroundColor: RoboColors.background,
      selectedItemColor: RoboColors.primary,
      unselectedItemColor: RoboColors.muted,
      type: BottomNavigationBarType.fixed,
      selectedFontSize: 14,
      unselectedFontSize: 14,
      items: [
        for (final (_, label, icon) in _items)
          BottomNavigationBarItem(icon: Icon(icon), label: label),
      ],
    );
  }
}
