import 'package:flutter/material.dart';
import '../theme/robo_theme.dart';

enum RoboTab { inicio, modo, cerco, actuadores }

/// 4-tab bottom nav matching BottomNav.dc.html's custom line-art icons.
class RoboBottomNav extends StatelessWidget {
  final RoboTab active;
  final ValueChanged<RoboTab> onTap;

  const RoboBottomNav({super.key, required this.active, required this.onTap});

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.fromLTRB(12, 10, 12, 16),
      decoration: const BoxDecoration(
        color: Colors.white,
        border: Border(top: BorderSide(color: RoboColors.border)),
        boxShadow: [
          BoxShadow(
            color: Color(0x0F000000),
            blurRadius: 16,
            offset: Offset(0, -4),
          ),
        ],
      ),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceAround,
        children: [
          _NavItem(
            label: 'Inicio',
            active: active == RoboTab.inicio,
            painter: (c) => _HomeIconPainter(c),
            onTap: () => onTap(RoboTab.inicio),
          ),
          _NavItem(
            label: 'Modo',
            active: active == RoboTab.modo,
            painter: (c) => _ModoIconPainter(c),
            onTap: () => onTap(RoboTab.modo),
          ),
          _NavItem(
            label: 'Cerco',
            active: active == RoboTab.cerco,
            painter: (c) => _CercoIconPainter(c),
            onTap: () => onTap(RoboTab.cerco),
          ),
          _NavItem(
            label: 'Actuadores',
            active: active == RoboTab.actuadores,
            painter: (c) => _ActuadoresIconPainter(c),
            onTap: () => onTap(RoboTab.actuadores),
          ),
        ],
      ),
    );
  }
}

class _NavItem extends StatelessWidget {
  final String label;
  final bool active;
  final CustomPainter Function(Color color) painter;
  final VoidCallback onTap;

  const _NavItem({
    required this.label,
    required this.active,
    required this.painter,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    final color = active ? RoboColors.darkGreen : RoboColors.disabled;
    return InkWell(
      onTap: onTap,
      borderRadius: BorderRadius.circular(12),
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 4),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            SizedBox(
              width: 22,
              height: 22,
              child: CustomPaint(painter: painter(color)),
            ),
            const SizedBox(height: 4),
            Text(
              label,
              style: TextStyle(
                fontSize: 10,
                fontWeight: FontWeight.w700,
                color: color,
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class _HomeIconPainter extends CustomPainter {
  final Color color;
  _HomeIconPainter(this.color);

  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = color
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2
      ..strokeJoin = StrokeJoin.round
      ..strokeCap = StrokeCap.round;
    final path = Path()
      ..moveTo(3, 10)
      ..lineTo(11, 3)
      ..lineTo(19, 10)
      ..lineTo(19, 19)
      ..lineTo(13, 19)
      ..lineTo(13, 13)
      ..lineTo(9, 13)
      ..lineTo(9, 19)
      ..lineTo(3, 19)
      ..close();
    canvas.drawPath(path, paint);
  }

  @override
  bool shouldRepaint(covariant _HomeIconPainter oldDelegate) =>
      oldDelegate.color != color;
}

class _ModoIconPainter extends CustomPainter {
  final Color color;
  _ModoIconPainter(this.color);

  @override
  void paint(Canvas canvas, Size size) {
    final stroke = Paint()
      ..color = color
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2;
    final fill = Paint()
      ..color = color
      ..style = PaintingStyle.fill;
    canvas.drawCircle(const Offset(11, 11), 8, stroke);
    canvas.drawCircle(const Offset(11, 11), 2.5, fill);
  }

  @override
  bool shouldRepaint(covariant _ModoIconPainter oldDelegate) =>
      oldDelegate.color != color;
}

class _CercoIconPainter extends CustomPainter {
  final Color color;
  _CercoIconPainter(this.color);

  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = color
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2
      ..strokeJoin = StrokeJoin.round;
    final path = Path()
      ..moveTo(11, 3)
      ..lineTo(19, 9)
      ..lineTo(16, 19)
      ..lineTo(6, 19)
      ..lineTo(3, 9)
      ..close();
    canvas.drawPath(path, paint);
  }

  @override
  bool shouldRepaint(covariant _CercoIconPainter oldDelegate) =>
      oldDelegate.color != color;
}

class _ActuadoresIconPainter extends CustomPainter {
  final Color color;
  _ActuadoresIconPainter(this.color);

  @override
  void paint(Canvas canvas, Size size) {
    final line = Paint()
      ..color = color
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2
      ..strokeCap = StrokeCap.round;
    final fill = Paint()
      ..color = color
      ..style = PaintingStyle.fill;

    canvas.drawLine(const Offset(5, 4), const Offset(5, 18), line);
    canvas.drawCircle(const Offset(5, 8), 2.5, fill);

    canvas.drawLine(const Offset(17, 4), const Offset(17, 18), line);
    canvas.drawCircle(const Offset(17, 14), 2.5, fill);
  }

  @override
  bool shouldRepaint(covariant _ActuadoresIconPainter oldDelegate) =>
      oldDelegate.color != color;
}
