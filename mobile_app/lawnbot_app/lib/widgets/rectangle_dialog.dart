import 'package:flutter/material.dart';
import 'package:latlong2/latlong.dart';
import '../theme/robo_theme.dart';
import '../utils/geo.dart';

/// Parses a positive number typed by the user ("12,5" and "12.5" both work).
double? parseMeasure(String text) {
  final v = double.tryParse(text.trim().replaceAll(',', '.'));
  return (v == null || !v.isFinite) ? null : v;
}

/// Asks width / length / bearing for a rectangle whose corner is [corner] and
/// shows the four computed vertices live. Returns them when the user accepts.
Future<List<LatLng>?> showRectangleDialog(BuildContext context, LatLng corner) {
  return showDialog<List<LatLng>>(
    context: context,
    builder: (_) => RectangleDialog(corner: corner),
  );
}

class RectangleDialog extends StatefulWidget {
  final LatLng corner;
  const RectangleDialog({super.key, required this.corner});

  @override
  State<RectangleDialog> createState() => _RectangleDialogState();
}

class _RectangleDialogState extends State<RectangleDialog> {
  static const double maxSide = 1000;
  final _width = TextEditingController();
  final _length = TextEditingController();
  final _bearing = TextEditingController(text: '0');

  @override
  void dispose() {
    _width.dispose();
    _length.dispose();
    _bearing.dispose();
    super.dispose();
  }

  List<LatLng>? get _vertices {
    final w = parseMeasure(_width.text);
    final l = parseMeasure(_length.text);
    final b = parseMeasure(_bearing.text);
    if (w == null || l == null || b == null) return null;
    if (w <= 0 || l <= 0 || w > maxSide || l > maxSide) return null;
    return rectangleFromCorner(widget.corner, w, l, b % 360);
  }

  Widget _field(String key, String label, TextEditingController c) => Padding(
        padding: const EdgeInsets.only(bottom: 8),
        child: TextField(
          key: Key(key),
          controller: c,
          keyboardType: const TextInputType.numberWithOptions(decimal: true),
          decoration: InputDecoration(labelText: label, isDense: true),
          onChanged: (_) => setState(() {}),
        ),
      );

  @override
  Widget build(BuildContext context) {
    final vertices = _vertices;
    return AlertDialog(
      title: const Text('Rectángulo por medidas'),
      content: SingleChildScrollView(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'La esquina es el punto que tocaste. El largo sigue la '
              'orientación y el ancho sale hacia la derecha.',
              style: TextStyle(fontSize: 12, color: RoboColors.textSecondary),
            ),
            const SizedBox(height: 10),
            _field('rect_width', 'Ancho (m)', _width),
            _field('rect_length', 'Largo (m)', _length),
            _field('rect_bearing', 'Orientación (° desde el norte)', _bearing),
            if (vertices == null)
              Text(
                'Ingresá ancho y largo mayores que 0 (máx. ${maxSide.toInt()} m).',
                key: const Key('rect_invalid'),
                style: const TextStyle(fontSize: 12, color: RoboColors.error),
              )
            else ...[
              const Text('Vértices:',
                  style: TextStyle(fontWeight: FontWeight.w700, fontSize: 12)),
              for (var i = 0; i < vertices.length; i++)
                Text(
                  '${i + 1}: ${vertices[i].latitude.toStringAsFixed(6)}, '
                  '${vertices[i].longitude.toStringAsFixed(6)}',
                  key: Key('rect_vertex_$i'),
                  style: const TextStyle(fontSize: 12),
                ),
            ],
          ],
        ),
      ),
      actions: [
        TextButton(
          onPressed: () => Navigator.pop(context),
          child: const Text('Cancelar'),
        ),
        FilledButton(
          key: const Key('rect_accept'),
          onPressed: vertices == null ? null : () => Navigator.pop(context, vertices),
          child: const Text('Usar en el mapa'),
        ),
      ],
    );
  }
}
