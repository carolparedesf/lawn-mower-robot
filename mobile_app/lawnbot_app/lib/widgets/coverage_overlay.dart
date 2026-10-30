import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:provider/provider.dart';
import '../models/coverage_map.dart';
import '../providers/coverage_map_provider.dart';
import '../theme/robo_theme.dart';

/// Map layer with the robot's coverage grid. Empty until a map with
/// `origin_ll` has been received and rendered.
class CoverageOverlayLayer extends StatelessWidget {
  const CoverageOverlayLayer({super.key});

  @override
  Widget build(BuildContext context) {
    final map = context.watch<CoverageMapProvider>();
    final bounds = map.data?.bounds;
    final image = map.image;
    if (bounds == null || image == null) return const SizedBox.shrink();
    return OverlayImageLayer(
      overlayImages: [
        OverlayImage(
          bounds: bounds,
          imageProvider: image,
          gaplessPlayback: true, // no flicker while a new image decodes
        ),
      ],
    );
  }
}

/// Per-layer switches and the "trend, not exact" caption. Hidden until the
/// robot has sent a map.
class CoverageControls extends StatelessWidget {
  const CoverageControls({super.key});

  @override
  Widget build(BuildContext context) {
    final map = context.watch<CoverageMapProvider>();
    final data = map.data;
    if (data == null) return const SizedBox.shrink();
    final layers = map.layers;

    Widget chip(String label, bool value, CoverageLayers Function(bool) next) {
      return FilterChip(
        label: Text(label, style: const TextStyle(fontSize: 11)),
        selected: value,
        visualDensity: VisualDensity.compact,
        onSelected: (v) => map.setLayers(next(v)),
      );
    }

    return Container(
      key: const Key('coverage_controls'),
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 8),
      decoration: BoxDecoration(
        color: Colors.white.withValues(alpha: 0.92),
        borderRadius: BorderRadius.circular(14),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        mainAxisSize: MainAxisSize.min,
        children: [
          Wrap(
            spacing: 6,
            children: [
              chip('Pasadas', layers.passes, (v) => layers.copyWith(passes: v)),
              chip('Sonar', layers.sonar, (v) => layers.copyWith(sonar: v)),
              chip('Cámara', layers.camera, (v) => layers.copyWith(camera: v)),
            ],
          ),
          const SizedBox(height: 4),
          Text(
            data.canPlace
                ? coverageCaption
                : 'El robot todavía no informa dónde está el origen del mapa: '
                    'no se puede dibujar la cobertura.',
            style: const TextStyle(
              fontSize: 10,
              fontWeight: FontWeight.w600,
              color: RoboColors.textTertiary,
            ),
          ),
        ],
      ),
    );
  }
}

/// Caption under the layer switches.
const String coverageCaption =
    'Cobertura: tendencia, no exacta (el GPS tiene 2-3 m de error).';
