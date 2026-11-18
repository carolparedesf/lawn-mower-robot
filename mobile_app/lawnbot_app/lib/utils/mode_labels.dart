/// Spanish name of a robot/status `mode` for the status bar and mode banner.
const Map<String, String> _modeLabels = {
  'idle': 'Detenido',
  'stop': 'Detenido',
  'manual': 'Manual',
  'auto_random': 'Aleatorio',
  'auto_parallel': 'Líneas paralelas',
  'auto_perimeter': 'Perimetral',
};

/// "Detenido", "Manual", "Aleatorio", "Líneas paralelas" or "Perimetral"; an
/// unknown mode is shown as received so nothing is silently hidden.
String modeLabel(String mode) => _modeLabels[mode] ?? mode;
