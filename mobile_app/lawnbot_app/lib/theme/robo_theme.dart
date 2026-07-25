import 'package:flutter/material.dart';

/// Palette extracted from the "Cortacésped App" design mockup.
class RoboColors {
  RoboColors._();

  static const Color background = Color(0xFFFFF8E1);
  static const Color primaryGreen = Color(0xFF4CAF50);
  static const Color darkGreen = Color(0xFF2E7D32);
  static const Color selectedSurface = Color(0xFFF1F8F1);
  static const Color selectedSurfaceAlt = Color(0xFFE8F5E9);
  static const Color blueAccent = Color(0xFF42A5F5);
  static const Color blueChip = Color(0xFFE3F2FD);
  static const Color amberAccent = Color(0xFFFFA726);
  static const Color amberChip = Color(0xFFFFF3E0);
  static const Color error = Color(0xFFEF5350);
  static const Color errorChip = Color(0xFFFDEAEA);
  static const Color textSecondary = Color(0xFF8A9A8C);
  static const Color textTertiary = Color(0xFF5D6B5E);
  static const Color border = Color(0xFFF1F1E8);
  static const Color borderStrong = Color(0xFFE0E0D5);
  static const Color disabled = Color(0xFFB9C9BA);
}

ThemeData roboTheme() {
  const colorScheme = ColorScheme.light(
    primary: RoboColors.primaryGreen,
    onPrimary: Colors.white,
    secondary: RoboColors.blueAccent,
    onSecondary: Colors.white,
    tertiary: RoboColors.amberAccent,
    onTertiary: Colors.white,
    error: RoboColors.error,
    onError: Colors.white,
    surface: Colors.white,
    onSurface: RoboColors.darkGreen,
  );

  return ThemeData(
    useMaterial3: true,
    colorScheme: colorScheme,
    scaffoldBackgroundColor: RoboColors.background,
    fontFamily: 'Roboto',
    textTheme: const TextTheme(
      titleLarge: TextStyle(
        fontWeight: FontWeight.w700,
        fontSize: 18,
        color: RoboColors.darkGreen,
      ),
      titleMedium: TextStyle(
        fontWeight: FontWeight.w700,
        fontSize: 15,
        color: RoboColors.darkGreen,
      ),
      bodyMedium: TextStyle(fontSize: 13, color: RoboColors.textTertiary),
      labelLarge: TextStyle(
        fontWeight: FontWeight.w700,
        fontSize: 12,
        color: RoboColors.textSecondary,
      ),
    ),
    cardTheme: CardThemeData(
      color: Colors.white,
      elevation: 0,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(20)),
      margin: EdgeInsets.zero,
    ),
    elevatedButtonTheme: ElevatedButtonThemeData(
      style: ElevatedButton.styleFrom(
        backgroundColor: RoboColors.primaryGreen,
        foregroundColor: Colors.white,
        elevation: 4,
        shadowColor: RoboColors.primaryGreen.withValues(alpha: 0.4),
        padding: const EdgeInsets.symmetric(vertical: 16, horizontal: 20),
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(24),
        ),
        textStyle: const TextStyle(fontWeight: FontWeight.w700, fontSize: 16),
      ),
    ),
    switchTheme: SwitchThemeData(
      thumbColor: WidgetStateProperty.resolveWith(
        (states) => Colors.white,
      ),
      trackColor: WidgetStateProperty.resolveWith((states) {
        if (states.contains(WidgetState.selected)) {
          return RoboColors.primaryGreen;
        }
        return RoboColors.borderStrong;
      }),
    ),
    dividerTheme: const DividerThemeData(color: RoboColors.border, space: 1),
  );
}
