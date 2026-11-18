import 'package:flutter/material.dart';

/// Palette of the first (pink) version of the app, sampled pixel by pixel from
/// docs/ref/app_rosa.png. Semantic colors (success / warning / error) keep the
/// usual green / orange / red of that version's connection chip.
class RoboColors {
  RoboColors._();

  // Sampled from docs/ref/app_rosa.png
  static const Color primary = Color(0xFFE91D8C); // buttons, selected nav, switches
  static const Color appBar = Color(0xFFE42189); // top bar (and status bar area)
  static const Color background = Color(0xFFFEF8F8); // page background
  static const Color statusBar = Color(0xFFFBE4EC); // strip under the top bar / tints
  static const Color borderStrong = Color(0xFFD3C3C6); // strip divider
  static const Color text = Color(0xFF22191C); // main text
  static const Color textSecondary = Color(0xFF504448); // chip text, secondary text
  static const Color muted = Color(0xFF9E9E9E); // unselected nav, unknown battery
  static const Color success = Color(0xFF4CAF50); // connected dot
  static const Color error = Color(0xFFF44336); // low battery, errors

  // Derived (not sampled): tints and secondary shades of the colors above
  static const Color textMuted = Color(0xFF7A7073); // captions on the pink background
  static const Color border = Color(0xFFEBDFE2); // card / input outlines
  static const Color disabled = Color(0xFFCFC6C8);
  static const Color warning = Color(0xFFFF9800); // "connecting", medium battery
  static const Color warningChip = Color(0xFFFFF3E0);
  static const Color errorChip = Color(0xFFFDEAEA);
  static const Color successDark = Color(0xFF2E7D32);
  static const Color successChip = Color(0xFFE8F5E9);
}

ThemeData roboTheme() {
  const colorScheme = ColorScheme.light(
    primary: RoboColors.primary,
    onPrimary: Colors.white,
    primaryContainer: RoboColors.statusBar,
    onPrimaryContainer: RoboColors.text,
    secondary: RoboColors.primary,
    onSecondary: Colors.white,
    error: RoboColors.error,
    onError: Colors.white,
    surface: RoboColors.background,
    onSurface: RoboColors.text,
    onSurfaceVariant: RoboColors.textSecondary,
    outline: RoboColors.borderStrong,
    outlineVariant: RoboColors.border,
  );

  return ThemeData(
    useMaterial3: true,
    colorScheme: colorScheme,
    scaffoldBackgroundColor: RoboColors.background,
    fontFamily: 'Roboto',
    textTheme: ThemeData(useMaterial3: true).textTheme.apply(
          fontFamily: 'Roboto',
          bodyColor: RoboColors.text,
          displayColor: RoboColors.text,
        ),
    appBarTheme: const AppBarTheme(
      backgroundColor: RoboColors.appBar,
      foregroundColor: Colors.white,
    ),
    cardTheme: CardThemeData(
      color: Colors.white,
      elevation: 0,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(20)),
      margin: EdgeInsets.zero,
    ),
    // Pill buttons, filled pink with white text and a soft shadow (as in the first version)
    elevatedButtonTheme: ElevatedButtonThemeData(
      style: ElevatedButton.styleFrom(
        backgroundColor: RoboColors.primary,
        foregroundColor: Colors.white,
        disabledBackgroundColor: RoboColors.disabled,
        disabledForegroundColor: Colors.white,
        elevation: 3,
        shape: const StadiumBorder(),
        padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 12),
        textStyle: const TextStyle(
            fontFamily: 'Roboto', fontWeight: FontWeight.w500, fontSize: 16),
      ),
    ),
    filledButtonTheme: FilledButtonThemeData(
      style: FilledButton.styleFrom(
        backgroundColor: RoboColors.primary,
        foregroundColor: Colors.white,
        shape: const StadiumBorder(),
      ),
    ),
    textButtonTheme: TextButtonThemeData(
      style: TextButton.styleFrom(foregroundColor: RoboColors.primary),
    ),
    floatingActionButtonTheme: const FloatingActionButtonThemeData(
      backgroundColor: RoboColors.primary,
      foregroundColor: Colors.white,
    ),
    bottomNavigationBarTheme: const BottomNavigationBarThemeData(
      backgroundColor: RoboColors.background,
      selectedItemColor: RoboColors.primary,
      unselectedItemColor: RoboColors.muted,
      type: BottomNavigationBarType.fixed,
      elevation: 8,
    ),
    switchTheme: SwitchThemeData(
      thumbColor: WidgetStateProperty.resolveWith((states) =>
          states.contains(WidgetState.selected) ? Colors.white : RoboColors.muted),
      trackColor: WidgetStateProperty.resolveWith((states) =>
          states.contains(WidgetState.selected)
              ? RoboColors.primary
              : RoboColors.borderStrong),
    ),
    dialogTheme: DialogThemeData(
      backgroundColor: RoboColors.background,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(24)),
    ),
    inputDecorationTheme: const InputDecorationTheme(
      focusedBorder: UnderlineInputBorder(
        borderSide: BorderSide(color: RoboColors.primary, width: 2),
      ),
      floatingLabelStyle: TextStyle(color: RoboColors.primary),
    ),
    dividerTheme: const DividerThemeData(color: RoboColors.border, space: 1),
  );
}
