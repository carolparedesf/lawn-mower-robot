import 'package:flutter/material.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:provider/provider.dart';
import 'package:robotmapapp/providers/connection_provider.dart';
import 'package:robotmapapp/widgets/joystick_widget.dart';

void main() {
  testWidgets('joystick keeps the pre-redesign look (package defaults)',
      (tester) async {
    await tester.pumpWidget(ChangeNotifierProvider(
      create: (_) => ConnectionProvider(),
      child: const MaterialApp(home: Scaffold(body: Center(child: JoystickWidget()))),
    ));

    // flutter_joystick schedules its initial animation (delayed calls); let them run.
    await tester.pump(const Duration(seconds: 10));

    final joystick = tester.widget<Joystick>(find.byType(Joystick));
    expect(joystick.mode, JoystickMode.all);

    final base = joystick.base as JoystickBase;
    const defaults = JoystickBase();
    expect(base.size, joystickBaseSize);
    expect(base.color, defaults.color); // dark gray disc
    expect(base.withBorderCircle, defaults.withBorderCircle); // light ring
    expect(base.mode, defaults.mode);
    expect(base.arrowsColor, defaults.arrowsColor);

    final stick = joystick.stick as JoystickStick;
    expect(stick.size, joystickStickSize);
    expect(stick.color, const JoystickStick().color); // blue knob
    await tester.pumpWidget(const SizedBox());
  });
}
