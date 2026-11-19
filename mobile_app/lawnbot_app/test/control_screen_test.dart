import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:provider/provider.dart';
import 'package:robotmapapp/providers/connection_provider.dart';
import 'package:robotmapapp/providers/robot_state_provider.dart';
import 'package:robotmapapp/providers/speed_limit_provider.dart';
import 'package:robotmapapp/screens/control_screen.dart';
import 'package:robotmapapp/widgets/joystick_widget.dart';

void main() {
  testWidgets('manual control is joystick only; mode buttons replace the old autonomous button',
      (tester) async {
    tester.view.physicalSize = const Size(1080, 2400);
    tester.view.devicePixelRatio = 2.75;
    addTearDown(tester.view.reset);
    await tester.pumpWidget(MultiProvider(
      providers: [
        ChangeNotifierProvider(create: (_) => ConnectionProvider()),
        ChangeNotifierProvider(create: (_) => RobotStateProvider()),
        ChangeNotifierProvider(create: (_) => SpeedLimitProvider()),
      ],
      child: MaterialApp(
        home: Scaffold(
          body: ControlScreen(onSettingsTap: () {}, onNavigate: (_) {}),
        ),
      ),
    ));
    await tester.pump(const Duration(seconds: 10)); // joystick intro animation

    expect(find.byType(JoystickWidget), findsOneWidget);
    expect(find.text('Flechas'), findsNothing);
    expect(find.text('Joystick'), findsNothing);
    expect(find.text('Iniciar modo autónomo'), findsNothing);
    expect(find.text('Manual'), findsOneWidget);
    expect(find.text('Autónomo'), findsOneWidget);
    expect(find.text('Stop'), findsOneWidget);
    expect(find.textContaining('▶'), findsNothing);

    await tester.pumpWidget(const SizedBox());
  });
}
