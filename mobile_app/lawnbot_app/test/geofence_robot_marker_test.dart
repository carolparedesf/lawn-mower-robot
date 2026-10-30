import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:provider/provider.dart';
import 'package:robotmapapp/models/robot_position.dart';
import 'package:robotmapapp/providers/connection_provider.dart';
import 'package:robotmapapp/providers/coverage_map_provider.dart';
import 'package:robotmapapp/providers/geofence_provider.dart';
import 'package:robotmapapp/providers/robot_state_provider.dart';
import 'package:robotmapapp/screens/geofence_screen.dart';
import 'package:robotmapapp/widgets/robot_marker.dart';

class _FakeRobot extends RobotStateProvider {
  final RobotPosition? pos;
  _FakeRobot(this.pos);

  @override
  RobotPosition? get currentPosition => pos;
}

Future<void> _pump(WidgetTester tester, RobotPosition? pos) async {
  tester.view.physicalSize = const Size(900, 2200);
  tester.view.devicePixelRatio = 1.0;
  addTearDown(tester.view.reset);
  await tester.pumpWidget(MultiProvider(
    providers: [
      ChangeNotifierProvider<RobotStateProvider>.value(value: _FakeRobot(pos)),
      ChangeNotifierProvider(create: (_) => ConnectionProvider()),
      ChangeNotifierProvider(create: (_) => GeofenceProvider()),
      ChangeNotifierProvider(create: (_) => CoverageMapProvider()),
    ],
    child: MaterialApp(home: Scaffold(body: GeofenceScreen(onSettingsTap: () {}))),
  ));
  await tester.pump();
}

void main() {
  testWidgets('the robot is drawn on the fence map when its position is known',
      (tester) async {
    await _pump(
        tester,
        RobotPosition(lat: -25.33, lng: -57.51, heading: 90, timestamp: DateTime.now()));
    expect(find.byKey(const Key('robot_marker_layer')), findsOneWidget);
    expect(find.byType(RobotMarker), findsOneWidget);
    expect(find.byKey(const Key('center_on_robot')), findsOneWidget);
  });

  testWidgets('no marker without a position; the button explains why', (tester) async {
    await _pump(tester, null);
    expect(find.byKey(const Key('robot_marker_layer')), findsNothing);
    expect(find.byType(RobotMarker), findsNothing);
    await tester.tap(find.byKey(const Key('center_on_robot')));
    await tester.pump();
    expect(find.text('Todavía no hay posición del robot'), findsOneWidget);
  });
}
