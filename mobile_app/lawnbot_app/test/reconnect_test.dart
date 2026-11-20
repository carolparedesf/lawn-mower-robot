import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:provider/provider.dart';
import 'package:robotmapapp/providers/connection_provider.dart';
import 'package:robotmapapp/providers/robot_state_provider.dart';
import 'package:robotmapapp/theme/robo_theme.dart';
import 'package:robotmapapp/widgets/robo_top_bar.dart';

const _tick = Duration(milliseconds: 20);

/// Lets the periodic retry timer run for a few ticks.
Future<void> _wait([int ticks = 6]) =>
    Future<void>.delayed(_tick * ticks);

void main() {
  late StreamController<bool> link; // what MqttService.connectionStream emits
  late int attempts;
  late int stateRequests;

  setUp(() {
    link = StreamController<bool>.broadcast();
    attempts = 0;
    stateRequests = 0;
  });

  tearDown(() => link.close());

  ConnectionProvider make(Future<bool> Function(String) connector) =>
      ConnectionProvider(
        connectionStream: link.stream,
        connector: connector,
        requestState: () => stateRequests++,
        retryEvery: _tick,
      );

  test('the retry period is 2 s', () {
    expect(reconnectInterval, const Duration(seconds: 2));
  });

  test('a dropped link is retried by itself until it comes back, and the '
      'state is requested again', () async {
    final conn = make((_) async {
      attempts++;
      return attempts >= 3; // fails twice, then connects
    });
    link.add(true); // was connected
    await _wait(1);
    expect(conn.isConnected, isTrue);
    expect(conn.isReconnecting, isFalse);

    link.add(false); // the link drops
    await _wait(1);
    expect(conn.isConnected, isFalse);
    expect(conn.isReconnecting, isTrue);

    await _wait(10);
    expect(attempts, 3);
    expect(conn.isConnected, isTrue);
    expect(conn.isReconnecting, isFalse);
    expect(stateRequests, 1, reason: 'state asked once, after reconnecting');

    await _wait(5); // no more attempts once connected
    expect(attempts, 3);
    conn.dispose();
  });

  test('keeps retrying while the broker stays down', () async {
    final conn = make((_) async {
      attempts++;
      return false;
    });
    link.add(true);
    link.add(false);
    await _wait(10);
    expect(attempts, greaterThanOrEqualTo(3));
    expect(conn.isReconnecting, isTrue);
    expect(stateRequests, 0);
    conn.dispose();
  });

  test('attempts do not overlap while one is still pending', () async {
    final pending = Completer<bool>();
    final conn = make((_) {
      attempts++;
      return pending.future;
    });
    link.add(true);
    link.add(false);
    await _wait(8);
    expect(attempts, 1);
    pending.complete(true);
    await _wait(2);
    expect(conn.isConnected, isTrue);
    expect(conn.isReconnecting, isFalse);
    conn.dispose();
  });

  test('no retries if the app never had a connection', () async {
    final conn = make((_) async {
      attempts++;
      return true;
    });
    link.add(false);
    await _wait(6);
    expect(attempts, 0);
    expect(conn.isReconnecting, isFalse);
    conn.dispose();
  });

  test('no retries after the user disconnects', () async {
    final conn = make((_) async {
      attempts++;
      return true;
    });
    link.add(true);
    await _wait(1);
    conn.disconnect(); // the user's own disconnection
    link.add(false);
    await _wait(6);
    expect(attempts, 0);
    expect(conn.isReconnecting, isFalse);
    conn.dispose();
  });

  test('a manual connect() cancels the automatic retries', () async {
    var linkUp = false; // the broker is back only when the user asks
    final conn = make((_) async {
      attempts++;
      return linkUp;
    });
    link.add(true);
    link.add(false);
    await _wait(1);
    expect(conn.isReconnecting, isTrue);
    linkUp = true;
    await conn.connect();
    expect(conn.isConnected, isTrue);
    expect(conn.isReconnecting, isFalse);
    expect(stateRequests, 1);
    final seen = attempts;
    await _wait(5);
    expect(attempts, seen);
    conn.dispose();
  });

  Future<void> pumpChip(WidgetTester tester, ConnectionProvider conn) async {
    tester.view.physicalSize = const Size(1080, 2400);
    tester.view.devicePixelRatio = 2.75;
    addTearDown(tester.view.reset);
    await tester.pumpWidget(MultiProvider(
      providers: [
        ChangeNotifierProvider<ConnectionProvider>.value(value: conn),
        ChangeNotifierProvider(create: (_) => RobotStateProvider()),
      ],
      child: MaterialApp(
        theme: roboTheme(),
        home: Scaffold(body: RoboTopBar(onSettingsTap: () {})),
      ),
    ));
  }

  testWidgets('the chip says "Reconectando..." while it retries', (tester) async {
    final conn = ConnectionProvider(
      connectionStream: link.stream,
      connector: (_) async => false,
      requestState: () {},
      retryEvery: const Duration(seconds: 2),
    );
    await pumpChip(tester, conn);
    expect(find.text('Desconectado'), findsOneWidget);

    link.add(true);
    await tester.pump(); // delivers the event
    await tester.pump(); // draws it
    expect(find.text('Conectado'), findsOneWidget);

    link.add(false);
    await tester.pump();
    await tester.pump();
    expect(find.text('Reconectando...'), findsOneWidget);
    expect(find.text('Desconectado'), findsNothing);

    await tester.pumpWidget(const SizedBox()); // unmount
    conn.dispose();
  });
}
