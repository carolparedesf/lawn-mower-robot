import 'package:flutter_test/flutter_test.dart';
import 'package:robotmapapp/models/robot_status.dart';
import 'package:robotmapapp/widgets/battery_display.dart';

void main() {
  group('RobotStatus without battery', () {
    test('missing battery key is null, not 0', () {
      final s = RobotStatus.fromJson({'mode': 'idle', 'alert': 'none'});
      expect(s.battery, isNull);
      expect(batteryLabel(s.battery), '--');
    });

    test('reported battery is kept', () {
      final s = RobotStatus.fromJson({'mode': 'idle', 'battery': 73});
      expect(s.battery, 73);
      expect(batteryLabel(s.battery), '73%');
    });

    test('0 reported by the robot is a real 0, distinct from missing', () {
      expect(RobotStatus.fromJson({'battery': 0}).battery, 0);
    });

    test('toJson omits a null battery and round-trips', () {
      final s = RobotStatus.fromJson({'mode': 'manual', 'alert': 'lifted'});
      final json = s.toJson();
      expect(json.containsKey('battery'), isFalse);
      expect(RobotStatus.fromJson(json).battery, isNull);
    });

    test('unknown battery is neutral grey, not the low-battery red', () {
      expect(batteryColorFor(null), isNot(batteryColorFor(5)));
    });

    test('existing keys keep their defaults', () {
      final s = RobotStatus.fromJson({});
      expect(s.mode, 'idle');
      expect(s.alert, 'none');
      expect(s.blade, isFalse);
      expect(s.trimmer, isFalse);
    });
  });
}
