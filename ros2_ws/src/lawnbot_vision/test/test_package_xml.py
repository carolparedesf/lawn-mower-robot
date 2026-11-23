"""Todos los package.xml del workspace tienen que ser XML valido para catkin_pkg.

Si no lo son, colcon identifica el paquete como python comun (no ros.ament_python), no genera el hook
ament_prefix_path y `source install/setup.bash` no lo registra en AMENT_PREFIX_PATH (ros2 run/launch no lo
encuentran). Paso con un comentario que tenia `--` (prohibido dentro de <!-- -->)."""
import glob
import os
import unittest

try:
    from catkin_pkg.package import parse_package
    HAVE_CATKIN = True
except ImportError:
    HAVE_CATKIN = False

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')


@unittest.skipUnless(HAVE_CATKIN, 'requiere catkin_pkg (ROS 2)')
class PackageXml(unittest.TestCase):
    def test_todos_se_parsean_y_declaran_su_build_type(self):
        paths = sorted(glob.glob(os.path.join(SRC, '*', 'package.xml')))
        self.assertTrue(paths)
        for path in paths:
            with self.subTest(path=os.path.relpath(path, SRC)):
                pkg = parse_package(os.path.dirname(path))
                self.assertTrue(pkg.get_build_type())

    def test_lawnbot_vision_es_ament_python(self):
        pkg = parse_package(os.path.join(SRC, 'lawnbot_vision'))
        self.assertEqual(pkg.get_build_type(), 'ament_python')


if __name__ == '__main__':
    unittest.main()
