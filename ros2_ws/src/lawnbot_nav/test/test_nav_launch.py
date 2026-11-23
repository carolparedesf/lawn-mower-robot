import importlib.util
import os
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH = os.path.join(HERE, '..', 'launch', 'nav.launch.py')

try:
    from launch.actions import DeclareLaunchArgument
    from launch_ros.actions import Node
    HAVE_LAUNCH = True
except ImportError:                                  # sin ROS 2 en el entorno
    HAVE_LAUNCH = False


def load():
    spec = importlib.util.spec_from_file_location('nav_launch', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.generate_launch_description()


@unittest.skipUnless(HAVE_LAUNCH, 'requiere ROS 2 (launch, launch_ros)')
class GeofenceLaunchArgsTests(unittest.TestCase):
    def setUp(self):
        try:
            self.entities = load().entities
        except Exception as e:                       # paquete sin instalar (falta source install/setup.bash)
            if type(e).__name__ == 'PackageNotFoundError':
                self.skipTest('lawnbot_nav no esta instalado en el entorno ROS')
            raise

    def defaults(self):
        return {e.name: ''.join(t.text for t in e.default_value)
                for e in self.entities if isinstance(e, DeclareLaunchArgument)}

    def test_argumentos_con_el_comportamiento_actual_por_defecto(self):
        d = self.defaults()
        self.assertEqual(d['geofence_conversion_mode'], 'navsat')
        self.assertEqual(d['datum_lat'], '0.0')
        self.assertEqual(d['datum_lng'], '0.0')

    def test_se_pasan_al_geofence_node(self):
        node = next(e for e in self.entities if isinstance(e, Node) and e.node_executable == 'geofence_node')
        params = node._Node__parameters
        # el ultimo diccionario (despues del YAML, asi que lo pisa) trae los tres parametros
        keys = {''.join(t.text for t in k) for p in params if isinstance(p, dict) for k in p}
        for key in ('conversion_mode', 'datum_lat', 'datum_lng'):
            self.assertIn(key, keys)


@unittest.skipUnless(HAVE_LAUNCH, 'requiere ROS 2 (launch, launch_ros)')
class VisionLaunchTests(unittest.TestCase):
    """La vision real es opcional (vision:=false por defecto) y nunca corre junto con vision_stub."""

    def setUp(self):
        try:
            self.entities = load().entities
        except Exception as e:
            if type(e).__name__ == 'PackageNotFoundError':
                self.skipTest('lawnbot_nav no esta instalado en el entorno ROS')
            raise

    def node(self, executable):
        return next(e for e in self.entities if isinstance(e, Node) and e.node_executable == executable)

    def launched(self, executable, **args):
        from launch import LaunchContext
        ctx = LaunchContext()
        defaults = {e.name: ''.join(t.text for t in e.default_value)
                    for e in self.entities if isinstance(e, DeclareLaunchArgument)}
        defaults.update(args)
        ctx.launch_configurations.update(defaults)
        return self.node(executable).condition.evaluate(ctx)

    def test_apagada_por_defecto(self):
        self.assertFalse(self.launched('vision_node'))
        self.assertFalse(self.launched('vision_stub'))

    def test_vision_true_lanza_el_nodo(self):
        self.assertTrue(self.launched('vision_node', vision='true'))

    def test_stub_solo_sin_vision_real(self):
        self.assertTrue(self.launched('vision_stub', use_vision_stub='true'))
        self.assertFalse(self.launched('vision_stub', use_vision_stub='true', vision='true'))


SIM_LAUNCH = os.path.join(HERE, '..', 'launch', 'sim_world.launch.py')


def load_sim():
    spec = importlib.util.spec_from_file_location('sim_world_launch', SIM_LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@unittest.skipUnless(HAVE_LAUNCH, 'requiere ROS 2 (launch, launch_ros)')
class SimWorldLaunchTests(unittest.TestCase):
    def test_argumento_world_con_room_por_defecto(self):
        entities = load_sim().generate_launch_description().entities
        args = {e.name: ''.join(t.text for t in e.default_value) for e in entities if isinstance(e, DeclareLaunchArgument)}
        self.assertEqual(args, {'world': 'room'})

    def test_world_elige_el_yaml(self):
        mod = load_sim()
        self.assertEqual(mod.world_config('/share', 'room'), '/share/config/sim_room.yaml')
        self.assertEqual(mod.world_config('/share', 'field'), '/share/config/sim_field.yaml')
        for world in mod.WORLDS:
            self.assertTrue(os.path.exists(os.path.join(HERE, '..', 'config', f'sim_{world}.yaml')))

    def test_world_invalido(self):
        with self.assertRaises(ValueError):
            load_sim().world_config('/share', 'jardin')


if __name__ == '__main__':
    unittest.main()
