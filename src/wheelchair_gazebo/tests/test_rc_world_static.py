#!/usr/bin/env python3
"""Static contract checks for the deterministic Gazebo Classic RC scenario."""

import hashlib
import importlib.util
import os
import unittest
import xml.etree.ElementTree as ET


PACKAGE = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
WORLD_PATH = os.path.join(PACKAGE, "worlds", "wheelchair_rc_scenarios.world")
RC_LAUNCH_PATH = os.path.join(PACKAGE, "launch", "rc_sim.launch")
ADAPTER_PATH = os.path.join(PACKAGE, "scripts", "simulation_controller_adapter.py")
BRINGUP_PATH = os.path.abspath(
    os.path.join(PACKAGE, "..", "wheelchair_bringup", "launch", "sim_bringup.launch")
)

REPO_ROOT = os.path.abspath(os.path.join(PACKAGE, "..", ".."))
GENERATOR_PATH = os.path.join(REPO_ROOT, "scripts", "generate_hanyang_gazebo_model.py")
MAP_PATH = os.path.join(REPO_ROOT, "data", "hanyang_aegimun_loop", "map.pgm")
MODEL_PATH = os.path.join(
    PACKAGE, "models", "hanyang_aegimun_occupancy_walls", "model.sdf"
)
MESH_PATH = os.path.join(
    PACKAGE, "models", "hanyang_aegimun_occupancy_walls", "meshes", "occupancy_walls.obj"
)
EXPECTED_MODEL_SHA256 = "d2e1e1742e7d09c59d78cf28ee2ad38f43721772de9535cab793515ccc90be6f"
EXPECTED_MESH_SHA256 = "50730156c13b3c0967256129329fdb978a51d1fc5634248610f1ebe388fe725e"


def named(elements):
    return {element.attrib["name"]: element for element in elements}


class RcWorldStaticTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world = ET.parse(WORLD_PATH).getroot().find("world")
        cls.rc_launch = ET.parse(RC_LAUNCH_PATH).getroot()
        cls.bringup = ET.parse(BRINGUP_PATH).getroot()
        with open(RC_LAUNCH_PATH, encoding="utf-8") as stream:
            cls.rc_text = stream.read()
        with open(ADAPTER_PATH, encoding="utf-8") as stream:
            cls.adapter_text = stream.read()
        with open(BRINGUP_PATH, encoding="utf-8") as stream:
            cls.bringup_text = stream.read()
        spec = importlib.util.spec_from_file_location("hanyang_generator", GENERATOR_PATH)
        cls.generator = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.generator)

    def test_world_has_all_ground_truth_contexts(self):
        models = named(self.world.findall("model"))
        expected = {
            "ground_truth_narrow_sidewalk",
            "ground_truth_curb_north",
            "ground_truth_curb_south",
            "ground_truth_road_free_space",
            "ground_truth_ramp_uphill_10deg",
            "ground_truth_ramp_downhill_10deg",
            "ground_truth_cross_slope_7deg",
            "ground_truth_static_obstacle_box",
            "ground_truth_static_obstacle_bollard",
            "ground_truth_blind_corner_wall_x",
            "ground_truth_blind_corner_wall_y",
            "ground_truth_occluder_parked_van",
            "ground_truth_geofence_sidewalk",
            "ground_truth_geofence_road_free_space",
        }
        self.assertTrue(expected.issubset(models))
        self.assertEqual(models["ground_truth_narrow_sidewalk"].findtext("pose"), "5 0 0.025 0 0 0")
        self.assertEqual(models["ground_truth_road_free_space"].findtext("pose"), "16 0 0.015 0 0 0")

    def test_ramp_and_cross_slope_geometry_is_explicit(self):
        models = named(self.world.findall("model"))
        uphill = models["ground_truth_ramp_uphill_10deg"]
        downhill = models["ground_truth_ramp_downhill_10deg"]
        cross_slope = models["ground_truth_cross_slope_7deg"]
        self.assertEqual(uphill.findtext("pose").split()[4], "0.174533")
        self.assertEqual(downhill.findtext("pose").split()[4], "-0.174533")
        self.assertEqual(cross_slope.findtext("pose").split()[3], "0.122173")
        self.assertEqual(uphill.findtext("link/collision/geometry/box/size"), "5 2.4 0.10")
        self.assertEqual(cross_slope.findtext("link/collision/geometry/box/size"), "5 3.2 0.10")

    def test_dynamic_crossings_are_repeatable_native_actor_scripts(self):
        actors = named(self.world.findall("actor"))
        self.assertEqual(set(actors), {"ground_truth_crossing_pedestrian", "ground_truth_crossing_car"})
        for actor in actors.values():
            self.assertEqual(actor.findtext("script/loop"), "true")
            self.assertEqual(actor.findtext("script/auto_start"), "true")
            self.assertGreaterEqual(len(actor.findall("script/trajectory/waypoint")), 3)
            self.assertEqual(actor.findtext("skin/filename"), "file://media/models/walk.dae")
            self.assertEqual(
                actor.findtext("animation/filename"), "file://media/models/walk.dae"
            )
        self.assertEqual(actors["ground_truth_crossing_pedestrian"].findtext("script/delay_start"), "2.0")
        self.assertEqual(actors["ground_truth_crossing_car"].findtext("script/delay_start"), "5.0")

    def test_world_physics_and_ground_truth_plugin_are_deterministic(self):
        physics = self.world.find("physics")
        self.assertEqual(physics.attrib, {"name": "deterministic_ode", "type": "ode"})
        self.assertEqual(physics.findtext("max_step_size"), "0.001")
        self.assertEqual(physics.findtext("ode/solver/iters"), "100")
        plugins = self.world.findall("model/plugin")
        self.assertEqual(len(plugins), 1)
        self.assertEqual(plugins[0].attrib["filename"], "libgazebo_ros_p3d.so")
        self.assertEqual(plugins[0].findtext("gaussianNoise"), "0.0")

    def test_rc_launch_binds_seed_headless_policies_and_only_simulation_sink(self):
        args = named(self.rc_launch.findall("arg"))
        for required in (
            "world", "scenario", "seed", "gui", "headless", "route_policy",
            "route_safety_policy", "localization_policy", "collision_policy",
            "slope_policy",
        ):
            self.assertIn(required, args)
        self.assertEqual(args["seed"].attrib["default"], "1701")
        self.assertEqual(args["gui"].attrib["default"], "false")
        self.assertEqual(args["headless"].attrib["default"], "true")
        gazebo_include = next(
            include for include in self.rc_launch.findall("include")
            if "gazebo_ros" in include.attrib["file"]
        )
        include_args = named(gazebo_include.findall("arg"))
        self.assertEqual(include_args["extra_gazebo_args"].attrib["value"], "--seed $(arg seed)")
        nodes = self.rc_launch.findall("node")
        by_type = {node.attrib["type"]: node for node in nodes}
        self.assertEqual(
            set(by_type),
            {"sim_sensor_canonicalizer.py", "simulation_controller_adapter.py"},
        )
        adapter = by_type["simulation_controller_adapter.py"]
        self.assertEqual(
            adapter.attrib,
            {
                "pkg": "wheelchair_gazebo",
                "type": "simulation_controller_adapter.py",
                "name": "simulation_controller_adapter",
                "output": "screen",
                "required": "true",
            },
        )
        canonicalizer = by_type["sim_sensor_canonicalizer.py"]
        self.assertEqual(canonicalizer.attrib["name"], "sim_sensor_canonicalizer")
        self.assertEqual(canonicalizer.attrib["required"], "true")
        self.assertNotIn("controller_sink", args)
        self.assertNotIn("topic_tools", self.rc_text)
        for node in nodes:
            identity = " ".join(node.attrib.values()).lower()
            for forbidden in ("relay", "mux", "plugin"):
                self.assertNotIn(forbidden, identity)

    def test_bringup_composes_canonical_software_only_chain(self):
        includes = "\n".join(include.attrib["file"] for include in self.bringup.findall("include"))
        for launch in (
            "wheelchair_gazebo)/launch/rc_sim.launch",
            "wheelchair_perception)/launch/perception.launch",
            "wheelchair_navigation)/launch/localization.launch",
            "wheelchair_navigation)/launch/route_manager.launch",
            "wheelchair_decision)/launch/decision.launch",
            "wheelchair_navigation)/launch/navigation.launch",
            "wheelchair_safety)/launch/safety.launch",
            "wheelchair_navigation)/launch/control_monitor.launch",
        ):
            self.assertIn(launch, includes)
        route_safety = self.bringup.find("node[@pkg='wheelchair_route_safety']")
        self.assertIsNotNone(route_safety)
        self.assertEqual(route_safety.find("param[@name='config_path']").attrib["value"], "$(arg route_safety_policy)")
        for topic in (
            "/sensors/lidar/points", "/sensors/imu/data",
            "/base_model/localization_pose", "/cmd_vel_nav", "/cmd_vel_safe",
        ):
            self.assertIn(topic, self.bringup_text)
        self.assertIn('<param name="/use_sim_time" value="true"/>', self.bringup_text)
        self.assertIn('<param name="/hardware_motion_authorized" value="false"/>', self.bringup_text)
        self.assertIn('<param name="/passenger_operation_authorized" value="false"/>', self.bringup_text)

    def test_adapter_has_fixed_command_ready_inputs_and_one_simulation_output(self):
        self.assertEqual(self.adapter_text.count("rospy.Subscriber("), 2)
        self.assertEqual(self.adapter_text.count("rospy.Publisher("), 1)
        self.assertIn('SOURCE_TOPIC = "/cmd_vel_safe"', self.adapter_text)
        self.assertIn('READY_TOPIC = "/wheelchair_base_controller/odom"', self.adapter_text)
        self.assertIn(
            'SINK_TOPIC = "/wheelchair_base_controller/cmd_vel"',
            self.adapter_text,
        )
        self.assertNotIn("get_param(\"~", self.adapter_text)

    def test_no_real_motor_or_ros2_actuation_path(self):
        combined = self.rc_text + self.bringup_text + self.adapter_text
        forbidden = (
            '<node pkg="wheelchair_hardware"',
            "wheelchair_hardware",
            "hardware_adapter",
            "/hardware/cmd_vel",
            "/motor",
            "/real_motor",
            "motor_topic",
            "ros2 launch",
            "ament_",
            "controller_sink",
        )
        for token in forbidden:
            self.assertNotIn(token, combined)

    def test_hanyang_mesh_is_exact_coalesced_occupancy_boundary(self):
        with open(MAP_PATH, "rb") as stream:
            width, height, raster = self.generator.read_pgm(stream.read())
        occupied, occupied_count = self.generator.occupied_cells(raster, width, height)
        segments = self.generator.boundary_segments(occupied, width, height)

        expected_edges = set()
        for row, column in occupied:
            bottom = height - row - 1
            if row == height - 1 or (row + 1, column) not in occupied:
                expected_edges.add(("h", bottom, column))
            if row == 0 or (row - 1, column) not in occupied:
                expected_edges.add(("h", bottom + 1, column))
            if column == 0 or (row, column - 1) not in occupied:
                expected_edges.add(("v", column, bottom))
            if column == width - 1 or (row, column + 1) not in occupied:
                expected_edges.add(("v", column + 1, bottom))

        actual_edges = set()
        for x0, y0, x1, y1 in segments:
            if y0 == y1:
                actual_edges.update(("h", y0, x) for x in range(x0, x1))
            else:
                actual_edges.update(("v", x0, y) for y in range(y0, y1))
        self.assertEqual(occupied_count, 210223)
        self.assertEqual(actual_edges, expected_edges)
        self.assertEqual(len(segments), 32712)

    def test_hanyang_generated_assets_are_deterministic_boundary_quads(self):
        with open(MODEL_PATH, "rb") as stream:
            model_data = stream.read()
        with open(MESH_PATH, "rb") as stream:
            mesh_data = stream.read()
        self.assertEqual(hashlib.sha256(model_data).hexdigest(), EXPECTED_MODEL_SHA256)
        self.assertEqual(hashlib.sha256(mesh_data).hexdigest(), EXPECTED_MESH_SHA256)

        model_text = model_data.decode("ascii")
        self.assertIn("source_occupied_points=16629", model_text)
        self.assertIn("final_occupied_cells=210223", model_text)
        self.assertIn("boundary_segments=32712 boundary_faces=65424", model_text)
        lines = mesh_data.decode("ascii").splitlines()
        vertices = [line.split()[1:] for line in lines if line.startswith("v ")]
        faces = [tuple(map(int, line.split()[1:])) for line in lines if line.startswith("f ")]
        self.assertEqual(len(vertices), 65424)
        self.assertEqual(len(faces), 65424)
        self.assertLess(len(faces), 139524)
        self.assertTrue(all(len(face) == 4 for face in faces))
        for front, back in zip(faces[::2], faces[1::2]):
            self.assertEqual(back, tuple(reversed(front)))
            points = [vertices[index - 1] for index in front]
            self.assertEqual({point[2] for point in points}, {"0", "2"})
            self.assertTrue(
                len({point[0] for point in points}) == 1
                or len({point[1] for point in points}) == 1
            )

        yaml_path = os.path.join(REPO_ROOT, "data", "hanyang_aegimun_loop", "map.yaml")
        with open(yaml_path, "rb") as stream:
            yaml_data = stream.read()
        with open(MAP_PATH, "rb") as stream:
            pgm_data = stream.read()
        width, height, raster = self.generator.read_pgm(pgm_data)
        occupied, occupied_count = self.generator.occupied_cells(raster, width, height)
        segments = self.generator.boundary_segments(occupied, width, height)
        generated_sdf = "".join(self.generator.sdf_lines(
            hashlib.sha256(yaml_data).hexdigest(),
            hashlib.sha256(pgm_data).hexdigest(),
            segments,
            width,
            height,
            self.generator.EXPECTED_RESOLUTION,
            self.generator.EXPECTED_ORIGIN,
            occupied_count,
        )).encode("ascii")
        generated_mesh = "".join(self.generator.obj_lines(
            segments,
            self.generator.EXPECTED_RESOLUTION,
            self.generator.EXPECTED_ORIGIN,
        )).encode("ascii")
        self.assertEqual(generated_sdf, model_data)
        self.assertEqual(generated_mesh, mesh_data)

if __name__ == "__main__":
    unittest.main()
