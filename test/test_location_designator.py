import unittest
from pycram.designators.location_designator import *
from pycram.designators import action_designator, object_designator
from pycram.robot_descriptions.robot_description_handler import InitializedRobotDescription as robot_description
from pycram.process_module import simulated_robot
import pycram.enums
import test_bullet_world


class TestActionDesignatorGrounding(test_bullet_world.BulletWorldTest):

    def test_reachability(self):
        self.robot.set_joint_state(robot_description.i.torso_joint, 0.3)
        object_desig = ObjectDesignatorDescription(names=["milk"])
        robot_desig = ObjectDesignatorDescription(names=[robot_description.i.name])
        location_desig = CostmapLocation(object_desig.resolve(), reachable_for=robot_desig.resolve())
        location = location_desig.resolve()
        self.assertTrue(len(location.pose) == 2)
        self.assertTrue(len(location.pose[0]) == 3)
        self.assertTrue(len(location.pose[1]) == 4)
        self.assertTrue("left" in location.reachable_arms or "right" in location.reachable_arms)

    def test_reachability_pose(self):
        robot_desig = ObjectDesignatorDescription(names=[robot_description.i.name])
        location_desig = CostmapLocation([[0.4, 0.6, 0.9], [0, 0, 0, 1]], reachable_for=robot_desig.resolve())
        location = location_desig.resolve()
        self.assertTrue(len(location.pose) == 2)
        self.assertTrue(len(location.pose[0]) == 3)
        self.assertTrue(len(location.pose[1]) == 4)
        self.assertTrue("left" in location.reachable_arms or "right" in location.reachable_arms)

    def test_visibility(self):
        object_desig = ObjectDesignatorDescription(names=["milk"])
        robot_desig = ObjectDesignatorDescription(names=[robot_description.i.name])
        location_desig = CostmapLocation(object_desig.resolve(), visible_for=robot_desig.resolve())
        location = location_desig.resolve()
        self.assertTrue(len(location.pose) == 2)
        self.assertTrue(len(location.pose[0]) == 3)
        self.assertTrue(len(location.pose[1]) == 4)

    def test_reachability_and_visibility(self):
        self.robot.set_joint_state(robot_description.i.torso_joint, 0.3)
        object_desig = ObjectDesignatorDescription(names=["milk"])
        robot_desig = ObjectDesignatorDescription(names=[robot_description.i.name])
        location_desig = CostmapLocation(object_desig.resolve(), reachable_for=robot_desig.resolve(), visible_for=robot_desig.resolve())
        location = location_desig.resolve()
        self.assertTrue(len(location.pose) == 2)
        self.assertTrue(len(location.pose[0]) == 3)
        self.assertTrue(len(location.pose[1]) == 4)
        self.assertTrue("left" in location.reachable_arms or "right" in location.reachable_arms)

    def test_semantic_location(self):
        kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
        location_desig = SemanticCostmapLocation("kitchen_island_surface", kitchen_desig.resolve())
        location = location_desig.resolve()
        self.assertTrue(len(location.pose) == 2)
        self.assertTrue(len(location.pose[0]) == 3)
        self.assertTrue(len(location.pose[1]) == 4)

        milk_desig = ObjectDesignatorDescription(names=["milk"])
        location_desig = SemanticCostmapLocation("kitchen_island_surface", kitchen_desig.resolve(), for_object=milk_desig.resolve())
        location = location_desig.resolve()
        self.assertTrue(len(location.pose) == 2)
        self.assertTrue(len(location.pose[0]) == 3)
        self.assertTrue(len(location.pose[1]) == 4)
