from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.enums import ObjectType
import pycram.external_interfaces.knowrob as knowrob
import pycram.bullet_world_reasoning as btr
import matplotlib.pyplot as plt

world = BulletWorld()
# robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=Pose([1, 2, 0]))
robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=Pose([8, -5.35, 0],[0,0,1,0]))
# apartment = Object("apartment", ObjectType.ENVIRONMENT, "apartment.urdf")
# apartment = Object("apartment", ObjectType.ENVIRONMENT, "apartment-paper.urdf")
apartment = Object("apartment", ObjectType.ENVIRONMENT, "Modified_ApartmentECAI.urdf")

# milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.6, 2.8, 1.05]), color=[1, 0, 0, 1])
milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([6.85, -4.95, 0.97]), color=[1, 0, 0, 1])
# bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([2.4, 2.3, 0.95]), color=[1, 1, 0, 1])
bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([7.05, -4.92, 0.90]), color=[1, 1, 0, 1])
# cup = Object("cup", ObjectType.JEROEN_CUP, "jeroen_cup.stl", pose=Pose([2.8, 2.5, 1.0]), color=[1, 1, 1, 1])
cup = Object("cup", ObjectType.JEROEN_CUP, "jeroen_cup.stl", pose=Pose([7.05, -5.17, 0.98],[0,0,1,0]), color=[1, 1, 1, 1])
# cerial = Object("cerial", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([2.5, 2.7, 1.05]),
cerial = Object("cerial", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([6.85, -5.47, 0.97]),
                color=[0, 1, 0, 1])
# apartment.attach(spoon, 'cabinet10_drawer_top')

# pick_pose = Pose([2.7, 2.5, 1])
pick_pose = Pose([7, -5.2, 1])

robot_desig = BelieveObject(names=["pr2"])
apartment_desig = BelieveObject(names=["apartment"])


@with_simulated_robot
def move_and_detect(obj_type):
    NavigateAction(target_locations=[Pose([8, -5.35, 0],[0,0,1,0])]).resolve().perform()

    LookAtAction(targets=[pick_pose]).resolve().perform()

    object_desig = DetectAction(BelieveObject(types=[obj_type])).resolve().perform()

    return object_desig

with simulated_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([0.25]).resolve().perform()

    # drink_type = knowrob.Prolog.ensure_once("[Query]")
    # maybe I need to do something with the food designator (Jonas said something aboyut the type and mapping this to the
    # Objecttype Enum


    drink_desig = move_and_detect(ObjectType.MILK)

    drink_desig = move_and_detect(ObjectType.MILK)

    TransportAction(drink_desig, ["left"], [Pose([4.4, 3.7, 1.1])]).resolve().perform()

    # container_type = knowrob.Prolog.ensure_once("[Query]")
    # Same as food designator

    container_desig = move_and_detect(ObjectType.JEROEN_CUP)
    # bowl_desig = move_and_detect(ObjectType.BOWL)

    TransportAction(container_desig, ["left"], [Pose([4.8, 3.7, 1.1], [0, 0, 1, 1])]).resolve().perform()
