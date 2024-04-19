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
milk = Object("milk", ObjectType.MILK, "MilkBox.stl", pose=Pose([6.85, -4.95, 0.97]), color=[1, 0, 0, 1])
# bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([2.4, 2.3, 0.95]), color=[1, 1, 0, 1])
# bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([7.05, -5.00, 0.91],[0,0,1,0]) , color=[1, 1, 0, 1])
bowl = Object("bowl", ObjectType.BOWL, "SmallBowl.stl", pose=Pose([7.05, -4.92, 0.90],[0,0,1,0]), color=[1, 1, 0, 1])
# cup = Object("cup", ObjectType.JEROEN_CUP, "jeroen_cup.stl", pose=Pose([2.8, 2.5, 1.0]), color=[1, 1, 1, 1])
# cup = Object("cup", ObjectType.JEROEN_CUP, "Cup.stl", pose=Pose([7.05, -5.17, 0.90],[0,0,1,0]), color=[1, 1, 1, 1])
# cerial = Object("cerial", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([2.5, 2.7, 1.05]),
cerial = Object("cerial", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([6.85, -5.47, 0.97]),
                color=[0, 1, 0, 1])
spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([7.13, -5.33, 0.75]), color=[0, 0, 1, 1])
# apartment.attach(spoon, 'cabinet10_drawer_top')
apartment.attach(spoon, 'SM_Kitchen_09_Drawer_01')


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

    # Not sure how to generalize maybe list will items I need to move?
    # There are semnatic costmaps which could give me pose for items
    objects_to_move=[milk,bowl,cerial]

    drop_location=iter(SemanticCostmapLocation(urdf_link_name="SM_DiningTable", part_of=apartment_desig.resolve()))
    for item in objects_to_move:
        item_desig=move_and_detect(item.type)

        drop_location_modified = next(drop_location)
        drop_location_modified.pose.position.z += .5
        while btr.pospection_contact(item, apartment, drop_location_modified.pose):
            drop_location_modified = next(drop_location)
            drop_location_modified.pose.position.z += .7

        TransportAction(item_desig,["left"],[drop_location_modified.pose]).resolve().perform()
        # TransportAction(drink_desig, ["left"], [Pose([3.85, -6.58, 0.86])]).resolve().perform()

    NavigateAction(target_locations=[Pose([8, -5.35, 0],[0,0,1,0])]).resolve().perform()
    # Finding and navigating to the drawer holding the spoon
    handle_desig = ObjectPart(names=["SM_Kitchen_Handle15"], part_of=apartment_desig.resolve())
    drawer_open_loc = AccessingLocation(handle_desig=handle_desig.resolve(),
                                        robot_desig=robot_desig.resolve()).resolve()

    NavigateAction([drawer_open_loc.pose]).resolve().perform()

    # OpenAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
    OpenAction(object_designator_description=handle_desig, arms=["left"]).resolve().perform()
    spoon.detach(apartment)

    # Detect and pickup the spoon
    LookAtAction([apartment.get_link_pose("SM_Kitchen_Handle15")]).resolve().perform()

    spoon_desig = DetectAction(BelieveObject(types=[ObjectType.SPOON])).resolve().perform()

    pickup_arm = "right"# if drawer_open_loc.arms[0] == "right" else "right"
    PickUpAction(spoon_desig, [pickup_arm], ["top"]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()

    close_loc = drawer_open_loc.pose
    # close_loc.position.y -= 0.2
    NavigateAction([close_loc]).resolve().perform()

    # CloseAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
    CloseAction(object_designator_description=handle_desig, arms=["left"]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([0.15]).resolve().perform()

    # Find a pose to place the spoon, move and then place it
    spoon_target_pose = Pose([3.42, -6.88, 0.80], [0, 0, 1, 1])
    # item_desig = move_and_detect(spoon.type)
    # drop_location_modified = next(drop_location)
    # drop_location_modified.pose.position.z += .5
    placing_loc = CostmapLocation(target=spoon_target_pose, reachable_for=robot_desig.resolve()).resolve()

    NavigateAction([placing_loc.pose]).resolve().perform()

    PlaceAction(spoon_desig, [spoon_target_pose], [pickup_arm]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()


    # drink_type = knowrob.Prolog.ensure_once("[Query]")
    # maybe I need to do something with the food designator (Jonas said something aboyut the type and mapping this to the
    # Objecttype Enum

    # drink_desig = move_and_detect(ObjectType.MILK)
    #
    # drop_location = SemanticCostmapLocation(urdf_link_name="SM_DiningTable", part_of=apartment_desig.resolve(), for_object=drink_desig)
    # print(drop_location.resolve())
    #
    # TransportAction(drink_desig, ["left"], [Pose([3.85, -6.58, 0.86])]).resolve().perform()
    #
    # # container_type = knowrob.Prolog.ensure_once("[Query]")
    # # Same as food designator
    #
    # # container_desig = move_and_detect(ObjectType.JEROEN_CUP)
    # container_desig = move_and_detect(ObjectType.BOWL)
    #
    #
    # TransportAction(container_desig, ["left"], [Pose([4.20, -6.58, 0.80], [0, 0, 1, 1])]).resolve().perform()
