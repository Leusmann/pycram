from dataclasses import dataclass

import numpy as np
import sqlalchemy.orm
import sqlalchemy.sql
from sqlalchemy import select, Select, join
from sqlalchemy.sql.elements import BinaryExpression

from typing_extensions import Tuple

import pycram.designators.location_designator
import pycram.task
from pycram.costmaps import OccupancyCostmap
from pycram.orm.action_designator import PickUpAction, Action
from pycram.orm.object_designator import Object
from pycram.orm.base import Position, RobotState, Pose as ORMPose, Quaternion
from pycram.orm.task import TaskTreeNode, Code
from ...pose import Pose


@dataclass
class Location(pycram.designators.location_designator.LocationDesignatorDescription.Location):
    """
    A location that is described by a pose, a reachable arm, a torso height and a grasp.
    """
    pose: Pose
    reachable_arm: str
    torso_height: float
    grasp: str


class RequiresDatabase:
    """
    Mixin class that provides a database session.
    """

    robot_pos = sqlalchemy.orm.aliased(Position)
    """3D Vector of robot position"""
    robot_pose = sqlalchemy.orm.aliased(ORMPose)
    object_pos = sqlalchemy.orm.aliased(Position)
    relative_x = robot_pos.x - object_pos.x
    relative_y = robot_pos.y - object_pos.y

    def __init__(self, session: sqlalchemy.orm.Session = None):
        """
        Create a new RequiresDatabase instance.

        :param session: The database session
        """
        self.session = session

    def create_query(self) -> Select:
        """
        Create a query that queries the database for all pick up actions with context.
        """

        # query all relative robot positions in regard to an objects position
        # make sure to order the joins() correctly
        query = self.join_statement(self.select_statement())
        return query

    def select_statement(self):
        return select(PickUpAction.arm, PickUpAction.grasp, RobotState.torso_height, self.relative_x, self.relative_y,
               Quaternion.x, Quaternion.y, Quaternion.z, Quaternion.w).distinct()
    def join_statement(self, query: Select):
        return (query.join(TaskTreeNode.code).join(Code.designator.of_type(PickUpAction)).
                join(PickUpAction.robot_state)
                .join(self.robot_pose, RobotState.pose)
                .join(self.robot_pos, self.robot_pose.position)
                .join(ORMPose.orientation)
                .join(PickUpAction.object)
                .join(Object.pose)
                .join(self.object_pos, ORMPose.position))


class DatabaseCostmapLocation(pycram.designators.location_designator.CostmapLocation, RequiresDatabase):
    """
    Class that represents costmap locations from a given Database.
    The database has to have a schema that is compatible with the pycram.orm package.
    """

    def __init__(self, target, session: sqlalchemy.orm.Session = None,
                 reachable_for=None, reachable_arm=None, resolver=None):
        """
        Create a Database Costmap

        :param target: The target object
        :param session: A session that can be used to execute queries
        :param reachable_for: The robot to grab the object with
        :param reachable_arm: The arm to use

        """
        super().__init__(target, reachable_for, None, reachable_arm, resolver)
        RequiresDatabase.__init__(self, session)

    def create_query_from_occupancy_costmap(self) -> Select:
        """
        Create a query that queries all relative robot positions from an object that are not occluded using an
        OccupancyCostmap.
        """

        # get query
        query = self.create_query()

        # constraint query to correct object type and successful task status
        query = query.where(Object.type == self.target.type).where(TaskTreeNode.status == "SUCCEEDED")

        # create Occupancy costmap for the target object
        ocm = OccupancyCostmap(distance_to_obstacle=0.3, from_ros=False, size=200, resolution=0.02,
                               origin=self.target.pose)

        # working on a copy of the costmap, since found rectangles are deleted
        map = np.copy(ocm.map)

        origin = np.array([ocm.height / 2, ocm.width / 2])

        filters = []

        # for every index pair (i, j) in the occupancy costmap
        for i in range(0, map.shape[0]):
            for j in range(0, map.shape[1]):

                # if this index has not been used yet
                if map[i][j] > 0:
                    curr_width = ocm._find_consectuive_line((i, j), map)
                    curr_pose = (i, j)
                    curr_height = ocm._find_max_box_height((i, j), curr_width, map)

                    # calculate the rectangle in the costmap
                    x_lower = (curr_pose[0] - origin[0]) * ocm.resolution
                    x_upper = (curr_pose[0] + curr_width - origin[0]) * ocm.resolution
                    y_lower = (curr_pose[1] - origin[1]) * ocm.resolution
                    y_upper = (curr_pose[1] + curr_height - origin[1]) * ocm.resolution

                    # mark the found rectangle as occupied
                    map[i:i + curr_height, j:j + curr_width] = 0

                    # transform to jpt query
                    filters.append(sqlalchemy.and_(self.relative_x >= x_lower, self.relative_x < x_upper,
                                                   self.relative_y >= y_lower, self.relative_y < y_upper))

        return query.where(sqlalchemy.or_(*filters))

    def sample_to_location(self, sample: sqlalchemy.engine.row.Row) -> Location:
        """
        Convert a database row to a costmap location.

        :param sample: The database row.
        :return: The costmap location
        """

        target_x, target_y, target_z = self.target.pose.position_as_list()
        position = [target_x + sample[3], target_y + sample[4], 0]
        orientation = [sample[5], sample[6], sample[7], sample[8]]

        result = Location(Pose(position, orientation), sample.arm, sample.torso_height, sample.grasp)
        return result

    def __iter__(self) -> Location:
        statement = self.create_query_from_occupancy_costmap().limit(200)
        samples = self.session.execute(statement).all()
        if samples:
            for sample in samples:
                yield self.sample_to_location(sample)
        else:
            raise ValueError("No samples found")
