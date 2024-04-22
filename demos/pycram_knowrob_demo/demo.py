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

# semrep imports dlquery as dl, and builds a cache for it.
# Therefore, will not import dlquery again, to avoid building another cache.
# dlquery functions are accessible through semrep: sr.dl.*
import dfl.semrep as sr
import itertools
import owlready2
import rdflib

world = BulletWorld()
robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=Pose([8, -5.35, 0],[0,0,1,0]))
apartment = Object("apartment", ObjectType.ENVIRONMENT, "Modified_ApartmentECAI.urdf")

milk = Object("milk", ObjectType.MILK, "MilkBox.stl", pose=Pose([6.85, -4.95, 0.97]), color=[1, 0, 0, 1])
bowl = Object("bowl", ObjectType.BOWL, "SmallBowl.stl", pose=Pose([7.05, -4.92, 0.90],[0,0,1,0]), color=[1, 1, 0, 1])
cerial = Object("cerial", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([6.85, -5.47, 0.97]),
                color=[0, 1, 0, 1])
spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([7.13, -5.33, 0.75]), color=[0, 0, 1, 1])
apartment.attach(spoon, 'SM_Kitchen_09_Drawer_01')


pick_pose = Pose([7, -5.2, 1])

robot_desig = BelieveObject(names=["pr2"])
apartment_desig = BelieveObject(names=["apartment"])

# TODOs
# - debugging, ensuring sensible results of queries

def prepareSemanticMap(sceneGraphOwlFile):
    onto = owlready2.get_ontology("file://"+sceneGraphOwlFile).load()
    # The owlready2 SPARQL engine is quite buggy, prefer to use RDFLib's instead
    semanticMap = owlready2.default_world.as_rdflib_graph()
    return semanticMap

def getObjectNamesFromSemanticMap(semanticMap):
    r = list(semanticMap.query("""
            SELECT DISTINCT ?p WHERE {
                ?p <http://www.w3.org/1999/02/22-rdf-syntax-ns#type> <https://ease-crc.org/ont/USD.owl#Prim>.
            }"""))
    return [str(x[0]) for x in r]

def getObjectTypesFromSemanticMap(objName, semanticMap):
    r = list(semanticMap.query("""
            SELECT DISTINCT ?p WHERE {
                <%s> <http://www.w3.org/1999/02/22-rdf-syntax-ns#type> ?p.
            }""" % (objName)))
    return [str(x[0]) for x in r]

def getObjectPartsFromSemanticMap(objName, semanticMap):
    r = list(semanticMap.query("""
            SELECT DISTINCT ?p WHERE {
                <%s> <http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#hasPart> ?p.
            }""" % (objName)))
    return [str(x[0]) for x in r]

# Obtain semantic reports for all objects in a scene
def initializeSemanticReports(semanticMap):
    def _contract(n, s):
        return s[n:]
    # Assumes semantic labelling was done already, and all we need to do now is
    #     to retrieve the SOMA_DFL types of the various objects
    dflPrefix = "http://www.ease-crc.org/ont/SOMA_DFL.owl#"
    n = len(dflPrefix)
    objNames = getObjectNamesFromSemanticMap(semanticMap)
    semanticReports = {k: [{"SOMA_DFL": "dfl:"+_contract(n, v)} for v in getObjectTypesFromSemanticMap(k, semanticMap)
                              if v.startswith(dflPrefix)] 
                         for k in objNames}
    return semanticReports

# Sanity ceck: concatenate a number of lists, removing repeated elements.
def sanityCheckListOfUniques(*lists):
    aux = set()
    retq = [(x,aux.add(x)) for x in itertools.chain.from_iterable(lists) if x not in aux]
    return [x[0] for x in retq]

def getObjectPartNamesAndSelf(objName: str, semanticMap):
    return set(getObjectPartsFromSemanticMap(objName, semanticMap)).union([objName])

# we assume we have only object names as keys for semantic reports
#     this is true currently as the USD does not record types.
# even when we will use type information when doing a semantic report,
#     it is easy to replace a report for type T with a list of identical reports
#     for each x that is an instance of T
def whatPlausibleDFLClassesForObject(oname: str, semanticReports: dict):
    return [x['SOMA_DFL'] for x in semanticReports.get(oname, []) if 'SOMA_DFL' in x]

def whatPlausibleDFLClassesForAllObjects(semanticMap, semanticReports: dict, whiteList=None):
    if whiteList is None:
        whiteList = getObjectNamesFromSemanticMap(semanticMap)
    return {x: whatPlausibleDFLClassesForObject(x, semanticReports) for x in whiteList}

#    \item Which objects do I need for breakfast?
def whichItemsForMeal(meal: str, semanticMap, semanticReports: dict):
    def _isOrContains(targetConcepts, candidateConcepts):
        return [x for x in candidateConcepts if (x in targetConcepts) or 
                                                targetConcepts.intersection(sr.dl.whatPartTypesDoesObjectHave(x))]
    # currently, only breakfast implemented
    meal = meal.lower().strip()
    if meal in {'breakfast', 'break fast', 'morning meal'}:
        # Find all plausible object classes (classes in the semantic reports) for all objects in the scene.
        plausibleClassMap = whatPlausibleDFLClassesForAllObjects(semanticMap, semanticReports)
        # Find classes of foods believed to be breakfast foods.
        breakfastFoods = set(sr.dl.whatSubclasses('dfl:breakfast_food.n.wn.food'))
        # Find all objects in the scene that plausibly are of, or contain, a breakfast food type.
        plausibleItems = [x for x,v in plausibleClassMap.items() if _isOrContains(breakfastFoods, v)]
        # Make a set of breakfast food types that we actually might have in the scene
        foundBreakfastFoodTypes = breakfastFoods.intersection(itertools.chain.from_iterable(plausibleClassMap.values()))
        # Find all classes of objects believed to be appropriate tools to serve food of the found types.
        plausibleUtensilTypes = itertools.chain.from_iterable(
                                    [sr.dl.whatToolsCanPerformTaskOnObject('dfl:serve.v.wn.consume..concrete', x) for
                                        x in foundBreakfastFoodTypes])
        plausibleUtensilTypes = set(plausibleUtensilTypes)
        # Find all objects in the scene that plausibly are of the appropriate utensil types.
        plausibleUtensils = [x for x,v in plausibleClassMap.items() if plausibleUtensilTypes.intersection(v)]
        # Sanity ceck: make sure nothing is returned twice. In case we need a list of items first, then utensils,
        #     we do not just call set here.
        return sanityCheckListOfUniques(plausibleItems,plausibleUtensils)
    return []

#    \item Which objects contain something to drink?
def whichItemsContainDrinks(semanticMap, semanticReports: dict):
    # Find all plausible object classes (classes in the semantic reports) for all objects in the scene.
    plausibleClassMap = whatPlausibleDFLClassesForAllObjects(semanticMap, semanticReports)
    # Find classes of entities believed to be drinks. 
    beverages = set(sr.dl.whatSubclasses('dfl:beverage.n.wn.food'))
    # Find classes of entities believed to be containers.
    containersForDrinks = set([x for x in sr.dl.whatSubclasses('dfl:container.n.wn.artifact') if
                                  beverages.intersection(sr.dl.whatPartTypesDoesObjectHave(x))])
    # Make a list of all items in the scene that are beverages
    plausibleDrinks = [x for x,v in plausibleClassMap.items() if beverages.intersection(v)]
    # Make a list of all items that are containers which may have drinks in them
    plausibleDrinkContainers = [x for x,v in plausibleClassMap.items() if containersForDrinks.intersection(v)]
    # Sanity ceck: make sure nothing is returned twice. In case we need a list of items first, then utensils,
    #     we do not just call set here.
    return sanityCheckListOfUniques(plausibleDrinks, plausibleDrinkContainers)

#    \item Where do we expect the Spoon to be?
def whichLocationsMayHaveItem(objName: str, semanticMap, semanticReports: dict):
    # Find all plausible object classes (classes in the semantic reports) for all objects in the scene.
    plausibleClassMap = whatPlausibleDFLClassesForAllObjects(semanticMap, semanticReports)
    # Find the plausible types of objName
    itemTypes = plausibleClassMap.get(objName, [])
    # Find types of objects that could plausibly store objName
    storages = itertools.chain.from_iterable(
                   [sr.dl.whatToolsCanPerformTaskOnObject("dfl:store.v.wn.possession..place", x)
                       for x in itemTypes])
    storages = set(storages)
    # Find objects in scene that can plausibly store item
    return [x for x,v in plausibleClassMap.items() if storages.intersection(v)]

#    \item What can I grasp on the cabinet to open it?
def whichPartsOfObjectAreGraspable(objName: str, semanticMap, semanticReports: dict):
    # Find all part names of objName that are in the scene, and the object name itself, as a set
    objPartNames = getObjectPartNamesAndSelf(objName, semanticMap)
    # Find all plausible object classes for the identified object names
    #     Note: here, plausibleClassMap is not for the entire scene but only for the objPartNames subset
    plausibleClassMap = whatPlausibleDFLClassesForAllObjects(semanticMap, semanticReports, whiteList=objPartNames)
    # Get all types of objects with the graspable disposition
    graspables = set(sr.dl.whatHasDisposition("dfl:hold.v.wn.contact..grasp.Theme"))
    # Get all plausible types
    plausibleTypes = set(itertools.chain.from_iterable(plausibleClassMap.values()))
    # Get all plausible types of objects with the graspable disposition
    plausibleHandleTypes = set([x for x in plausibleTypes if x in graspables])
    # Get all object names that are plausibly handles
    return [x for x,v in plausibleClassMap.items() if plausibleHandleTypes.intersection(v)]

#    \item Where should I put the utensils for breakfast?
def whereToPlaceItemsForMeal(meal: str, semanticMap, semanticReports: dict):
    ## currently, all meals go to the same places
    #meal = meal.lower().strip()
    #if meal in {'breakfast', 'break fast', 'morning meal'}:
    if True:
        # Find all plausible object classes (classes in the semantic reports) for all objects in the scene.
        plausibleClassMap = whatPlausibleDFLClassesForAllObjects(semanticMap, semanticReports)
        # Get meal locations
        #     note: for now we hack a single disposition for all meals to be served at the same place.
        mealLocations = set(sr.dl.whatHasDisposition("dfl:serve.v.wn.consumption..concrete.Location"))
        # Get all plausible found types
        foundTypes = itertools.chain.from_iterable(plausibleClassMap.values())
        # Get all object types, from those found, which are of items where meals are served
        foundMealLocations = mealLocations.intersection(foundTypes)
        return [x for x,v in plausibleClassMap.items() if foundMealLocations.intersection(v)]
    return []

@with_simulated_robot
def move_and_detect(obj_type):
    NavigateAction(target_locations=[Pose([8, -5.35, 0],[0,0,1,0])]).resolve().perform()

    LookAtAction(targets=[pick_pose]).resolve().perform()

    object_desig = DetectAction(BelieveObject(types=[obj_type])).resolve().perform()

    return object_desig

with simulated_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([0.25]).resolve().perform()

    semanticMap = prepareSemanticMap(sceneGraphOwlFile)
    semanticReports = initializeSemanticReports(semanticMap)

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
    placing_loc = CostmapLocation(target=spoon_target_pose, reachable_for=robot_desig.resolve()).resolve()

    NavigateAction([placing_loc.pose]).resolve().perform()

    PlaceAction(spoon_desig, [spoon_target_pose], [pickup_arm]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()
