from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.enums import ObjectType
from pycram.plan_failures import PerceptionObjectNotFound
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
cup = Object("cup", ObjectType.JEROEN_CUP, "Cup.stl", pose=Pose([7.35, -4.95, 0.94],[0,0,1,0]), color=[1,1,0,1])
spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([7.13, -5.33, 0.75]), color=[0, 0, 1, 1])
USDPrefix = "https://ease-crc.org/ont/USD.owl#"
drawerName = "SM_Kitchen_09_Drawer_01"
USDDrawerName = USDPrefix+drawerName
apartment.attach(spoon, drawerName)

# TODO: This should be constructed automatically from something like the semantic map.
#     For now, the bullet scene construction cannot use the scene graph directly.
name2ObjectMap = {
    "https://ease-crc.org/ont/USD.owl#SM_MilkBox": milk,
    "https://ease-crc.org/ont/USD.owl#SM_SmallBowl": bowl,
    "https://ease-crc.org/ont/USD.owl#SM_CerealBox": cerial,
    "https://ease-crc.org/ont/USD.owl#SM_Spoon": spoon,
    "https://ease-crc.org/ont/USD.owl#SM_Cup": cup,
    USDDrawerName: drawer
}

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

def whichItemsToServeItemWith(oName: str, semanticMap, semanticReports: dict):
    # Find all plausible object classes (classes in the semantic reports) for all objects in the scene.
    plausibleClassMap = whatPlausibleDFLClassesForAllObjects(semanticMap, semanticReports)
    # Find all object part types (including types for the whole object)
    partTypes = set(plausibleClassMap[oName])
    partTypes = partTypes.union(itertools.chain.from_iterable([sr.dl.whatPartTypesDoesObjectHave(x) for x in partTypes]))
    # Find all tools T such that a use match (serve,T,P) exists for some part P of the object
    servingTools = set(itertools.chain.from_iterable(
                           [sr.dl.whatToolsCanPerformTaskOnObject("dfl:serve.v.wn.consumption..concrete", x) 
                               for x in partTypes]))
    return [x for x,v in plausibleClassMap.items() if servingTools.intersection(v)]

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
        plausibleUtensils = list(itertools.chain.from_iterable([whichItemsToServeItemWith(x, semanticMap, semanticReports)
                                                                   for x in plausibleItems]))
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
    # Get the robot into a standard pose.
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    MoveTorsoAction([0.25]).resolve().perform()

    # Initialize all the fancy reasoning stuff.
    semanticMap = prepareSemanticMap(sceneGraphOwlFile)
    semanticReports = initializeSemanticReports(semanticMap)

    # The robot needs to prepare a place to serve breakfast. What should it bring?
    #   the following lines retrieve object names from the semantic map
    #   CQ1, CQx: what to serve at meal, and what to serve it with 
    foodAndUtensilsForBreakfast = whichItemsForMeal("breakfast", semanticMap, semanticReports)
    #   CQ2: what is there to drink
    drinks = whichItemsContainDrinks(semanticMap, semanticReports)
    #   CQx: what to serve something with
    toServeDrinksIn = set(itertools.chain.from_iterable([whichItemsToServeItemWith(x, semanticMap, semanticReports)
                                                            for x in drinks]))
    #   associate retrieved names to bullet world objects, where possible
    namesToBring = set(foodAndUtensilsForBreakfast).union(drinks).union(toServeDrinksIn)    
    objects_to_move=[(x,name2ObjectMap[x]) for x in namesToBring if x in name2ObjectMap]

    # CQ5: where to serve the meal?
    locations = whereToPlaceItemsForMeal("breakfast", semanticMap, semanticReports)
    # We will assume we retrieved something, so just choose one of the retrieved locations
    #     (if we didn't find any location, this is a place where an error should be signalled)
    for location in locations:
        break
    # PyCRAM does not need the USD.owl prefix.
    urdf_link_name = location[len(USDPrefix)+1:]
    drop_location=iter(SemanticCostmapLocation(urdf_link_name=urdf_link_name, part_of=apartment_desig.resolve()))
    for name, item in objects_to_move:
        try:
            item_desig=move_and_detect(item.type)
            found = True
        except PerceptionObjectNotFound:
            found = False
        if found:
            drop_location_modified = next(drop_location)
            drop_location_modified.pose.position.z += .5
            while btr.pospection_contact(item, apartment, drop_location_modified.pose):
                drop_location_modified = next(drop_location)
                drop_location_modified.pose.position.z += .7
            TransportAction(item_desig,["left"],[drop_location_modified.pose]).resolve().perform()
        else:
            # CQ3: where to search for an item?
            plausibleLocations = whichLocationsMayHaveItem(name, semanticMap, semanticReports)
            # There are many drawers, and in principle the robot should search all of them.
            #   to get the plan to finish faster, we will just jump to the correct drawer -- assuming it has
            #   been returned by our reasoner.
            if USDDrawerName in plausibleLocations:
                # Finding and navigating to the drawer holding the item
                NavigateAction(target_locations=[Pose([8, -5.35, 0],[0,0,1,0])]).resolve().perform()
                # CQ4: where to grasp an object?
                whereToGrasp = whichPartsOfObjectAreGraspable(USDDrawerName, semanticMap, semanticReports)
                # The drawer itself will be returned as a graspable thing -- which it is -- but we will ignore that
                whereToGrasp = [x for x in whereToGrasp if USDDrawerName != x]
                # We assume we got a result, else we should signal an error
                #     as before, PyCRAM does not need the USD prefix, so remove it from the name
                handle_desig = ObjectPart(names=[x[len(USDPrefix)+1:] for x in whereToGrasp], part_of=apartment_desig.resolve())
                drawer_open_loc = AccessingLocation(handle_desig=handle_desig.resolve(),
                                        robot_desig=robot_desig.resolve()).resolve()
                NavigateAction([drawer_open_loc.pose]).resolve().perform()
                # OpenAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
                OpenAction(object_designator_description=handle_desig, arms=["left"]).resolve().perform()
                item.detach(apartment)
                # Detect and pickup the item
                LookAtAction([apartment.get_link_pose(whereToGrasp[0][len(USDPrefix)+1:])]).resolve().perform()
                # TODO: item.type used to be ObjectType.SPOON, but this is not general enough
                #     will this replacement work? If item is the spoon (it will be), is item.type=ObjectType.SPOON?
                item_desig = DetectAction(BelieveObject(types=[item.type])).resolve().perform()
                pickup_arm = "right"# if drawer_open_loc.arms[0] == "right" else "right"
                PickUpAction(item_desig, [pickup_arm], ["top"]).resolve().perform()
                ParkArmsAction([Arms.BOTH]).resolve().perform()
                close_loc = drawer_open_loc.pose
                # close_loc.position.y -= 0.2
                NavigateAction([close_loc]).resolve().perform()
                # CloseAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
                CloseAction(object_designator_description=handle_desig, arms=["left"]).resolve().perform()
                ParkArmsAction([Arms.BOTH]).resolve().perform()
                MoveTorsoAction([0.15]).resolve().perform()
                # Find a pose to place the item, move and then place it
                # TODO: this should be generated automatically -- is it possible to generate a pose from the location iterator?
                item_target_pose = Pose([3.42, -6.88, 0.80], [0, 0, 1, 1])
                placing_loc = CostmapLocation(target=item_target_pose, reachable_for=robot_desig.resolve()).resolve()
                NavigateAction([placing_loc.pose]).resolve().perform()
                PlaceAction(item_desig, [item_target_pose], [pickup_arm]).resolve().perform()
                ParkArmsAction([Arms.BOTH]).resolve().perform()
