"""totem_controller_py controller."""

from controller import Robot, Camera, CameraRecognitionObject, Supervisor, Keyboard

import optparse
import math
import time


class Totem(Supervisor):
    def __init__(self):
        Supervisor.__init__(self)
        self.TIME_STEP = 32
        self.CAPACITY = 20
        self.robot = self
        self.index = 0
        self.indoorPed = []
        self.total_spawned_pedestrian = 0
        self.crossing = 0

        self.keyboard = Keyboard()

    """
    UTILITIES FUNCTIONS
    """
    def openDoor(self, door):
        door = self.getFromDef(door)
        door.getField("position").setSFFloat(-1.5)

    def closeDoor(self, door):
        door = self.getFromDef(door)
        door.getField("position").setSFFloat(0)

    def updateCustomData(self):
        totem_node = self.getFromDef("TOTEM")
        totem_node_field = totem_node.getField("customData")
        totem_node_field.setSFString(str(self.total_spawned_pedestrian))

    def canSanitize(self):
        check = self.getFromDef("CHECK")
        check_description = check.getField("description")
        check_description.setSFString("true")
        self.crossing = 0

    """
    PEDESTRIAN MANAGEMENT FUNCTIONS
    """

    def getOldestIndoorPedestrian(self):
        return self.indoorPed[0]

    def getNumberOfVisitors(self):
        return self.camera.getRecognitionNumberOfObjects() - 1

    def buildDefUseString(self, pedIndex):
        epuck_delete_target = "E" + str(pedIndex)
        return epuck_delete_target

    def changeEpuckBehaviourToLineFollowing(self, robot_def):
        epuck_node_target = self.getFromDef(robot_def)
        print(robot_def)
        epuck_node_target.getField("controller").setSFString("e-puck_line_mod")

    def incrementCrossed(self):
        self.crossing += 1
    
    def getEpuck(self, id):
        print(id)
        pedestrianCustomDataArgs = self.getNode(id).getField("customData").getSFString()
        return self.getFromDef(self.buildDefUseString(int(pedestrianCustomDataArgs)))
    
    def getNode(self, id):
        return self.getFromId(id)

    """
    PEDESTRIAN ENTER/EXIT FUNCTIONS
    """

    def aPedestrianHasLeft(self):
        if(self.getNumberOfVisitors() < len(self.indoorPed)):
            print("A pedestrian is left")
            return True
        return False

    def aPedestrianEntered(self):
        if(self.getNumberOfVisitors() > len(self.indoorPed)):
            print("A pedestrian entered")
            return True
        return False

    def registerNewPedestrian(self, id):
        self.total_spawned_pedestrian += 1
        self.indoorPed.append(id)
        #self.incrementCrossed()

    def kickOut(self):
        id = self.getOldestIndoorPedestrian()
        self.getEpuck(id).remove()
        self.incrementCrossed()
        del self.indoorPed[0]

    def passive_wait(self, sec):
        start_time = self.getTime()
        while(start_time + sec > self.getTime()):
            if(self.step(self.TIME_STEP) == -1):
                break
    
    def check_keyboard(self):
        key = self.keyboard.getKey()
        # print(key)
        if(key >= 0):
            if(key == 315):
                self.openDoor("ENTERDOOR")
            elif(key == 317):
                self.closeDoor("ENTERDOOR")
            elif(key == 316):
                self.openDoor("EXITDOOR")
            elif(key == 314):
                self.closeDoor("EXITDOOR")

    def run(self):
        #self.openDoor("ENTERDOOR")
        #self.openDoor("EXITDOOR")
        self.camera = self.robot.getDevice("camera_totem_1")
        self.camera.enable(self.TIME_STEP)
        self.camera.recognitionEnable(self.TIME_STEP)
        self.camera.enableRecognitionSegmentation()

        self.keyboard.enable(self.TIME_STEP)

        while not self.step(self.TIME_STEP) == -1:
            self.check_keyboard()
            self.recObj = self.camera.getRecognitionObjects()
            time = self.getTime()
            if(self.crossing == 4):
                #self.closeDoor("ENTERDOOR")
                self.canSanitize()
                #self.passive_wait(10)
                #self.openDoor("ENTERDOOR")

            if(self.aPedestrianHasLeft()):
                self.kickOut()

            if(self.aPedestrianEntered()):
                index = self.getNumberOfVisitors()
                id = self.recObj[index].get_id()
                self.getEpuck(id).getField("controller").setSFString("e-puck_line_mod")
     
                self.registerNewPedestrian(id)
                self.updateCustomData()


controller = Totem()
controller.run()
