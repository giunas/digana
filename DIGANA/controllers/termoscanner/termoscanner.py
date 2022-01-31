"""totem_controller_py controller."""

from controller import Robot, Camera, CameraRecognitionObject, Supervisor, Keyboard

import optparse
import math
import time
import random


class Termoscanner(Supervisor):
    def __init__(self):
        Supervisor.__init__(self)
        self.TIME_STEP = 32
        self.CAPACITY = 20
        self.robot = self
        self.total_spawned_pedestrian = 0

        self.old = 0
        self.recObj = []

        self.keyboard = Keyboard()

    def buildDefUseString(self, pedIndex):
        epuck_delete_target = "E" + str(pedIndex)
        return epuck_delete_target

    def incrementVisitor(self):
        self.total_spawned_pedestrian += 1

    def getDefUse(self, i, x):
        def_str = "DEF E" + str(i) + " E-puck { translation "+str(
            x)+" -1.50432e-05 0.49 rotation 0.0020 0.9999 0.0020 1.5708} "
        return def_str

    def generate_color(self):
        # ROSSO: (1, 0, 0) = ALTAMENTE INFETTIVO
        # ARANCIONE: (1, 0.3, 0) = INFETTIVO
        # GIALLO: (1, 1, 0) = RAFFREDDATO MA OK
        # BIANCO: (1, 1, 1) = PERFETTO
        R = 1
        G = random.choice([0, 0.3, 1])
        B = 0
        if(G == 0 or G == 0.3):
            pass
        elif(G == 1):
            B = random.choice([0, 1])
        return [R, G, B]

    def newVisitor(self, index, x):
        root_node = self.getRoot()
        pedColor = self.generate_color()
        children_of_root = root_node.getField("children")
        children_of_root.importMFNodeFromString(-1, self.getDefUse(index, x))
        epuck_node = self.getFromDef("E"+str(index))
        turretSlot = epuck_node.getField("turretSlot")
        turretSlot.importMFNodeFromString(-1, "DEF PED" + str(index) + " Pedestrian { customData \""+str(index)+"\" scale 0.18 0.18 0.18 translation 0 0.23 0 rotation 0 1 0 3.14 skinColor " +
                                          str(pedColor[0]) + " " + str(pedColor[1]) + " " + str(pedColor[2])+"}")

        groundSensorsSlot = epuck_node.getField("groundSensorsSlot")
        groundSensorsSlot.importMFNodeFromString(-1, "E-puckGroundSensors{}")

        """
        groundSensorsSlot.importMFNodeFromString(
            -2, "DEF GROUND_CAMERA"+str(index)+" Camera { name \"ground_camera\" }")
        self.robot.getFromDef("GROUND_CAMERA"+str(index)).getField(
            "recognition").importSFNodeFromString("Recognition {}")
        """
        epuck_node.getField("controller").setSFString(
            "e-puck_line_mod")
            
    def spawnSingle(self, index, pos):
        self.newVisitor(index, pos)
        self.incrementVisitor()

    def getEpuck(self, id):
        pedestrianCustomDataArgs = self.getNode(id).getField("customData").getSFString()
        return self.getFromDef(self.buildDefUseString(int(pedestrianCustomDataArgs)))

    def kickOut(self, id):
        self.getEpuck(id).remove()
        #self.passive_wait(3)
        #self.spawnSingle(self.total_spawned_pedestrian, 4.6)

    def getNode(self, id):
        return self.getFromId(id)

    def openDoor(self, door):
        door = self.getFromDef(door)
        door.getField("position").setSFFloat(-1.5)

    def closeDoor(self, door):
        door = self.getFromDef(door)
        door.getField("position").setSFFloat(0)

    def isInfected(self, id):
        ped_node = self.getNode(id)
        skinColor = ped_node.getField("skinColor").getSFColor()
        # ROSSO: (1, 0, 0) = ALTAMENTE INFETTIVO
        rosso = [1, 0, 0]
        # ARANCIONE: (1, 0.3, 0) = INFETTIVO
        arancione = [1, 0.3, 0]
        if(skinColor == rosso or skinColor == arancione):
            return True

    def readFromTotem(self):
        totem_node = self.getFromDef("TOTEM")
        actual = int(totem_node.getField("customData").getSFString())
        if(actual != self.old):
            if(actual < self.old):
                print("XXX")
            elif(actual > self.old):
                print("Il totem interno ha rilevato un visitatore")
                self.old += 1
                return True
        #print("UGUALI")
        return False

    def passive_wait(self, sec):
        start_time = self.getTime()
        while(start_time + sec > self.getTime()):
            if(self.step(self.TIME_STEP) == -1):
                break
    
    def check_keyboard(self):
        key = self.keyboard.getKey()
        if(key == 83):
            self.spawnSingle(self.total_spawned_pedestrian, 4.60)
            self.passive_wait(3)

    def run(self):
        self.camera = self.robot.getDevice("camera_totem_2")
        self.camera.enable(self.TIME_STEP)
        self.camera.recognitionEnable(self.TIME_STEP)
        self.camera.enableRecognitionSegmentation()

        #self.spawnSingle(self.total_spawned_pedestrian, 2.10)
        #self.spawnSingle(self.total_spawned_pedestrian, 4.6)
        self.keyboard.enable(self.TIME_STEP)

        while not self.step(self.TIME_STEP) == -1:
            self.check_keyboard()
            #if(self.readFromTotem()):
                #self.spawnSingle(self.total_spawned_pedestrian, 4.6)

            self.recObj = self.camera.getRecognitionObjects()
            if(len(self.recObj) == 0):
                pass

            elif(self.recObj[0].get_model() == 'pedestrian'.encode()):
                id = self.recObj[0].get_id()
                if(self.isInfected(id)):
                    self.kickOut(id)
                #else:
                    #self.openDoor("ENTERDOOR")

controller = Termoscanner()
controller.run()
