#!/usr/bin/python

import sys, rospy, math
from PyQt4 import QtGui, QtCore
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import ColorRGBA, Float32, Bool
from apriltags_intrude_detector.srv import apriltags_intrude
from apriltags_intrude_detector.srv import apriltags_info

## Team kappa code

class Field:
    strength = None
    cenX = None
    cenY = None
    def __init__(self, strength, cenX, cenY):
	self.strength = strength
	self.cenX = cenX
	self.cenY = cenY
	print "Field init"

    def fieldAction(self, fieldPointX, fieldPointY):
	print "Field Action"

class AttractorField(Field):
    def __init__(self, strength, cenX, cenY):
	Field.__init__(self, strength, cenX, cenY)
	print "AttractorField init"

    def fieldAction(self, fieldPointX, fieldPointY):#Stronger the further you are from the field
	distX = fieldPointX - self.cenX
	distY = fieldPointY - self.cenY
	distance = math.sqrt(distX*distX + distY*distY)
	forceX = -distX*self.strength/75
	forceY = distY*self.strength/75

	if distance > 80:
	    forceX = -self.strength*4 * distX/distance
	    forceY = self.strength*4 * distY/distance
	return (forceX,forceY)

class RepulsorField(Field):
    def __init__(self, strength, cenX, cenY):
	Field.__init__(self, strength, cenX, cenY)
	print "RepulsorField init"

    def fieldAction(self, fieldPointX, fieldPointY):#Stronger the closer you get to the field
	distX = self.cenX - fieldPointX
	distY = self.cenY - fieldPointY
	distance = math.sqrt(distX*distX + distY*distY)
	#forceX = -self.strength/distX*50
	#forceY = self.strength/distY*50
	#if abs(distX) > 75:
	#    forceX=0
	#if abs(distY) > 75:
	#    forceY=0
	forceX = -distX/distance * self.strength*30
	forceY = distY/distance * self.strength*30
	if distance > 85:
	    forceX = 0
	    forceY = 0
	return (forceX,forceY)

class WindyField(Field):
    def __init__(self, strength, cenX, cenY):
	Field.__init__(self, strength, cenX, cenY)
	print "WindyField init"

    def fieldAction(self, fieldPointX, fieldPointY):
	forceX = 0
	forceY = 0
	distX = self.cenX - fieldPointX
	distY = self.cenY - fieldPointY
	if abs(distX) < 85 and abs(distY) < 85:
	    forceY = self.strength
	return (forceX, forceY)

class Environment:
    fields = list()
    def __init__(self):
	print "Environment init"

    def addField(self, field):
	self.fields.append(field)
	print field.cenX

    def fieldAction(self, fieldPointX, fieldPointY):
	fieldSumX = 0
	fieldSumY = 0
	for field in self.fields:
	    action = field.fieldAction(fieldPointX, fieldPointY)
	    fieldSumX += action[0]
	    fieldSumY += action[1]
	return (fieldSumX, fieldSumY)

class Agent:
    def __init__(self):
	print "Agent init"

# You implement this class
class Controller:
    stop = True # This is set to true when the stop button is pressed
    agent = None
    environment = None

    def __init__(self):
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.trackposSub = rospy.Subscriber("tracked_pos", Pose2D, self.trackposCallback)

    def trackposCallback(self, msg):
        # This function is continuously called
        if not self.stop:
	    # print "TEAM kappa trackposCallback msg: %s"%msg
	    if self.environment != None:
	        fieldAction = self.environment.fieldAction(msg.x, msg.y)
	        # print "kappa field action: %f, %f"%(fieldAction[0], fieldAction[1])
            	twist = Twist()
            	# Change twist.linear.x to be your desired x velocity
            	twist.linear.x = fieldAction[0]
            	# Change twist.linear.y to be your desired y velocity
            	twist.linear.y = fieldAction[1]
            	twist.linear.z = 0
            	twist.angular.x = 0
            	twist.angular.y = 0
            	twist.angular.z = 0
            	self.cmdVelPub.publish(twist)

    def start(self):
        rospy.wait_for_service("apriltags_info")
        try:
	    self.agent = Agent()
	    self.environment = Environment()
            info_query = rospy.ServiceProxy("apriltags_info", apriltags_info)
            resp = info_query()

            for i in range(len(resp.polygons)):
                # A polygon (access points using poly.points)
                poly = resp.polygons[i]
                # The polygon's id (just an integer, 0 is goal, all else is bad)
                t_id = resp.ids[i]
		totX=0
		totY=0
		for point in poly.points:
		    totX+=point.x
		    totY+=point.y
		field = None
		if t_id == 0:
		    field = AttractorField(20, totX/4, totY/4)
		elif t_id == 1:
		    field = WindyField(-80, totX/4, totY/4)
		else:
		    field = RepulsorField(20, totX/4, totY/4)
		self.environment.addField(field)
        except Exception, e:
            print "Exception: " + str(e)
        finally:
            self.stop = False

    def stop(self):
        self.stop = True


class SpheroIntrudeForm(QtGui.QWidget):
    controller = Controller()
    
    def __init__(self):
        super(QtGui.QWidget, self).__init__()
        self.resize(600, 480) 
        self.initUI()

        rospy.init_node('sphero_intrude', anonymous=True)
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.cmdVelSub = rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback) 

        self.trackposSub = rospy.Subscriber("tracked_pos", Pose2D, self.trackposCallback) 
       
    def initUI(self):

        self.stateLabel = QtGui.QLabel("Position")
        self.stateTextbox = QtGui.QTextEdit()
        self.stateTextbox.setReadOnly(True)
        self.connect(self, QtCore.SIGNAL("sendPosIDText(PyQt_PyObject)"), self.updateStateTextbot)     
        
        key_instruct_label = """
	Control Your Sphero!
	---------------------------
	Moving around:
	   u    i    o
	   j    k    l
	   m    ,    .
	"""
        self.keyInstructLabel = QtGui.QLabel(key_instruct_label)
        self.cmdVelLabel = QtGui.QLabel("cmd_vel")
        self.cmdVelTextbox = QtGui.QTextEdit()
        self.cmdVelTextbox.setReadOnly(True)  
        self.connect(self, QtCore.SIGNAL("sendCmdVelText(PyQt_PyObject)"), self.updateCmdVelTextbox)

        self.aprilTagsInfoLabel = QtGui.QLabel("april tags info")
        self.aprilTagsInfoBtn = QtGui.QPushButton("Query")
        self.aprilTagsInfoBtn.clicked.connect(self.queryAprilTagsInfo)
        self.aprilTagsTextbox = QtGui.QTextEdit()
        self.aprilTagsTextbox.setReadOnly(True)
        self.connect(self, QtCore.SIGNAL("sendTagInfoText(PyQt_PyObject)"), self.updateAprilTagsTextbot)

        self.aprilTagsStartBtn = QtGui.QPushButton("Start")
        self.aprilTagsStartBtn.clicked.connect(self.controller.start)

        self.aprilTagsStopBtn = QtGui.QPushButton("Stop")
        self.aprilTagsStopBtn.clicked.connect(self.controller.stop)


        self.layout =  QtGui.QVBoxLayout()
        self.layout.addWidget(self.stateLabel)
        self.layout.addWidget(self.stateTextbox)
        self.layout.addWidget(self.keyInstructLabel)
        self.layout.addWidget(self.cmdVelLabel)
        self.layout.addWidget(self.cmdVelTextbox)
        hlayout = QtGui.QHBoxLayout()
        hlayout.addWidget(self.aprilTagsInfoLabel)
        hlayout.addWidget(self.aprilTagsInfoBtn)
        hlayout.addWidget(self.aprilTagsStartBtn)
        hlayout.addWidget(self.aprilTagsStopBtn)
        self.layout.addLayout(hlayout)
        self.layout.addWidget(self.aprilTagsTextbox)
        self.setLayout(self.layout)

        self.setWindowTitle("Sphero Intrude")
        self.show()

    def keyPressEvent(self, e): 
        twist = None       
        if e.key() == QtCore.Qt.Key_U:
            twist = Twist()
            twist.linear.x = -80; twist.linear.y = 80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_I:
            twist = Twist()  
            twist.linear.x = 0; twist.linear.y = 80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0     
        elif e.key() == QtCore.Qt.Key_O:
            twist = Twist()
            twist.linear.x = 80; twist.linear.y = 80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_J:
            twist = Twist()
            twist.linear.x = -80; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_K:
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_L:
            twist = Twist()
            twist.linear.x = 80; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_M:
            twist = Twist()
            twist.linear.x = -80; twist.linear.y = -80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_Comma:
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = -80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_Period:
            twist = Twist()
            twist.linear.x = 80; twist.linear.y = -80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0 
        if twist != None:
            self.cmdVelPub.publish(twist)

    def cmdVelCallback(self, msg):
        cmd_vel_text = "x=" + str(msg.linear.x) + " y=" + str(msg.linear.y)
        self.emit(QtCore.SIGNAL("sendCmdVelText(PyQt_PyObject)"), cmd_vel_text) 

    def updateCmdVelTextbox(self, value):
        self.cmdVelTextbox.moveCursor(QtGui.QTextCursor.End)
        self.cmdVelTextbox.ensureCursorVisible()
        self.cmdVelTextbox.append(str(value))
        self.cmdVelTextbox.update()

    def trackposCallback(self, msg):
        rospy.wait_for_service("apriltags_intrude")
        try:
            intrude_query = rospy.ServiceProxy("apriltags_intrude", apriltags_intrude)
            resp = intrude_query(int(msg.x), int(msg.y))
            pos_id_text = "["+str(int(msg.x))+"," +str(int(msg.y))+"]" + "(" + str(resp.id) + ")"
            self.emit(QtCore.SIGNAL("sendPosIDText(PyQt_PyObject)"), pos_id_text)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def updateStateTextbot(self, value):
        self.stateTextbox.moveCursor(QtGui.QTextCursor.End)
        self.stateTextbox.ensureCursorVisible()
        self.stateTextbox.append(str(value))
        self.stateTextbox.update()

    def queryAprilTagsInfo(self):
        #print "clicked"
        rospy.wait_for_service("apriltags_info")
        try:
            info_query = rospy.ServiceProxy("apriltags_info", apriltags_info)
            resp = info_query()
               
            #print str(resp)

            info_text = "" 
            for i in range(len(resp.polygons)):
                poly = resp.polygons[i]
                t_id = resp.ids[i]

                #print(str(poly))
                #print(str(t_id))
                info_text += "["+str(t_id)+"] "
                for p in poly.points:
                    info_text += "(" + str(int(p.x)) + "," + str(int(p.y)) + ")"
                info_text += "\n" 

            self.emit(QtCore.SIGNAL("sendTagInfoText(PyQt_PyObject)"), info_text)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
    def updateAprilTagsTextbot(self, value):
        self.aprilTagsTextbox.clear()
        self.aprilTagsTextbox.moveCursor(QtGui.QTextCursor.End)
        self.aprilTagsTextbox.ensureCursorVisible()
        self.aprilTagsTextbox.append(str(value))
        self.aprilTagsTextbox.update()        

if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)
    w = SpheroIntrudeForm()
    w.show()
    sys.exit(app.exec_())
  
