#!/usr/bin/python

import sys, rospy, math
from PyQt4 import QtGui, QtCore
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import ColorRGBA, Float32, Bool
from apriltags_intrude_detector.srv import apriltags_intrude
from apriltags_intrude_detector.srv import apriltags_info

## Team kappa code for RRT

class Maze:
    nodes=list()
    def __init__(self):
	print "init Maze"

    def addNode(self, node):
	self.nodes.append(node)

class Node:
    poly=None
    def __init__(self, polygon):
	self.poly=polygon
	print "init Node"

    def getPoly(self):
	return poly

class Goal(Node):
    def __init__(self, polygon):
	Node.__init__(self, polygon)
	print "init Goal"

    def getCenter(self):
	totX=0
	totY=0
	for point in self.poly.points:
	    totX+=point.x
	    totY+=point.y
	valX=totX/4
	valY=totY/4
	point=Point(valX,valY)
	return point

class Point:
    xVal=None
    yVal=None
    neighbor=None
    def __init__(self, x, y):
	self.xVal=x
	self.yVal=y
	print "init Point"

    def randomPoint(self, maxX, maxY):
	x=random.uniform(0,maxX)
	y=random.uniform(0,maxY)
	point=Point(x,y)
	return point

    def distanceToPoint(self, point):
	diffX=self.xVal-point.getX()
	diffY=self.yVal-point.getY()
	tot=diffX*diffX+diffY*diffY
	return sqrt(tot)

    def getX(self):
	return xVal

    def getY(self):
	return yVal

    def setNeighbor(self, point):#neighbor is previous point on path
	neighbor=point

    def getNeighbor(self):
	return neighbor

class Path:
    points=list()
    def __init__(self):
	print "init Path"

    def addPoint(self, point):
	self.points.append(point)

    def getPath(self):
	return points

# You implement this class
class Controller:
    stop = True # This is set to true when the stop button is pressed
    maze = None
    path = None
    goal = None

    def __init__(self):
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.trackposSub = rospy.Subscriber("tracked_pos", Pose2D, self.trackposCallback)

    def trackposCallback(self, msg):
        # This function is continuously called
        if not self.stop:
	    # print "TEAM kappa trackposCallback msg: %s"%msg
	    if self.maze != None:
	        # print "kappa field action: 0, 0"
            	twist = Twist()
            	# Change twist.linear.x to be your desired x velocity
            	twist.linear.x = 0
            	# Change twist.linear.y to be your desired y velocity
            	twist.linear.y = 0
            	twist.linear.z = 0
            	twist.angular.x = 0
            	twist.angular.y = 0
            	twist.angular.z = 0
            	self.cmdVelPub.publish(twist)

    def start(self):
	rospy.wait_for_service("apriltags_info")
        try:
	    self.maze = Maze()
	    self.path = Path()
            info_query = rospy.ServiceProxy("apriltags_info", apriltags_info)
            resp = info_query()

            for i in range(len(resp.polygons)):
                # A polygon (access points using poly.points)
                poly = resp.polygons[i]
                # The polygon's id (just an integer, 0 is goal, all else is bad)
                t_id = resp.ids[i]
		node = None
		if t_id == 0:
		    goal = Goal(poly)
		else:
		    node = Node(poly)
		self.maze.addNode(node)		
        except Exception, e:
            print "Exception: " + str(e)
        finally:
            self.stop = False

    def stop(self):
        self.stop = True

    def buildPath(self, start):#start=sphero position
	goalPoint=goal.getCenter()
	tempPath=Path()
	tempPath.addPoint(goalPoint)
	keepGoing=True
	while keepGoing:
	    point=goalPoint.randomPoint(800,600)
	    closePoint=None
	    closeDist=3000
	    for p in points:
		pDist=point.distanceToPoint(p)
	        if pDist<closeDist:
		    if checkPathClear(point,p):
		    	closePoint=p
			closeDist=pDist
	    if closePoint != None:
		point.setNeighbor(closePoint)
		tempPath.addPoint(point)
		if checkPathClear(point, start):
		    start.setNeighbor(point)
		    tempPath.addPoint(start)
		    keepGoing=False
	newPath=Path()
	tempPoint=tempPath.getPath()[0]
	while tempPoint != None:
	    newPath.addPoint(tempPoint)
	    tempPoint=tempPoint.getNeighbor()
    	self.path=newPath

    def checkPathClear(self, firstPoint, secondPoint):
	deltaX=firstPoint.getX()-secondPoint.getX()
	deltaY=firstPoint.getY()-secondPoint.getY()
	#for node in maze:
	    
	
##end of Team kappa code

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
  
