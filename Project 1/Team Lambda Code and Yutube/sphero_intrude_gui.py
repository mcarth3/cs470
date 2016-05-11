#!/usr/bin/python

import sys, rospy, math, numpy
from PyQt4 import QtGui, QtCore
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import ColorRGBA, Float32, Bool
from apriltags_intrude_detector.srv import apriltags_intrude
from apriltags_intrude_detector.srv import apriltags_info

spheroPosition = [0.0, 0.0]

# field.x - x then y - field.y

# Base Field Class
class Field_Base:
    #r = 5.0
    #s = 5.0
    #point = (0.0,0.0)
    def __init__(self, x, y, r):
        self.point = (x, y)
        self.r = r
        self.s = 12 * self.r
    def getX(self):
        return point[0]
    def getY(self):
        return point[1]

class Obstacle(Field_Base):
    def __init__(self, x, y, r):
        Field_Base.__init__(self, x, y, r)
        self.beta = 4 #10
        self.s = 2 * self.r
    def getGradient(self):
        distance = math.sqrt((self.point[0] - spheroPosition[0])**2 + (spheroPosition[1] - self.point[1])**2)
        theta = math.atan2((self.point[1] - spheroPosition[1]), (self.point[0] - spheroPosition[0]))
        if distance < self.r:
            print 'A'
            return float("-inf") * numpy.sign(math.cos(theta)), float("-inf") * numpy.sign(math.sin(theta))
        elif (self.r <= distance and distance <= (self.s + self.r)):
            print 'B'
            return -1 * self.beta * (self.s + self.r - distance) * math.cos(theta), 1 * self.beta * (self.s + self.r - distance) * math.sin(theta)
        else:
            print 'C'
            return 0.0, 0.0


class CWTangent(Field_Base):
    def __init__(self, x, y, r):
        Field_Base.__init__(self, x, y, r)
        self.alpha2 = 2
        self.param = 50
        self.s = 20 * self.r
    def getGradient(self):
        distance = math.sqrt((self.point[0] - spheroPosition[0])**2 + (spheroPosition[1] - self.point[1])**2)
        theta = math.atan2((self.point[1] - spheroPosition[1]), (self.point[0] - spheroPosition[0]))
        if distance < self.r:
            print 'CW A'
            return float("-inf") * numpy.sign(math.cos(theta)), float("-inf") * numpy.sign(math.sin(theta))
        elif self.r <= distance and distance <= (self.s + self.r):
            x2 = -1 * (spheroPosition[0] - self.point[0])
            y2 = -1 * (spheroPosition[1] - self.point[1])
            print 'CW B'
            return self.param * y2 / (math.sqrt(x2**2 + y2**2)), self.param * x2 / (math.sqrt(x2**2 + y2**2))
        else:
            print 'CW C'
            return self.alpha2 * self.s * math.cos(theta), -1 * self.alpha2 * self.s * math.sin(theta)

class CCWTangent(Field_Base):
    def __init__(self, x, y, r):
        Field_Base.__init__(self, x, y, r)
        self.alpha2 = 2
        self.param = 50
        self.s = 20 * self.r
    def getGradient(self):
        distance = math.sqrt((self.point[0] - spheroPosition[0])**2 + (spheroPosition[1] - self.point[1])**2)
        theta = math.atan2((self.point[1] - spheroPosition[1]), (self.point[0] - spheroPosition[0]))
        if distance < self.r:
            print 'CCW A'
            return float("-inf") * numpy.sign(math.cos(theta)), float("-inf") * numpy.sign(math.sin(theta))
        elif self.r <= distance and distance <= (self.s + self.r):
            x2 = spheroPosition[0] - self.point[0]
            y2 = spheroPosition[1] - self.point[1]
            print 'CCW B'
            return self.param * y2 / (math.sqrt(x2**2 + y2**2)), self.param * x2 / (math.sqrt(x2**2 + y2**2))
        else:
            print 'CCW C'
            return self.alpha2 * self.s * math.cos(theta), -1 * self.alpha2 * self.s * math.sin(theta)


class Magnet(Field_Base):
    def __init__(self, x, y, r):
        Field_Base.__init__(self, x, y, r)
        self.beta = .15

    def getGradient(self):
        distance = math.sqrt((self.point[0] - spheroPosition[0])**2 + (spheroPosition[1] - self.point[1])**2)
        theta = math.atan2((self.point[1] - spheroPosition[1]), (self.point[0] - spheroPosition[0]))
        if distance < self.r:
            print 'A'
            return float("-inf") * numpy.sign(math.cos(theta)), float("-inf") * numpy.sign(math.sin(theta))
        elif (self.r <= distance and distance <= (self.s + self.r)):
            print 'B'
            return 1 * self.beta * (self.s + self.r - distance) * math.cos(theta), -1 * self.beta * (self.s + self.r - distance) * math.sin(theta)
        else:
            print 'C'
            return 1 * self.beta * (self.s + self.r - distance) * math.cos(theta), -1 * self.beta * (self.s + self.r - distance) * math.sin(theta)

class Goal(Field_Base):
    def __init__(self, x, y, r):
        Field_Base.__init__(self, x, y, r)
        self.alpha = .4
        self.s = 20 * self.r
    def getGradient(self):
        distance = math.sqrt((self.point[0] - spheroPosition[0])**2 + (spheroPosition[1] - self.point[1])**2)
        theta = math.atan2((self.point[1] - spheroPosition[1]), (self.point[0] - spheroPosition[0]))
        if distance < self.r:
            print 'A'
            return 0, 0;
        elif self.r <= distance and distance <= (self.s + self.r):
            print 'B'
            self.alpha += .00125
            return self.alpha * (distance - self.r) * math.cos(theta), self.alpha * (self.r - distance) * math.sin(theta)
        else:
            print 'C'
            print self.alpha * self.s * math.cos(theta) 
            print self.alpha * self.s * math.sin(theta)
            return self.alpha * self.s * math.cos(theta), self.alpha * self.s * math.sin(theta)

# You implement this class
class Controller:
    stop = True # This is set to true when the stop button is pressed
    def __init__(self):
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.trackposSub = rospy.Subscriber("tracked_pos", Pose2D, self.trackposCallback)
        self.goal = []
        self.field = []
        #self.use8 = True
        #self.oldX = -1.0
        #self.newX = -1.0
        #self.state = 1

    def trackposCallback(self, msg):
        # This function is continuously called
        if not self.stop:
            global spheroPosition
            spheroPosition[0] = msg.x
            spheroPosition[1] = msg.y
            twist = Twist()
            # Change twist.linear.x and twist.linear.y to be your desired x velocity
            # delta_x = delta_Ox + delta_Gx

            #print self.goal[0].getGradient() # goal
            #twist.linear.x, twist.linear.y = self.goal[0].getGradient()

            #print self.field[0].getGradient() # obstacle
            #twist.linear.x, twist.linear.y = self.field[0].getGradient()

            # get averages
            #if len(self.goal) > 0:
            #    self.field.append(self.goal[0])
            x = 0.0
            y = 0.0
            #if not self.use8:
            for f in self.field:
                x += f.getGradient()[0]
                y += f.getGradient()[1]
            x /= len(self.field)
            y /= len(self.field)
            print x, y
            twist.linear.x, twist.linear.y = x, y
            #else:
                #self.newX = numpy.sign(spheroPosition[0] - self.field[0].getGradient()[0])
                #print self.oldX, self.newX
                #if self.oldX != self.newX:
                #    if self.state == 4:
                #        self.state = 1
                 #   else:
                 #       self.state += 1
                #    self.oldX = self.newX
               # print 'State: ' + str(self.state)
               # if self.state == 1 or self.state == 2:
               #     print 'field A'
                 #   x, y = self.field[0].getGradient()
               # else:
               #     print 'field B'
                #    x, y = self.field[1].getGradient()
                
            #twist.linear.x, twist.linear.y = x, y

            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.cmdVelPub.publish(twist)

    def start(self):
        rospy.wait_for_service("apriltags_info")
        try:
            info_query = rospy.ServiceProxy("apriltags_info", apriltags_info)
            resp = info_query()

            for i in range(len(resp.polygons)):
                # A polygon (access points using poly.points)
                poly = resp.polygons[i]
                # The polygon's id (just an integer, 0 is goal, all else is bad)
                t_id = resp.ids[i]
                x = 0.0
                y = 0.0
                for p in poly.points:
                    x += p.x
                    y += p.y
                x /= 4
                y /= 4
                if t_id == 0: 
                    print "added goal"
                    #self.goal.append(Goal(x,y, abs((poly.points[0].x - poly.points[2].x) / 2)))
                    self.field.append(Goal(x,y, abs((poly.points[0].x - poly.points[2].x) / 2)))
                elif t_id == 2: #CW TANGENT
                    self.field.append(CWTangent(x,y, abs((poly.points[0].x - poly.points[2].x) / 2)))
                elif t_id == 3: #CCW TANGENT
                    self.field.append(CCWTangent(x,y, abs((poly.points[0].x - poly.points[2].x) / 2)))
                else:
                    print "added obstacle"
                    self.field.append(Obstacle(x, y, abs((poly.points[0].x - poly.points[2].x) / 2)))
                    #self.field.append(Magnet(x, y, abs((poly.points[0].x - poly.points[2].x) / 2)))

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
  
        
