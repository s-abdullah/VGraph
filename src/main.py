#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, Twist
from std_msgs.msg import Header, ColorRGBA
from scipy.spatial import ConvexHull
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
import numpy
import math
import random
import os
# dimensions of the turtle bot
# its 36x36 and i am taking the centre to rxpand the obstackes
time =4
scale = 1
scaleA = 0.025
INFINITY = 200000;
dx = float(18/scale);
dy = float(18/scale);
theObs = "../data/world_obstacles.txt"
theGoal = "../data/goal.txt"

def growPoints(listofPoints):
	# according to calculations the four corners are 18x by 18y away from the centre
	expandedPoints= [];

	# print listofPoints
	for point in listofPoints:
		# print point[0], point[1]
		# - -
		expandedPoints.append([point[0] - dx, point[1] - dy]);
		# + -
		expandedPoints.append([point[0] + dx, point[1] - dy]);
		# + +
		expandedPoints.append([point[0] + dx, point[1] + dy]);
		# - +
		expandedPoints.append([point[0] - dx, point[1] + dy]);

	# print "expanded", expandedPoints
	return expandedPoints;

def showGrow(marker_publisher, r,g,b, thelist, idee, life):
    marker = Marker(
                # type=Marker.POINTS,
                type=Marker.LINE_STRIP,
                id=idee,
                lifetime=rospy.Duration(life), # set thi to zero for this to remain forever

                # scale=Vector3(0.05, 0.05, 0.05),
                scale=Vector3(scaleA, scaleA, scaleA),


                header=Header(frame_id='base_link'),
                # color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                color=ColorRGBA(r, g, b, 1.0),
                # text=text,
                # points=[Point(0,0,0),Point(-3,3,0),Point(9,3,0),Point(9,-3,0),Point(-3,-3,0),Point(0,0,0)],
                points=thelist,

				frame_locked=False) # set this so that it doesnt transform with the new frame
    marker_publisher.publish(marker)



def readObs(filename):
	# opening the file
	obsFile = open(filename, 'r')

	# dictionaray of obs in the file
	obstacles = {}

	# first line of the file is the number of objects
	numObs = int(obsFile.readline());

	print "The number of obstacles is: ", numObs
	# iterating over all the obstaces
	for x in range(numObs):
		# firstline of each obstacle is the number of edges
		numEdges = int(obsFile.readline());
		print "the number of edges of:", x, "is", numEdges
		# iterating ove all the edges
		corners = []
		for y in range(numEdges):

			c = obsFile.readline();
			# removing whitespace and storing the resulting words in array
			c = c.split();
			corners.append((float(c[0])/scale, float(c[1])/scale))
		# print corners
		# UPDATING THE dictionaray
		obstacles[x] = corners

	return obstacles

def main():
	rospy.init_node('my_node')
	global marker_publisher
	marker_publisher = rospy.Publisher('vgraph_markers', Marker, queue_size=700)
	x = 1
	y = 1
	z = 0
	obstacles = readObs(theObs)
	something = 100.0;
	# print "le obstacles", obstacles[0]
	lines = []
	idcounter = 11
	for key in obstacles:

		grownObs = growPoints(obstacles[key])
		hull = ConvexHull(grownObs)
		theHulInd =  hull.vertices

		final = [];
		for thing in theHulInd:
			final.append(Point(hull.points[thing][0]/something, hull.points[thing][1]/something, 0))

		obstacles[key] = final;

		final.append(final[0])

		# populating the lines
		for x in range(len(final)-1):
			temp = Line((final[x].x,final[x].y), (final[x+1].x,final[x+1].y));
			lines.append(temp)

		rospy.sleep(0.5)
		showGrow(marker_publisher, x,y,z, final, idcounter, 0)
		idcounter+=1
		# rospy.sleep(1.5)

	# print len(lines)


	# start is always 0 0 0
	start = Point(0, 0, 0)
	gFile = open(theGoal, 'r')
	# first line of the file is the number of objects
	lineG = (gFile.readline()).split();
	global goal
	goal = Point(float(lineG[0])/100, float(lineG[1])/100, 0)

	# creating new dictionary of connecting points
	obsPoints = {}
	for k1 in obstacles:
		entire = []
		for k2 in obstacles:
			if k1 != k2:
				entire = entire + obstacles[k2]
			else:
				pass

		obsPoints[k1] = entire + [goal]


	entire = []
	for k1 in obstacles:
		entire = entire + obstacles[k1]
	# obsPoints["final"] = entire
	# obstacles["final"] = [goal]

	obsPoints["start"] = entire
	obstacles["start"] = [start]

	finalDict = {}


	for key in obstacles:
		flist = obstacles[key]
		slist = obsPoints[key]
		for kpts in flist:
			finalDict[kpts] = []
			for pts in slist:
				line1 = Line((kpts.x, kpts.y), (pts.x, pts.y))
				c2 = 0
				for l in lines:
					if not (line1.intersect(l))&(l.intersect(line1)):
					# if not (l.intersect(line1)):
						c2 +=1
				# print c2
				if c2 >= len(lines):
					# showGrow(marker_publisher, 1,1,0, [kpts, pts], idcounter, time+9)
					# idcounter +=1
					finalDict[kpts].append([kpts, pts, dist(kpts, pts), INFINITY])

	for key in obstacles:
		flist = obstacles[key]
		if len(flist) > 1 :
			for x in range(len(flist)):
				if x == 0:
					finalDict[flist[x]].append([flist[x], flist[x+1], dist(flist[x], flist[x+1]), INFINITY])
					finalDict[flist[x]].append([flist[x], flist[len(flist)-1], dist(flist[x], flist[len(flist)-1]), INFINITY])


				elif x == len(flist)-1:
					finalDict[flist[x]].append([flist[x], flist[0], dist(flist[x], flist[0]), INFINITY])
					finalDict[flist[x]].append([flist[x], flist[x-1], dist(flist[x], flist[x-1]), INFINITY])

				else:
					finalDict[flist[x]].append([flist[x], flist[x+1], dist(flist[x], flist[x+1]), INFINITY])
					finalDict[flist[x]].append([flist[x], flist[x-1], dist(flist[x], flist[x-1]), INFINITY])


	# for key in finalDict:
	# 	flist = finalDict[key]
	# 	for node in flist:
	# 		revNode = [node[1], node[0], node[2], node[3]];
	# 		if finalDict.has_key(node[1]):
	# 			finalDict[node[1]].append(revNode);


	for key in finalDict:
		flist = finalDict[key]
		for node in flist:
			rospy.sleep(0.01)
			showGrow(marker_publisher, 1,1,0, [node[0], node[1]], idcounter, time)
			idcounter+=1

	# Dijkstras Algorithm
	visited = []
	theStart = [Point(0, 0, 0), start, 0, 0]
	unvisited = [theStart]

	distDict = {}
	for keys in finalDict:
		distDict[keys] = [];

	end = False
	# while not end:
	while unvisited:

		curNode = minDist(unvisited);
		rospy.sleep(0.01)
		# showGrow(marker_publisher, 0.5,y,0.5, [curNode[0], curNode[1]], idcounter, time)
		# idcounter+=1
		visited.append(curNode)
		end = distExpand(curNode, finalDict, visited, unvisited, distDict);

		# rospy.sleep(0.1)


	# tracing back path
	path = []
	theNode = traceback(finalDict, goal);
	while not (theNode[0].x == 0)&(theNode[0].y == 0)&(theNode[0].z == 0):
		showGrow(marker_publisher, 0,0,0, [theNode[1], theNode[0]], idcounter, 0)
		idcounter+=1
		path = [theNode[0], theNode[1]] + path
		the = traceback(finalDict, theNode[0])
		theNode = the
		print "getting path"
	showGrow(marker_publisher, 0,0,0, [theNode[1], theNode[0]], idcounter, 0)
	idcounter+=1
	path = [theNode[0], theNode[1]] + path

	follo = OutAndBack();
	follo.follow(path);


def traceback(theDict, ender):
	init = []
	for key in theDict:
		theList = theDict[key]
		for items in theList:
			if (items[1].x == ender.x)&(items[1].y == ender.y)&(items[1].z == ender.z):
				init.append(items);
	return minDist(init)

def getMin(theList):
	ref = ["none", "none",INFINITY, INFINITY]
	for items in theList:
		if items[3] < ref[3]:
			ref[0] = items[0]
			ref[1] = items[1]
			ref[2] = items[2]
			ref[3] = items[3]
	return ref



def dist(pt1, pt2):
	return math.sqrt((math.pow(pt1.x - pt2.x,2)) + (math.pow(pt1.y - pt2.y,2)))

# sanity check this
def dictNotEmpty(theDict):
	for keys in theDict:
		if theDict[keys]:
			return True

	return False

def distExpand(node, theDict, visit, unvisit, distance):
	sPoint = node[1]
	if (sPoint.x == goal.x)&(sPoint.y == goal.y)&(sPoint.z == goal.z):
		return False
	links = theDict[sPoint]
	for its in links:
		if its[3] > node[3] + its[2]:
			its[3] = node[3] + its[2]
			distance[its[1]] = [its[1], its[0], its[2], its[3]]

		theAppend(visit, unvisit,its);

	return False;


def theAppend(v, unv, item):
	for ites in  v:
		if (item[0].x == ites[0].x)&(item[0].y == ites[0].y)&(item[0].z == ites[0].z):
			if (item[1].x == ites[1].x)&(item[1].y == ites[1].y)&(item[1].z == ites[1].z):
				if (item[2] == ites[2])&(item[3] == ites[3]):
					return
	# rospy.sleep(0.01)
	# showGrow(marker_publisher, 0,0,0, [item[0], item[1]], random.randint(1, 100000), 3)
	unv.append(item);

def minDist(theList):
	# theList = theDict[key]

	ref = ["none", "none",INFINITY, INFINITY]
	for items in theList:
		if items[3] < ref[3]:
			ref[0] = items[0]
			ref[1] = items[1]
			ref[2] = items[2]
			ref[3] = items[3]

	# removing the item
	remove = []
	for x in range(len(theList)):
		if (theList[x][0].x == ref[0].x)&(theList[x][0].y == ref[0].y)&(theList[x][0].z == ref[0].z):
			if (theList[x][1].x == ref[1].x)&(theList[x][1].y == ref[1].y)&(theList[x][1].z == ref[1].z):
				if (theList[x][2] == ref[2])&(theList[x][3] == ref[3]):
					remove.append(x)
	# print 'remove',remove
	count = 0;
	for x in remove:
		del theList[x-count]
		count +=1


	return ref


class Line():
	def __init__(self, arg1, arg2):
		self.ep1 = arg1
		self.ep2 = arg2
		self.m = 0
		self.c = 0
		self.h = 0
		self.v = 0
		## x values are equal so vertical line
		if arg1[0] == arg2[0]:
			self.type = 'v'
			self.v = arg2[0]
		elif arg1[1] == arg2[1]:
			self.type = 'h'
			self.h = arg2[1]
		else:
			self.type = 's'
			self.m = (arg1[1]-  arg2[1])/(arg1[0] - arg2[0]);
			self.c = arg1[1] - self.m*arg1[0]



	def equality(self, lineCheck):
		if (self.ep1[0] - lineCheck.ep1[0] == 0)&(self.ep1[1] - lineCheck.ep1[1] == 0):
			return True;
		if (self.ep2[0] - lineCheck.ep1[0] == 0)&(self.ep2[1] - lineCheck.ep1[1] == 0):
			return True;

		if (self.ep1[0] - lineCheck.ep2[0] == 0)&(self.ep1[1] - lineCheck.ep2[1] == 0):
			return True;

		if (self.ep2[0] - lineCheck.ep2[0] == 0)&(self.ep2[1] - lineCheck.ep2[1] == 0):
			return True;

		return False;


	def intersect(self, lineCheck):

		# checking if two points are common or not
		if self.equality(lineCheck):
			return False
		# checking if the lines are parallel or not
		if (self.type == lineCheck.type)&(self.m == lineCheck.m):
			return False;

		# taking care of easy cases
		if (self.type == 'h'):
			if (lineCheck.type == 'v'):
				if (min(self.ep1[0],self.ep2[0]) < lineCheck.v)&(max(self.ep1[0],self.ep2[0]) > lineCheck.v):
					if (min(lineCheck.ep1[1],lineCheck.ep2[1]) < self.h)&(max(lineCheck.ep1[1],lineCheck.ep2[1]) > self.h):
						return True
					else:
						return False
				else:
					return False
			elif (lineCheck.type == 'h'):
				return False
			else:
				# getting the intersect
				tempX = float((self.h - lineCheck.c)/lineCheck.m)
				if (min(self.ep1[0],self.ep2[0]) < tempX)&(max(self.ep1[0],self.ep2[0]) > tempX):
					if (min(lineCheck.ep1[1],lineCheck.ep2[1]) < self.h)&(max(lineCheck.ep1[1],lineCheck.ep2[1]) > self.h):
						return True
					else:
						return False
				else:
					return False

		elif (self.type == 'v'):
			if (lineCheck.type == 'h'):
				if (min(self.ep1[1],self.ep2[1]) < lineCheck.h)&(max(self.ep1[1],self.ep2[1]) > lineCheck.h):
					if (min(lineCheck.ep1[0],lineCheck.ep2[0]) < self.v)&(max(lineCheck.ep1[0],lineCheck.ep2[0]) > self.v):
						return True
					else:
						return False
				else:
					return False
			elif (lineCheck.type == 'v'):
				return False
			else:
				tempY = (self.v)*lineCheck.m + lineCheck.c
				if (min(self.ep1[1],self.ep2[1]) < tempY)&(max(self.ep1[1],self.ep2[1]) > tempY):
					if (min(lineCheck.ep1[0],lineCheck.ep2[0]) < self.v)&(max(lineCheck.ep1[0],lineCheck.ep2[0]) > self.v):
						return True
					else:
						return False
				else:
					return False

		else:
			if (lineCheck.type == 'h'):
				tempX = float((lineCheck.h - self.c)/self.m)
				if (min(lineCheck.ep1[0],lineCheck.ep2[0]) < tempX)&(max(lineCheck.ep1[0],lineCheck.ep2[0]) > tempX):
					if (min(self.ep1[1],self.ep2[1]) < lineCheck.h)&(max(self.ep1[1],self.ep2[1]) > lineCheck.h):
						return True
					else:
						return False
				else:
					return False

			elif (lineCheck.type == 'v'):
				tempY = (lineCheck.v)*self.m + self.c
				if (min(lineCheck.ep1[1],lineCheck.ep2[1]) < tempY)&(max(lineCheck.ep1[1],lineCheck.ep2[1]) > tempY):
					if (min(self.ep1[0],self.ep2[0]) < lineCheck.v)&(max(self.ep1[0],self.ep2[0]) > lineCheck.v):
						return True
					else:
						return False
				else:
					return False
			else:
				tempX = (lineCheck.c - self.c)/(self.m - lineCheck.m);
				tempY = (self.m)*tempX + self.c;
				if (min(lineCheck.ep1[1],lineCheck.ep2[1]) < tempY)&(max(lineCheck.ep1[1],lineCheck.ep2[1]) > tempY):
					if(min(lineCheck.ep1[0],lineCheck.ep2[0]) < tempX)&(max(lineCheck.ep1[0],lineCheck.ep2[0]) > tempX):
						return True
					else:
						return False
				else:
					return False


class OutAndBack():
    def __init__(self):

        # Set rospy to execute a shutdown function when exiting
        rospy.on_shutdown(self.shutdown)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # How fast will we update the robot's movement?
        self.rate = 50

        # Set the equivalent ROS rate variable
        self.r = rospy.Rate(self.rate)

        # Set the forward linear speed to 0.15 meters per second
        self.linear_speed = 0.15

        # Set the rotation speed in radians per second
        self.angular_speed = 1.0

        # Set the angular tolerance in degrees converted to radians
        self.angular_tolerance = math.radians(0.001)

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        # Give tf some time to fill its buffer
        rospy.sleep(2)

        # Set the odom frame
        self.odom_frame = '/odom'

        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")

        # Initialize the position variable as a Point type
        self.position = Point()
        self.init = Point()
        # Initialize the movement command
        move_cmd = Twist()

        # Set the movement command to forward motion
        move_cmd.linear.x = self.linear_speed

        # Get the starting position values
        (self.position, self.rotation) = self.get_odom()

        self.init.x = self.position.x
        self.init.y = self.position.y
        # print "The positon is:", x_start, y_start
        self.initObject = Point();

        # Stop the robot for good
        self.stop()
        move_cmd = Twist()

        # Set the equivalent ROS rate variable
        r = rospy.Rate(self.rate)


    def follow(self, path):
	    while path:
			strt = path.pop(0)
			dest  = path.pop(0)
			(self.position, self.rotation) = self.get_odom()
			strt.x = float((math.floor(self.position.x*100))/100)
			strt.y = float((math.floor(self.position.y*100))/100)

			hyp = dist(strt, dest)
			pre = (dest.x - strt.x)
			base = (dest.y - strt.y)
			# print "position",self.position.x, self.position.y
			# print "position",strt.x, strt.y

			if (hyp == 0)|(abs(base) <= 0.01):
			    angle = 0;
			else:
			    angle = math.acos(abs(pre/hyp))

			# Set the rotation speed to 1.0 radians per second
			angular_speed = 1.0
			if base > 0:
			    self.angular_speed = 1.0
			else:
			    self.angular_speed = -1.0

			self.rotate(angle, self.angular_tolerance)
			self.stop()
			self.move(hyp);
			self.stop()
			self.rotate(-1*angle, self.angular_tolerance)
			self.stop()



    def rotate(self, angle, tolerance):
        (self.position, self.rotation) = self.get_odom()
        move_cmd = Twist()
        # Set the movement command to a rotation
        if angle > 0:
            move_cmd.angular.z = self.angular_speed
        else:
            move_cmd.angular.z = -1*self.angular_speed
            angle = -1*angle

        # Track the last angle measured
        last_angle = self.rotation

        # Track how far we have turned
        turn_angle = 0
        while abs(turn_angle + tolerance) < abs(angle) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()

            # Get the current rotation
            (self.position, self.rotation) = self.get_odom()

            # Compute the amount of rotation since the last loop
            delta_angle = normalize_angle(self.rotation - last_angle)

            # Add to the running total
            turn_angle += delta_angle
            last_angle = self.rotation


    def move(self, dist):

        # How long should it take us to get there?
        linear_duration = dist / self.linear_speed

        # Initialize the movement command
        move_cmd = Twist()

        # Set the forward speed
        move_cmd.linear.x = self.linear_speed

        # Move forward for a time to go the desired distance
        ticks = int(linear_duration * self.rate)

        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()


    def stop(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
  main()
