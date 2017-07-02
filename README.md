
import Queue
import Tkinter as tk
import time  # sleep
import threading
from HamsterAPI.comm_usb import RobotComm
gMaxRobotNum = 1; # max number of robots to control

gRobotList = None

#CREATING CLASSES
class Node:
    def __init__(self):
      self.name = ''
      self.data =[]
      self.f_cost = 0
      #self.h_cost = 0
      self.back_pointer = False
      self.closed = False
      self.edges = []

class Graph:
    def __init__(self, canvas):
        self.nodes = {}
        self.startNode = None
        self.goalNode = None
        self.path = []
        #self.path_cost = False
        self.queue = Queue.Queue()
        #self.lifo_queue = Queue.LifoQueue()
        #self.priority_queue = Queue.PriorityQueue()
        self.canvas = canvas
        self.node_dist = 60
        self.node_size = 20
        self.edgesID = []

    def draw_node(self, a_node, n_color):
        if a_node.data: # coordinate to draw
          x = a_node.data[0]
          y = a_node.data[1]
          canvas = self.canvas
          dist = self.node_dist
          size = self.node_size
          canvas.create_oval(x*dist-size, y*dist-size, x*dist+size, y*dist+size, fill=n_color)
          canvas.create_text(x*dist, y*dist,fill="white",text=a_node.name)

    def draw_edge(self, node1, node2, e_color):
      if (node1.data and node2.data):
          x1 = node1.data[0]
          y1 = node1.data[1]
          x2 = node2.data[0]
          y2 = node2.data[1]
          dist = self.node_dist
          canvas = self.canvas
          edgeID = canvas.create_line(x1*dist, y1*dist, x2*dist, y2*dist, fill=e_color)
          self.edgesID.append(edgeID)

    def add_node(self, name, data):
        a_node = Node()
        a_node.name = name
        a_node.data = data
        self.nodes[name] = a_node
        self.draw_node(a_node, "blue")
        return a_node

    def remove_node(self, ObsCoord):
      self.draw_node(self.nodes[ObsCoord], "white")
      self.nodes[ObsCoord].closed = True
      
    def set_start(self, name):
        self.startNode = name
        self.draw_node(self.nodes[name], "red")
        self.queue.put(self.nodes[name])

    def set_goal(self, name):
        self.goalNode = name
        self.draw_node(self.nodes[name], "green")

    def add_edge(self, node1, node2, g_cost):
        self.nodes[node1].edges.append([node2, g_cost])
        self.nodes[node2].edges.append([node1, g_cost])
        self.draw_edge(self.nodes[node1], self.nodes[node2], "blue")

    #Breadth First Search
    def BFS(self): 
      self.path = []
      self.clearEdges()
      while not self.queue.empty(): #while the queue isn't empty
        current_node = self.queue.get() #get first node in queue (starts off with only node is start node)
        if (current_node.closed != True): #while the current node isn't closed
          for an_edge in current_node.edges: #iterate through all the connecting edges
            a_node_name = an_edge[0] #get the node connected to the current node
            if not self.nodes[a_node_name].closed: #has been "expanded"
              self.queue.put(self.nodes[a_node_name]) #put the connected nodes in the queue
              self.nodes[a_node_name].back_pointer = current_node #set backpointer to starter node
              if a_node_name == self.goalNode: #if reached the goal node
                path = [a_node_name] 
                path_node = self.nodes[a_node_name]
                while path_node.back_pointer != False: #iterate back through creating path
                  self.draw_edge(path_node, path_node.back_pointer, "yellow")
                  path_node = path_node.back_pointer
                  path.append(path_node.name)
                if not self.path:
                  self.path = path
                  self.path.reverse()
                  pathList = self.path
                  return pathList
        current_node.closed = True

    
    def checkIfObstacle(self, currentNode, nextNode):
      global Obstacle
      if (len(gRobotList) > 0):
        robot = gRobotList[0]
        prox_l = robot.get_proximity(0)
        prox_r = robot.get_proximity(1)
        if(prox_l > 60 or prox_r > 60):
          stop_move()
          with self.queue.mutex:
            self.queue.queue.clear()
          self.createGraph()
          self.set_start(currentNode)
          self.remove_node(nextNode)
          self.BFS()
          Obstacle = True
        else:
          Obstacle = False
          return False
      else:
        Obstacle = False
        return False 

    def clearEdges(self):
      for x in self.edgesID:
        self.canvas.delete(x)
      self.edgesID = []

    def createGraph(self):
      for n in self.nodes.values():
        n.closed = False
        n.backpointer = True
      #creating nodes
      for y in xrange(0,4):
        for x in xrange(0,5):
          coordinate = str(x) + str(y)
          self.add_node(coordinate, [x+3, y+3])

      #creating edges
      for i in xrange(0,4):
        for j in xrange(0,5):
            currentNode = str(j) + str(i)
            secondCoord = str(j+1) + str(i)
            thirdCoord = str(j) + str(i+1)
            if(j == 4 and i != 3):
              self.add_edge(currentNode, thirdCoord, 2)
            if(i == 3 and j != 4):
              self.add_edge(currentNode, secondCoord, 2)
            elif(j!=4 and i!=3):
              self.add_edge(currentNode, thirdCoord, 2)
              self.add_edge(currentNode, secondCoord, 2)

        #removing nodes
        self.remove_node("33")
        self.remove_node("32")
        self.remove_node("11")
        self.remove_node("12")

        #setting start and goal
        self.set_goal("03")




    #checks if there is an obstacle and rerouts
  
#MOVE FUNCTIONS

def behavior_p_controller():
  S_t = 80
  K_p = 1

  while not gQuit:
    low_range= 25
    high_range= 75
    if (len(gRobotList) >0):
      if gState == "P":
        robot = gRobotList[0]
        floor_l = robot.get_floor(0)
        floor_r = robot.get_floor(1)
        sensor_diff = floor_l - floor_r
        error = sensor_diff
        T_r = error * K_p
        print "floor l, floor r"
        s_l = S_t + T_r
        s_r = S_t - T_r
        robot.set_wheel(0, int(s_l))
        robot.set_wheel(1, int(s_r))
        time.sleep(0.01)
        return

      print s_l, s_r


#stop 
def stop_move(event=None):
  robot = gRobotList[1]
  robot.set_wheel(0,0)
  robot.set_wheel(1,0)

#move one grid space straight
def move_straight(MyGraph, currentNode, nextNode):
  if (len(gRobotList) > 0):
      robot = gRobotList[0]
      for robot in gRobotList:
        move_upUntilWhite()
        MyGraph.checkIfObstacle(currentNode, nextNode)
        if(Obstacle == False):
          moveAnEdge(1, 0, False)
          move_upUntilWhite()
        stop_move()
        time.sleep(1)

#turn left
def turn_left(event=None): 
    if (len(gRobotList) > 0):
      robot = gRobotList[0]
      for robot in gRobotList:
          robot.set_wheel(0,-30)
          robot.set_wheel(1,30)
    else:
        print "waiting for robot"

#move one grid space left
def move_left(MyGraph, currentNode, nextNode): 
    if (len(gRobotList) > 0):
      robot = gRobotList[0]
      for robot in gRobotList:
        move_upUntilWhite()
        turn_left()
        time.sleep(.8)
        stop_move()
        time.sleep(1)
        MyGraph.checkIfObstacle(currentNode, nextNode)
        if(Obstacle == False):
          moveAnEdge(1, 0, False)
          move_upUntilWhite()  
    else:
        print "waiting for robot"

#turn right
def turn_right():
    if (len(gRobotList) > 0):
      robot = gRobotList[0]
      for robot in gRobotList:
          robot.set_wheel(0,30)
          robot.set_wheel(1,-30)
    else:
        print "waiting for robot"

#move one grid space right
def move_right(MyGraph, currentNode, nextNode):
    if (len(gRobotList) > 0):
      robot = gRobotList[0]
      for robot in gRobotList:
        move_upUntilWhite()
        turn_right()
        time.sleep(.8)
        stop_move()
        time.sleep(1)
        MyGraph.checkIfObstacle(currentNode, nextNode)
        if(Obstacle == False):
          moveAnEdge(1, 0, False)
          move_upUntilWhite()
    else:
        print "waiting for robot"

#moves until both sensors see white
def move_upUntilWhite(event=None):
    if (len(gRobotList) > 0):
      robot = gRobotList[0]
      for robot in gRobotList:
        while not(robot.get_floor(0) > 50 or robot.get_floor(1) > 50):
          #print "MOVING UNTILL WHITE"
          robot.set_wheel(0,30)
          robot.set_wheel(1,30)
          time.sleep(.1)
    else:
        print "waiting for robot"

#line follows
def lineFollow():
    S_t = 30 # wheel speed
    T_r = 30 # turning radius - smaller will track close to curve, but oscillate 
    b2 = 20 # half distance between two wheels in mm
    if (len(gRobotList) > 0):
      robot = gRobotList[0]
      floor_l = robot.get_floor(0)
      floor_r = robot.get_floor(1)
      sensor_diff = floor_l - floor_r
      #print "LINE FOLLOWING"
      if (sensor_diff < 0): #steer left
          s_l = S_t*(T_r-b2)/(T_r+b2)
          s_r = S_t
      elif (sensor_diff > 0):
          s_r = S_t*(T_r-b2)/(T_r+b2)
          s_l = S_t
      else:
          s_l = s_r = S_t
      robot.set_wheel(0,s_l)
      robot.set_wheel(1,s_r)
      time.sleep(0.1)     

#move one edge
def moveAnEdge(numberOfGrids, numberOfGridsDone, line):
  if (len(gRobotList) > 0):
    robot = gRobotList[0]
    while(numberOfGridsDone < numberOfGrids):
      floor_l = robot.get_floor(0)
      floor_r = robot.get_floor(1)
      if(floor_l < 50 and floor_r < 50 and line == False):
          numberOfGridsDone=numberOfGridsDone+1
          line = True
      elif(floor_l > 50 or floor_r > 50):
          line = False
      if(numberOfGridsDone >= numberOfGrids):
          stop_move()
      else:
          lineFollow()
      time.sleep(.1)

#navigates grid
def moveOnGrid(pathList, MyGraph):
  global Obstacle
  x=0
  #set initial direction

  if(pathList[0][1:2]  > pathList[1][1:2]):
    Direction  = 0 #North
  elif(pathList[0][1:2] < pathList[1][1:2]):
    Direction = 2 #South
  elif(pathList[0][:1] < pathList[1][:1]):
    Direction = 1 #East
  elif(pathList[0][:1] > pathList[1][:1]):
    Direction = 3 #West

  if(len(pathList) > 0): #if there is a pathlist
    while( x< len(pathList)-1): #go through every data in path list
      print "X: ", x 
      print "OBSTACLE", Obstacle
      if(Obstacle == True):
        pathList = MyGraph.path
        print pathList
        x = 0
        Obstacle = False 

      xCoordFirst = int(pathList[x][:1])
      yCoordFirst = int(pathList[x][1:2]) 
      xCoordNext = int(pathList[x+1][:1])
      yCoordNext = int(pathList[x+1][1:2])
      #print "xCoordFirst: ", xCoordFirst, "yCoordFirst: ", yCoordFirst, "xCoordNext: ", xCoordNext, "yCoordNext: ", yCoordNext, "Direction: ", Direction
      #commands for moving
      currentNode = pathList[x]
      nextNode = pathList[x+1]
      print "CURRENT NODE: ", currentNode, "NEXT NODE: ", nextNode  
      x= x+1 
      print x 

      if(Direction == 0):
        if(xCoordFirst > xCoordNext): #move LEFT
          move_left(MyGraph, currentNode, nextNode)
          Direction  = 3
          continue
        if(xCoordFirst < xCoordNext):
          move_right(MyGraph, currentNode, nextNode)
          Direction  = 1
          continue
        if(yCoordFirst > yCoordNext): 
            move_straight(MyGraph, currentNode, nextNode)
            continue
      if(Direction == 1):
        if(xCoordFirst < xCoordNext):
          move_straight(MyGraph, currentNode, nextNode)
          continue
        if(yCoordFirst < yCoordNext):
          move_right(MyGraph, currentNode, nextNode)
          Direction  = 2
          continue
        if(yCoordFirst > yCoordNext):
          move_left(MyGraph, currentNode, nextNode)
          Direction  = 0
          continue
      if(Direction == 2):
        if(yCoordFirst < yCoordNext):
          move_straight(MyGraph, currentNode, nextNode)
          continue
        if(xCoordFirst < xCoordNext):
          move_left(MyGraph, currentNode, nextNode)
          Direction  = 1
          continue
        if(xCoordFirst > xCoordNext):
          move_right(MyGraph, currentNode, nextNode)
          Direction  = 3
          continue
      if(Direction == 3):
        if(xCoordFirst > xCoordNext):
          move_straight(MyGraph, currentNode, nextNode)
          continue
        if(yCoordFirst < yCoordNext):
          move_left(MyGraph, currentNode, nextNode)
          Direction  = 2
          continue
        if(yCoordFirst > yCoordNext):
          move_right(MyGraph, currentNode, nextNode)
          Direction  = 0
          continue
      else:
        stop_move()


    stop_move()

def P_controller(event=None):
  global gState
  gState = "P"



#GRAPH AND BUTTON FUNCTIONS




def exitProg(event=None):
    frame.quit()
    print "Exit"


#MAIN
def main():
    global gMaxRobotNum
    global frame
    global gRobotList
    global gQuit
    global Obstacle

    Obstacle = False

    frame = tk.Tk()
    gMaxRobotNum = 1
    comm = RobotComm(gMaxRobotNum)
    comm.start()

    print 'Bluetooth starts'

    gRobotList = comm.robotList
    gQuit = False

    #creating canvas
    canvas = tk.Canvas(frame, bg="white", width=500, height=500)
    canvas.pack(expand=1, fill='both')
    #Creating Graph 
    MyGraph = Graph(canvas)
    MyGraph.createGraph()

    MyGraph.set_start("43")

    
    #Getting Path
    myPathList = list(MyGraph.BFS())  

    #thread to move on the grid
    behavior_thread = threading.Thread(target=moveOnGrid, args = [MyGraph.path, MyGraph])
    behavior_thread.daemon = True
    behavior_thread.start()

    behavior_thread3 = threading.Thread(target=behavior_p_controller)
    behavior_thread3.daemon = True
    behavior_thread3.start()    

    #creating a button to exit
    button6 = tk.Button(frame,text="Exit")
    button6.pack(side='left')
    button6.bind('<Button-1>', exitProg)

    button1 = tk.Button(frame,text="P")
    button1.pack(side='left')
    button1.bind('<Button-1>', P_controller)


    #looping drawing
    frame.mainloop()

    print "Cleaning up"
    gQuit = False

    for robot in gRobotList:
      robot.reset()

    comm.stop()
    comm.join()

if __name__ == "__main__":
    main()
