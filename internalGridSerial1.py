from sendserial.sendserial import *

arduinoPath = "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.2:1.0"
s = SerialCommunication(arduinoPath)

printall = 0

sendStraightTicks = 4

##################  DFS  ##################
# Which is short for da finding solution
class positionStatus:
    SHOOTING_THRU_GATE = 0
    RETRIEVING_BALL = 1
    GOING_TO_NEXT_GATE = 2
    IDLE = 3

    def __init__(self, state = 3):
        self.state = state

    def toString(state):
        if state == 0:
            return 'SHOOTING BALL THROUGH THE GATE'
        elif state == 1:
            return 'RETRIEVING BALL'
        elif state == 2:
            return 'GOING TO NEXT GATE'
        else:
            return 'IDLE'
                                                                                                                                          
class Point:
    def __init__(self, x = 0, y = 0):
        self.x = x
        self.y = y
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    def __hash__(self):
        return hash((self.x, self.y))
    def __str__(self):
        return '({0}, {1})'.format(self.x, self.y)

class PathFinder:
    def __init__(self, field):
        self._field = field

    def shortest_path(self, start_point, end_point):
        self._check_range(start_point)
        self._check_range(end_point)
        if self.is_obstacle(start_point) or self.is_obstacle(end_point):
            return []
        queue = []
        visited = set()
        queue.append([start_point])
        while queue:
            path = queue.pop(0)
            current_point = path[-1]
            if current_point == end_point:
                return path

            for adjacent_point in self._adjacent_points(current_point):
                if adjacent_point not in visited:
                    visited.add(adjacent_point)
                    new_path = list(path)
                    new_path.append(adjacent_point)
                    queue.append(new_path)
        return []

    @property
    def max_y(self):
        return len(self._field)
    @property
    def max_x(self):
        return len(self._field[0])

    def is_obstacle(self, point):
        return self._field[point.y][point.x] == '0'

    def _adjacent_points(self, point):
        ''' given a point, finds all adjacent points that are not obstacles
        '''
        adjacent_points = []
        # can take a step into either directions
        for x in range(-1,2):       ## -1 <- x -> +1
            # 0 <= adj_x <= self.max_x - 1
            adj_x = min(max(point.x + x, 0), self.max_x - 1)

            for y in range (-1,2):    ## -1 <- y -> +1
                if (x == 1 and y == 1) or (x == -1 and y == -1) or (x == -1 and y == 1) or (x == 1 and y == -1):
                    continue
                # 0 <= adj_y <= self.max_y - 1
                adj_y = min(max(point.y + y, 0), self.max_y - 1)

                if adj_x == point.x and adj_y == point.y:
                    continue
                adjacent_point = Point(adj_x, adj_y)
                if self.is_obstacle(adjacent_point):
                    continue
                # all checks passed
                adjacent_points.append(adjacent_point)
        return adjacent_points

    def _check_range(self, point):
        if point.x < 0 or point.x >= self.max_x or point.y < 0 or point.y >= self.max_y:
            raise ValueError("out of defined field range")

##################  Main  ##################
import random
import time
def print_field(field, start_point, end_point, path):
    s = set(path)
    def node_repr(point):
        if path and point == start_point:
            return 'S'
        elif path and point == end_point:
            return 'E'
        elif point in s:
            return 'X'
        elif field[point.y][point.x] == '0':
            return '☐'
        else:
            return ' '
    visual_field = [[node_repr(Point(x,y)) for x in range(range_X)] for y in range(range_Y)]
    reversed_visual_field = [row for row in reversed(visual_field)]
    print()
    print('\n'.join(str(row) for row in reversed_visual_field))

def print_summary(start_point, end_point, path):
    if path:
        # print('\nShortest path from {0} to {1}): \n '.format(start_point, end_point), \
        #                                            '->'.join([s.__str__() for s in path]))
        direction = 'rotate down'
        prevX = path[0].x
        prevY = path[0].y
        robotDirections = []
        for i in path:
            if i.x == prevX and i.y == prevY:
                continue
            
            if i.y < prevY:
                if direction != 'rotate down':
                    if direction == 'rotate right':
                        s.send_right(28)
                    if direction == 'rotate left':
                        s.send_left(28)
                    if direction == 'rotate up':
                        s.send_left(56)
                    robotDirections.append('rotate down')
                    direction = 'rotate down' 
                s.send_straight(sendStraightTicks)  
                robotDirections.append('move forward')
            elif i.y > prevY:
                if direction != 'rotate up':
                    if direction == 'rotate right':
                        s.send_left(28)
                    if direction == 'rotate left':
                        s.send_right(28)
                    if direction == 'rotate down':
                        s.send_left(56)
                    robotDirections.append('rotate up')
                    direction = 'rotate up'
                s.send_straight(sendStraightTicks)
                robotDirections.append('move forward')    
            elif i.x > prevX:
                if direction != 'rotate right':
                    if direction == 'rotate down':
                        s.send_left(28)
                    if direction == 'rotate left':
                        s.send_left(56)
                    if direction == 'rotate up':
                        s.send_right(28)
                    robotDirections.append('rotate right')
                    direction = 'rotate right'
                s.send_straight(sendStraightTicks)
                robotDirections.append('move forward')
            elif i.x < prevX:
                if direction != 'rotate left':
                    if direction == 'rotate down':
                        s.send_right(28)
                    if direction == 'rotate right':
                        s.send_left(56)
                    if direction == 'rotate up':
                        s.send_left(28)
                    robotDirections.append('rotate left')
                    direction = 'rotate left'
                s.send_straight(sendStraightTicks)
                robotDirections.append('move forward')  
            prevX = i.x
            prevY = i.y
        print(robotDirections)
        s.send_stop(1)
    else:
        print('No path from {0} to {1}'.format(start_point, end_point))

def print_inverse(start_point, end_point, path):
    if path:
        # print('\nShortest path from {0} to {1}): \n '.format(start_point, end_point), \
        #                                            '->'.join([s.__str__() for s in path]))
        direction = 'rotate down'
        prevX = path[0].x
        prevY = path[0].y
        robotDirections = []

        for i in path:
            if i.x == prevX and i.y == prevY:
                continue
            
            if i.y < prevY:
                if direction != 'rotate down':
                    if direction == 'rotate right':
                        s.send_right(28)
                    if direction == 'rotate left':
                        s.send_left(28)
                    if direction == 'rotate up':
                        s.send_left(56)
                    robotDirections.append('rotate down')
                    direction = 'rotate down'
                s.send_straight(sendStraightTicks)
                robotDirections.append('move forward')
            elif i.y > prevY:
                if direction != 'rotate up':
                    if direction == 'rotate right':
                        s.send_left(28)
                    if direction == 'rotate left':
                        s.send_right(28)
                    if direction == 'rotate down':
                        s.send_left(56)
                    robotDirections.append('rotate up')
                    direction = 'rotate up'
                s.send_straight(sendStraightTicks)
                robotDirections.append('move forward')    
            elif i.x < prevX:
                if direction != 'rotate right':
                    if direction == 'rotate down':
                        s.send_left(28)
                    if direction == 'rotate left':
                        s.send_left(56)
                    if direction == 'rotate up':
                        s.send_right(28)
                    robotDirections.append('rotate right')
                    direction = 'rotate right'
                s.send_straight(sendStraightTicks)
                robotDirections.append('move forward')
            elif i.x > prevX:
                if direction != 'rotate left':
                    if direction == 'rotate down':
                        s.send_right(28)
                    if direction == 'rotate right':
                        s.send_left(56)
                    if direction == 'rotate up':
                        s.send_left(28)
                    robotDirections.append('rotate left')
                    direction = 'rotate left'
                s.send_straight(sendStraightTicks)
                robotDirections.append('move forward')  
            prevX = i.x
            prevY = i.y
        print(robotDirections)
        s.send_stop(50)
    else:
        print('No path from {0} to {1}'.format(start_point, end_point))

if __name__ == '__main__':
    
    range_X, range_Y = 24, 24
    test_field = [x[:] for x in [[' '] * range_X] * range_Y]

    #print("Here " + test_field[7][1])
    ############################ GOOD #######################################
    pathFinder = PathFinder(test_field)
    path = []

    barricadePositions = [
    18,3,
    6,3,
    6,9,
    18,9,
    15,6,
    9,6,
    12,6
    ]

    barricadePositions2 = [
    18,3,
    6,3,
    6,9,
    18,9,
    15,6,
    9,6,
    18,3,
    6,3,
    6,9,
    18,9,
    15,6,
    9,6,
    12,6]  

    startPoints = []
    endPoints = []
    i = 0
    while i < len(barricadePositions2):
        endPoints.append(barricadePositions2[i + 1])
        endPoints.append(barricadePositions2[i] - 1)
        startPoints.append(barricadePositions2[i + 1])        
        startPoints.append(barricadePositions2[i] + 1)
        i+=2
    i=0

    startingGate = int(input('What gate are we starting at?'))
    offset = (startingGate - 1) * 2

    endingGate = int(input('How many gates do we have left?'))

    start_point, end_point = Point(9,23), Point(startPoints[0 + offset], startPoints[1 + offset])
    gateX = 0
    for i in range(0, 7):
        test_field[barricadePositions[gateX]][barricadePositions[gateX+1]] = '0'
        #test_field[barricadePositions[gateX]][barricadePositions[gateX+1]-1] = '0'
        #test_field[barricadePositions[gateX]][barricadePositions[gateX+1]+1] = '0'
        gateX+=2
    
    counter = 0
    for i in range(endingGate * 2):
        path = pathFinder.shortest_path(start_point, end_point)
        if path and printall :
            print_field(test_field, start_point, end_point, path)

        # Uncheck this if we want to hug the inside of the court instead of the fastest path for gates 1 and 2
        if (counter + offset == 2 or counter + offset == 4):
            print_inverse(start_point, end_point, path)
        else: 
            print_summary(start_point, end_point, path)
        
        time.sleep(1)
        start_point = end_point
        if(i % 2 == 0):
            end_point = Point(endPoints[counter + offset], endPoints[counter + 1 + offset])
            # print(counter + offset)
            counter += 2
        else:
            end_point = Point(startPoints[counter + offset], startPoints[counter + 1 + offset])
    
    end_point = Point(6, 12) # FIX
    path = pathFinder.shortest_path(start_point, end_point)
    if path and printall :
        print_field(test_field, start_point, end_point, path)
        print_summary(start_point, end_point, path)

