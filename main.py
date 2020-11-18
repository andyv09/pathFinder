from queue import PriorityQueue
import numpy as np
import pygame, time, random

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.neighbors = []
        self.prev = None
        self.wall = False
        self.visited = False
        self.f = float('inf')
        self.g = float('inf')
        self.h = float('inf')
        self.growthDuration = 25
        self.pathSizeH = (h-1) * (1/self.growthDuration)
        self.pathSizeW = (w-1) * (1/self.growthDuration)
    def draw(self, win, val=0):
        if val == 1:
            pygame.draw.rect(win, (255, 200, 200), (self.x * w, self.y * h, w - 1, h - 1))
            pygame.draw.circle(win, (0, 255, 0), (self.x * w + w // 2, self.y * h + h // 2), w // 3)
        elif val == 2:
            pygame.draw.rect(win, (255, 200, 200), (self.x * w, self.y * h, w - 1, h - 1))
            pygame.draw.circle(win, (255, 0, 0), (self.x * w + w // 2, self.y * h + h // 2), w // 3)
        elif val == 3:
            pygame.draw.rect(win, (222, 177, 38), (self.x * w + (w - 1 - self.pathSizeW)/2, self.y * h + (h - 1 - self.pathSizeH)/2, self.pathSizeW, self.pathSizeH))
            if self.pathSizeH < (h-1):
                self.pathSizeH += (h-1)*(1/self.growthDuration)
            if self.pathSizeW < (w-1):
                self.pathSizeW += (w-1)*(1/self.growthDuration)

        elif val == 4:
            pygame.draw.rect(win, (255, 200, 200), (self.x * w, self.y * h, w - 1, h - 1))
            pygame.draw.circle(win, (0,255,255), (self.x * w + w // 2, self.y * h + h // 2), w // 3)
        elif self.wall == True:
            pygame.draw.rect(win, (255, 200, 200), (self.x * w, self.y * h, w - 1, h - 1))
            pygame.draw.rect(win, (0, 55, 250), (self.x * w + (w - 1 - self.pathSizeW)/2, self.y * h + (h - 1 - self.pathSizeH)/2, self.pathSizeW, self.pathSizeH))
            if self.pathSizeH < (h-1):
                self.pathSizeH += (h-1)*(1/self.growthDuration)
            if self.pathSizeW < (w-1):
                self.pathSizeW += (w-1)*(1/self.growthDuration)
        elif self.visited == True:
            pygame.draw.rect(win, (0, 204, 204), (self.x * w, self.y * h, w - 1, h - 1))
        else:
            pygame.draw.rect(win, (255, 200, 200), (self.x * w, self.y * h, w - 1, h - 1))
    def get_pos(self):
        return (self.x, self.y)
    def add_neighbors(self, G):
        if self.x < GRID_SIZE - 1:
            self.neighbors.append(G[self.x + 1][self.y])
        if self.x > 0:
            self.neighbors.append(G[self.x - 1][self.y])
        if self.y < GRID_SIZE - 1:
            self.neighbors.append(G[self.x][self.y + 1])
        if self.y > 0:
            self.neighbors.append(G[self.x][self.y - 1])

size = (width, height) = 990, 990
GRID_SIZE = 33
GRAPH_SIZE = (GRID_SIZE - 1)/2
w = width//GRID_SIZE
h = height//GRID_SIZE
grid = []
start = None
end = None

#Init GRID
for i in range(GRID_SIZE):
    arr = []
    for j in range(GRID_SIZE):
        arr.append(Node(i, j))
    grid.append(arr)

#Create neighbors
for i in range(GRID_SIZE):
    for j in range(GRID_SIZE):
        grid[i][j].add_neighbors(grid)

pathFound = False
queue = []
visited = []
stack = []
path = []
drawPath = []
#ASTAR
count = 0
open_set = PriorityQueue()
open_set_hash = []

maze = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.uint8)
win = pygame.display.set_mode(size)
pygame.display.set_caption("Pathfinding Visualizer")

def clickBox(pos):
    x = pos[0] // w
    y = pos[1] // h
    grid[x][y].wall = True

def deleteWall(pos):
    x = pos[0] // w
    y = pos[1] // h
    grid[x][y].wall = False
    grid[x][y].pathSizeH = (h - 1) * (1 / grid[x][y].growthDuration)
    grid[x][y].pathSizeW = (w - 1) * (1 / grid[x][y].growthDuration)


def clear():
    global queue, visited, stack, path, count, open_set_hash, open_set, start, end, pathFound, drawPath, maze
    maze = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.uint8)
    drawPath = []
    queue = []
    visited = []
    stack = []
    path = []
    count = 0
    open_set = PriorityQueue()
    open_set_hash = []
    pathFound = False
    # Init GRID
    grid.clear()
    for i in range(GRID_SIZE):
        arr = []
        for j in range(GRID_SIZE):
            arr.append(Node(i, j))
        grid.append(arr)
    # Create neighbors
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            grid[i][j].add_neighbors(grid)
    start = None
    end = None

def reset():
    global queue, visited, stack, path, count, open_set_hash, open_set, pathFound, end, start, drawPath
    drawPath = []
    queue = []
    visited = []
    stack = []
    path = []
    count = 0
    open_set = PriorityQueue()
    open_set_hash = []
    pathFound = False
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            grid[i][j].prev = None
            grid[i][j].visited = False
            grid[i][j].f = float('inf')
            grid[i][j].g = float('inf')
            grid[i][j].h = float('inf')
            grid[i][j].pathSizeH = (h-1) * (1/grid[i][j].growthDuration)
            grid[i][j].pathSizeW = (w-1) * (1/grid[i][j].growthDuration)
    start.visited = True
    end.prev = None
    start.prev = None
    start.visited = True
    start.f = dist(start.get_pos(), end.get_pos())
    start.g = 0
    start.h = 0
    stack.append(start)
    # ASTAR
    open_set.put((0, 0, start))
    open_set_hash.append(start)

_dir_two = [
            lambda x, y: (x + 2, y),
            lambda x, y: (x - 2, y),
            lambda x, y: (x, y - 2),
            lambda x, y: (x, y + 2)
        ]
_dir_one = [
            lambda x, y: (x + 1, y),
            lambda x, y: (x - 1, y),
            lambda x, y: (x, y - 1),
            lambda x, y: (x, y + 1)
        ]
_range  = list(range(4))
def shuffle():
    random.shuffle(_range)
    return _range

def isInGrid(x, y):
    ret = x < 0 or y < 0 or x >= GRID_SIZE or y >= GRID_SIZE
    return ret

def prim():
    frontier = []
    x = random.randint(0, GRID_SIZE - 2)+1
    y = random.randint(0, GRID_SIZE - 2)+1
    maze[x,y] = 1 #0 - WALL, 1 - EMPTY, 2 - FRONTIER

    for direction in _dir_two:
        tx,ty = direction(x, y)
        if not isInGrid(tx, ty):
            frontier.append((tx, ty))
            maze[tx, ty] = 2

    while frontier:
        x, y = frontier.pop(random.randint(0, len(frontier) - 1))
        for idx in shuffle():
            tx, ty = _dir_two[idx](x, y)
            if not isInGrid(tx, ty) and maze[tx, ty] == 1:
                maze[x, y] = maze[_dir_one[idx](x, y)] = 1
                break
        for direction in _dir_two:
            tx, ty = direction(x, y)
            if not isInGrid(tx, ty) and maze[tx, ty] == 0:
                frontier.append((tx, ty))
                maze[tx, ty] = 2
    return True



def DFS():
    global pathFound
    if len(stack) > 0:
        node = stack.pop(0)
        #continue search
        if pathFound == False:
            for i in node.neighbors:
                if i.visited == False and i.wall == False:
                    i.visited = True
                    i.prev = node
                    if i == end:
                        temp = i
                        while temp.prev:
                            path.append(temp.prev)
                            temp = temp.prev
                        print('Found end')
                        pathFound = True
                    stack.insert(0, i)

def BFS():
    global pathFound
    if len(stack) > 0:
        node = stack.pop(0)
        if node == end:
            temp = node
            while temp.prev:
                path.append(temp.prev)
                temp = temp.prev
            print('Found end')
            pathFound = True
        #continue search
        if pathFound == False:
            for i in node.neighbors:
                if i.visited == False and i.wall == False:
                    i.visited = True
                    i.prev = node
                    stack.append(i)
def dist(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)

def ASTAR():
    global count, pathFound
    if not open_set.empty():
        current = open_set.get()[2]
        open_set_hash.remove(current)

        if current == end:
            pathFound = True
            temp = current
            while temp.prev:
                path.append(temp.prev)
                temp = temp.prev

        if not pathFound:
            for i in current.neighbors:
                if i.wall == False:
                    temp_g = current.g + 1
                    if temp_g < i.g:
                        i.prev = current
                        i.g = temp_g
                        i.f = temp_g + dist(i.get_pos(), end.get_pos())
                        if i not in open_set_hash:
                            count += 1
                            open_set.put((i.f, count, i))
                            open_set_hash.append(i)
                            i.visited = True

def DIJKSTRA():
    global count, pathFound
    if not open_set.empty():
        current = open_set.get()[2]
        open_set_hash.remove(current)
        if current == end:
            pathFound = True
            temp = current
            while temp.prev:
                path.append(temp.prev)
                temp = temp.prev
        if not pathFound:
            for i in current.neighbors:
                if i.wall == False:
                    temp_g = current.g + 1
                    if temp_g < i.g:
                        i.prev = current
                        i.g = temp_g
                        i.f = temp_g
                        if i not in open_set_hash:
                            count += 1
                            open_set.put((i.f, count, i))
                            open_set_hash.append(i)
                            i.visited = True

class Program:
    def __init__(self):
        self.algorithm = 0
        self.startAlgorithm = False

    def runAlgorithm(self, startAlgorithm, algorithm):
        if startAlgorithm:
            if start == None or end == None:
                startAlgorithm = False
            elif algorithm == 0:
                DFS()
            elif algorithm == 1:
                BFS()
            elif algorithm == 2:
                ASTAR()
            elif algorithm == 3:
                DIJKSTRA()

    def sleep(self, algorithm):
        if algorithm != 1 or algorithm != 3:
            time.sleep(.01)
        if len(drawPath) > 0:
            time.sleep(.01)

    def draw(self):
        if len(path) > 0 and (len(stack) == 0 or len(open_set_hash) == 0):
            drawPath.append(path.pop(len(path) - 1))
        win.fill((0, 20, 20))
        for i in range(GRID_SIZE):
            for j in range(GRID_SIZE):
                node = grid[i][j]
                node.draw(win)
                if node == start:
                    node.draw(win, val=1)
                if node == end:
                    node.draw(win, val=2)
                if node != start:
                    if node in drawPath:
                        node.draw(win, val=3)
                    if node in stack or node in open_set_hash:
                        node.draw(win, val=4)
        pygame.display.flip()

    def mainLoop(self):
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    return
                if event.type == pygame.MOUSEBUTTONUP and not self.startAlgorithm:
                    if event.button == 1:
                        clickBox(pygame.mouse.get_pos())
                    if event.button == 2:
                        deleteWall(pygame.mouse.get_pos())
                    if event.button == 3:
                        global start, end
                        if start == None:
                            pos = pygame.mouse.get_pos()
                            x = pos[0] // w
                            y = pos[1] // h
                            start = grid[x][y]
                            start.g = 0
                            start.h = 0
                            #DFS / BFS
                            start.visited = True
                            stack.append(start)
                            #ASTAR
                            open_set.put((0, 0, start))
                            open_set_hash.append(start)
                        elif end == None:
                            pos = pygame.mouse.get_pos()
                            x = pos[0] // w
                            y = pos[1] // h
                            end = grid[x][y]
                            start.f = dist(start.get_pos(), end.get_pos())

                if event.type == pygame.MOUSEMOTION and not self.startAlgorithm:
                    if pygame.mouse.get_pressed()[0]:
                        clickBox(pygame.mouse.get_pos())
                if event.type == pygame.KEYDOWN:
                    #H DFS
                    if event.key == 104:
                        self.startAlgorithm = True
                        self.algorithm = 0
                    #J BFS
                    if event.key == 106:
                        self.startAlgorithm = True
                        self.algorithm = 1
                    #K A*
                    if event.key == 107:
                        self.startAlgorithm = True
                        self.algorithm = 2
                    #L DIJKSTRA
                    if event.key == 108:
                        self.startAlgorithm = True
                        self.algorithm = 3
                    if event.key == 99:
                        clear()
                        self.startAlgorithm = False
                    if event.key == 118:
                        reset()
                        self.startAlgorithm = False
                    if event.key == 97:

                        clear()
                        prim()
                        for i in range(GRID_SIZE):
                            for j in range(GRID_SIZE):
                                if maze[i][j] == 1:
                                    grid[i][j].wall = False
                                else:
                                    grid[i][j].wall = True

            self.runAlgorithm(self.startAlgorithm, self.algorithm)
            self.draw()
            self.sleep(self.algorithm)

program = Program()
program.mainLoop()
