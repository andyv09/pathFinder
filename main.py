from queue import PriorityQueue
import numpy as np
import pygame, time, random

class Node:
    def __init__(self, x, y, GRID_SIZE, w, h):
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
        self.GRID_SIZE = GRID_SIZE
        self.wl = w
        self.hl = h
    def draw(self, win, val=0):
        if val == 1:
            pygame.draw.rect(win, (255, 200, 200), (self.x * self.wl, self.y * self.hl, self.wl - 1, self.hl - 1))
            pygame.draw.circle(win, (0, 255, 0), (self.x * self.wl + self.wl // 2, self.y * self.hl + self.hl // 2), self.wl // 3)
        elif val == 2:
            pygame.draw.rect(win, (255, 200, 200), (self.x * self.wl, self.y * self.hl, self.wl - 1, self.hl - 1))
            pygame.draw.circle(win, (255, 0, 0), (self.x * self.wl + self.wl // 2, self.y * self.hl + self.hl // 2), self.wl // 3)
        elif val == 3:
            pygame.draw.rect(win, (222, 177, 38), (self.x * self.wl + (self.wl - 1 - self.pathSizeW)/2, self.y * self.hl + (self.hl - 1 - self.pathSizeH)/2, self.pathSizeW, self.pathSizeH))
            if self.pathSizeH < (self.hl-1):
                self.pathSizeH += (self.hl-1)*(1/self.growthDuration)
            if self.pathSizeW < (self.wl-1):
                self.pathSizeW += (self.wl-1)*(1/self.growthDuration)

        elif val == 4:
            pygame.draw.rect(win, (255, 200, 200), (self.x * self.wl, self.y * self.hl, self.wl - 1, self.hl - 1))
            pygame.draw.circle(win, (0,255,255), (self.x * self.wl + self.wl // 2, self.y * self.hl + self.hl // 2), self.wl // 3)
        elif self.wall == True:
            pygame.draw.rect(win, (255, 200, 200), (self.x * self.wl, self.y * self.hl, self.wl - 1, self.hl - 1))
            pygame.draw.rect(win, (0, 55, 250), (self.x * self.wl + (self.wl - 1 - self.pathSizeW)/2, self.y * self.hl + (self.hl - 1 - self.pathSizeH)/2, self.pathSizeW, self.pathSizeH))
            if self.pathSizeH < (self.hl-1):
                self.pathSizeH += (self.hl-1)*(1/self.growthDuration)
            if self.pathSizeW < (self.wl-1):
                self.pathSizeW += (self.wl-1)*(1/self.growthDuration)
        elif self.visited == True:
            pygame.draw.rect(win, (0, 204, 204), (self.x * self.wl, self.y * self.hl, self.wl - 1, self.hl - 1))
        else:
            pygame.draw.rect(win, (255, 200, 200), (self.x * self.wl, self.y * self.hl, self.wl - 1, self.hl - 1))
    def get_pos(self):
        return (self.x, self.y)
    def add_neighbors(self, G):
        if self.x < self.GRID_SIZE - 1:
            self.neighbors.append(G[self.x + 1][self.y])
        if self.x > 0:
            self.neighbors.append(G[self.x - 1][self.y])
        if self.y < self.GRID_SIZE - 1:
            self.neighbors.append(G[self.x][self.y + 1])
        if self.y > 0:
            self.neighbors.append(G[self.x][self.y - 1])


class Program:
    def __init__(self):
        self.algorithm = 0
        self.startAlgorithm = False
        self.size = (self.width, self.height) = 990, 990
        self.GRID_SIZE = 33
        self.GRAPH_SIZE = (self.GRID_SIZE - 1) / 2
        self.w = self.width // self.GRID_SIZE
        self.h = self.height // self.GRID_SIZE
        self.grid = []
        self.start = None
        self.end = None

        # Init GRID
        for i in range(self.GRID_SIZE):
            arr = []
            for j in range(self.GRID_SIZE):
                arr.append(Node(i, j, self.GRID_SIZE, self.w, self.h))
            self.grid.append(arr)

        # Create neighbors
        for i in range(self.GRID_SIZE):
            for j in range(self.GRID_SIZE):
                self.grid[i][j].add_neighbors(self.grid)

        self.pathFound = False
        self.queue = []
        self.visited = []
        self.stack = []
        self.path = []
        self.drawPath = []
        # ASTAR
        self.count = 0
        self.open_set = PriorityQueue()
        self.open_set_hash = []

        self.maze = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=np.uint8)
        self.win = pygame.display.set_mode(self.size)
        pygame.display.set_caption("Pathfinding Visualizer")
        self._dir_two = [
            lambda x, y: (x + 2, y),
            lambda x, y: (x - 2, y),
            lambda x, y: (x, y - 2),
            lambda x, y: (x, y + 2)
        ]
        self._dir_one = [
            lambda x, y: (x + 1, y),
            lambda x, y: (x - 1, y),
            lambda x, y: (x, y - 1),
            lambda x, y: (x, y + 1)
        ]
        self._range = list(range(4))

    def clickBox(self, pos):
        x = pos[0] // self.w
        y = pos[1] // self.h
        self.grid[x][y].wall = True

    def deleteWall(self, pos):
        x = pos[0] // self.w
        y = pos[1] // self.h
        self.grid[x][y].wall = False
        self.grid[x][y].pathSizeH = (self.h - 1) * (1 / self.grid[x][y].growthDuration)
        self.grid[x][y].pathSizeW = (self.w - 1) * (1 / self.grid[x][y].growthDuration)

    def clear(self, completeClear=True):
        self.maze = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=np.uint8)
        self.drawPath = []
        self.queue = []
        self.visited = []
        self.stack = []
        self.path = []
        self.count = 0
        self.open_set = PriorityQueue()
        self.open_set_hash = []
        self.pathFound = False
        if completeClear:
            self.grid.clear()
            for i in range(self.GRID_SIZE):
                arr = []
                for j in range(self.GRID_SIZE):
                    arr.append(Node(i, j, self.GRID_SIZE, self.w, self.h))
                self.grid.append(arr)
            # Create neighbors
            for i in range(self.GRID_SIZE):
                for j in range(self.GRID_SIZE):
                    self.grid[i][j].add_neighbors(self.grid)
            self.start = None
            self.end = None
        else:
            for i in range(self.GRID_SIZE):
                for j in range(self.GRID_SIZE):
                    self.grid[i][j].prev = None
                    self.grid[i][j].visited = False
                    self.grid[i][j].f = float('inf')
                    self.grid[i][j].g = float('inf')
                    self.grid[i][j].h = float('inf')
                    self.grid[i][j].pathSizeH = (self.h - 1) * (1 / self.grid[i][j].growthDuration)
                    self.grid[i][j].pathSizeW = (self.w - 1) * (1 / self.grid[i][j].growthDuration)
            self.start.visited = True
            self.end.prev = None
            self.start.prev = None
            self.start.visited = True
            self.start.f = self.dist(self.start.get_pos(), self.end.get_pos())
            self.start.g = 0
            self.start.h = 0
            self.stack.append(self.start)
            # ASTAR
            self.open_set.put((0, 0, self.start))
            self.open_set_hash.append(self.start)



    def shuffle(self):
        random.shuffle(self._range)
        return self._range

    def isInGrid(self, x, y):
        ret = x < 0 or y < 0 or x >= self.GRID_SIZE or y >= self.GRID_SIZE
        return ret

    def prim(self):
        frontier = []
        x = random.randint(0, self.GRID_SIZE - 2) + 1
        y = random.randint(0, self.GRID_SIZE - 2) + 1
        self.maze[x, y] = 1  # 0 - WALL, 1 - EMPTY, 2 - FRONTIER

        for direction in self._dir_two:
            tx, ty = direction(x, y)
            if not self.isInGrid(tx, ty):
                frontier.append((tx, ty))
                self.maze[tx, ty] = 2

        while frontier:
            x, y = frontier.pop(random.randint(0, len(frontier) - 1))
            for idx in self.shuffle():
                tx, ty = self._dir_two[idx](x, y)
                if not self.isInGrid(tx, ty) and self.maze[tx, ty] == 1:
                    self.maze[x, y] = self.maze[self._dir_one[idx](x, y)] = 1
                    break
            for direction in self._dir_two:
                tx, ty = direction(x, y)
                if not self.isInGrid(tx, ty) and self.maze[tx, ty] == 0:
                    frontier.append((tx, ty))
                    self.maze[tx, ty] = 2
        return True

    def DFS(self):
        if len(self.stack) > 0:
            node = self.stack.pop(0)
            # continue search
            if self.pathFound == False:
                for i in node.neighbors:
                    if i.visited == False and i.wall == False:
                        i.visited = True
                        i.prev = node
                        if i == self.end:
                            temp = i
                            while temp.prev:
                                self.path.append(temp.prev)
                                temp = temp.prev
                            print('Found end')
                            self.pathFound = True
                        self.stack.insert(0, i)

    def BFS(self):
        if len(self.stack) > 0:
            node = self.stack.pop(0)
            if node == self.end:
                temp = node
                while temp.prev:
                    self.path.append(temp.prev)
                    temp = temp.prev
                print('Found end')
                self.pathFound = True
            # continue search
            if self.pathFound == False:
                for i in node.neighbors:
                    if i.visited == False and i.wall == False:
                        i.visited = True
                        i.prev = node
                        self.stack.append(i)

    def dist(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        return abs(x1 - x2) + abs(y1 - y2)

    def ASTAR(self):
        if not self.open_set.empty():
            current = self.open_set.get()[2]
            self.open_set_hash.remove(current)

            if current == self.end:
                self.pathFound = True
                temp = current
                while temp.prev:
                    self.path.append(temp.prev)
                    temp = temp.prev

            if not self.pathFound:
                for i in current.neighbors:
                    if i.wall == False:
                        temp_g = current.g + 1
                        if temp_g < i.g:
                            i.prev = current
                            i.g = temp_g
                            i.f = temp_g + self.dist(i.get_pos(), self.end.get_pos())
                            if i not in self.open_set_hash:
                                self.count += 1
                                self.open_set.put((i.f, self.count, i))
                                self.open_set_hash.append(i)
                                i.visited = True

    def DIJKSTRA(self):
        if not self.open_set.empty():
            current = self.open_set.get()[2]
            self.open_set_hash.remove(current)
            if current == self.end:
                self.pathFound = True
                temp = current
                while temp.prev:
                    self.path.append(temp.prev)
                    temp = temp.prev
            if not self.pathFound:
                for i in current.neighbors:
                    if i.wall == False:
                        temp_g = current.g + 1
                        if temp_g < i.g:
                            i.prev = current
                            i.g = temp_g
                            i.f = temp_g
                            if i not in self.open_set_hash:
                                self.count += 1
                                self.open_set.put((i.f, self.count, i))
                                self.open_set_hash.append(i)
                                i.visited = True
    def runAlgorithm(self, startAlgorithm, algorithm):
        if startAlgorithm:
            if self.start == None or self.end == None:
                startAlgorithm = False
            elif algorithm == 0:
                self.DFS()
            elif algorithm == 1:
                self.BFS()
            elif algorithm == 2:
                self.ASTAR()
            elif algorithm == 3:
                self.DIJKSTRA()

    def sleep(self, algorithm):
        if algorithm != 1 or algorithm != 3:
            time.sleep(.01)
        if len(self.drawPath) > 0:
            time.sleep(.01)

    def draw(self):
        if len(self.path) > 0 and (len(self.stack) == 0 or len(self.open_set_hash) == 0):
            self.drawPath.append(self.path.pop(len(self.path) - 1))
        self.win.fill((0, 20, 20))
        for i in range(self.GRID_SIZE):
            for j in range(self.GRID_SIZE):
                node = self.grid[i][j]
                node.draw(self.win)
                if node == self.start:
                    node.draw(self.win, val=1)
                if node == self.end:
                    node.draw(self.win, val=2)
                if node != self.start:
                    if node in self.drawPath:
                        node.draw(self.win, val=3)
                    if node in self.stack or node in self.open_set_hash:
                        node.draw(self.win, val=4)
        pygame.display.flip()

    def events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
            if event.type == pygame.MOUSEBUTTONUP and not self.startAlgorithm:
                if event.button == 1:
                    self.clickBox(pygame.mouse.get_pos())
                if event.button == 2:
                    self.deleteWall(pygame.mouse.get_pos())
                if event.button == 3:
                    if self.start == None:
                        pos = pygame.mouse.get_pos()
                        x = pos[0] // self.w
                        y = pos[1] // self.h
                        self.start = self.grid[x][y]
                        self.start.g = 0
                        self.start.h = 0
                        # DFS / BFS
                        self.start.visited = True
                        self.stack.append(self.start)
                        # ASTAR
                        self.open_set.put((0, 0, self.start))
                        self.open_set_hash.append(self.start)
                    elif self.end == None:
                        pos = pygame.mouse.get_pos()
                        x = pos[0] // self.w
                        y = pos[1] // self.h
                        self.end = self.grid[x][y]
                        self.start.f = self.dist(self.start.get_pos(), self.end.get_pos())

            if event.type == pygame.MOUSEMOTION and not self.startAlgorithm:
                if pygame.mouse.get_pressed()[0]:
                    self.clickBox(pygame.mouse.get_pos())
            if event.type == pygame.KEYDOWN:
                # H DFS
                if event.key == 104:
                    self.startAlgorithm = True
                    self.algorithm = 0
                # J BFS
                if event.key == 106:
                    self.startAlgorithm = True
                    self.algorithm = 1
                # K A*
                if event.key == 107:
                    self.startAlgorithm = True
                    self.algorithm = 2
                # L DIJKSTRA
                if event.key == 108:
                    self.startAlgorithm = True
                    self.algorithm = 3
                if event.key == 99:
                    self.clear()
                    self.startAlgorithm = False
                if event.key == 118:
                    self.clear(completeClear=False)
                    self.startAlgorithm = False
                if event.key == 97:
                    self.clear()
                    self.prim()
                    for i in range(self.GRID_SIZE):
                        for j in range(self.GRID_SIZE):
                            if self.maze[i][j] == 1:
                                self.grid[i][j].wall = False
                            else:
                                self.grid[i][j].wall = True

    def startProgram(self):
        while True:
            self.events()
            self.runAlgorithm(self.startAlgorithm, self.algorithm)
            self.draw()
            self.sleep(self.algorithm)



program = Program()
program.startProgram()
