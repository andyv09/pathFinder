from queue import PriorityQueue
import pygame, time

size = (width, height) = 960, 960
GRID_SIZE = 64
w = width//GRID_SIZE
h = height//GRID_SIZE
grid = []
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

    def draw(self, win, val=0):
        if val == 1:
            pygame.draw.rect(win, (0, 255, 0), (self.x * w, self.y * h, w - 1, h - 1))
        elif val == 2:
            pygame.draw.rect(win, (255, 0, 0), (self.x * w, self.y * h, w - 1, h - 1))
        elif val == 3:
            pygame.draw.rect(win, (222, 177, 38), (self.x * w, self.y * h, w - 1, h - 1))
        elif val == 4:
            pygame.draw.rect(win, (255, 200, 200), (self.x * w, self.y * h, w - 1, h - 1))
            pygame.draw.circle(win, (99,99,99), (self.x * w + w // 2, self.y * h + h // 2), w // 3)
        elif self.wall == True:
            pygame.draw.rect(win, (0, 255, 200), (self.x * w, self.y * h, w - 1, h - 1))
        elif self.visited == True:
            pygame.draw.rect(win, (46, 47, 48), (self.x * w, self.y * h, w - 1, h - 1))
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

queue = []
visited = []
stack = []
path = []
#ASTAR
count = 0
open_set = PriorityQueue()
open_set_hash = []

win = pygame.display.set_mode(size)
pygame.display.set_caption("Test")


def clickBox(pos):
    x = pos[0] // w
    y = pos[1] // h
    grid[x][y].wall = True

pathFound = False

def clear():
    global queue, visited, stack, path, count, open_set_hash, open_set, start, end, pathFound
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

def main():
    startAlgorithm = False
    algorithm = 0

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
            if event.type == pygame.MOUSEBUTTONUP and not startAlgorithm:
                if event.button == 1:
                    clickBox(pygame.mouse.get_pos())
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

            if event.type == pygame.MOUSEMOTION and not startAlgorithm:
                if pygame.mouse.get_pressed()[0]:
                    clickBox(pygame.mouse.get_pos())
            if event.type == pygame.KEYDOWN:
                #H DFS
                if event.key == 104:
                    startAlgorithm = True
                #J BFS
                if event.key == 106:
                    startAlgorithm = True
                    algorithm = 1
                #K A*
                if event.key == 107:
                    startAlgorithm = True
                    algorithm = 2
                #L DIJKSTRAj
                if event.key == 108:
                    startAlgorithm = True
                    algorithm = 3
                if event.key == 99:
                    clear()

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

        win.fill((0, 20, 20))
        for i in range(GRID_SIZE):
            for j in range(GRID_SIZE):
                node = grid[i][j]
                node.draw(win)
                if node == start:
                    node.draw(win, val=1)
                if node == end:
                    node.draw(win, val=2)
                if node in path and node != start:
                    node.draw(win, val=3)
        pygame.display.flip()
        if algorithm != 1 or algorithm != 3:
            time.sleep(.01)

main()
