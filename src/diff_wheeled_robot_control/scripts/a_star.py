#!/usr/bin/python
import heapq

def heuristic(start,goal):
    return abs(start[0]-goal[0])+abs(start[1]-goal[1])

def displayPathtoPrincess(n,grid):
#print all the moves here
    directions = [(-1,0),(0,-1),(0,1),(1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    dir_in_words = ["UP","LEFT","RIGHT","DOWN","RIGHT-DOWN","LEFT-DOWN","RIGHT-UP","LEFT-UP"]
    path = [];              open_list = []
    size = n;               came_from = {}
    cost = {}
    for i in range(0,n):
        for j in range(0,n):
            if(grid[i][j] == 'p'):
                princess = (i,j)
            if(grid[i][j] == 'm'):
                bot = (i,j)
    heapq.heappush(open_list,(heuristic(bot,princess),bot))
    came_from[bot] = None
    cost[bot] = 0
    while len(open_list) != 0:
        current = heapq.heappop(open_list)[1]
        if(current == princess):
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            for i in range(0,len(path)-1):
                print(dir_in_words[directions.index((path[i+1][0]-path[i][0],path[i+1][1]-path[i][1]))])
            return
        for i,j in directions:
            neighbor = (current[0]+i,current[1]+j)
            if neighbor[0] < 0 or neighbor[0] >= size or neighbor[1] < 0 or neighbor[1] >= size:
                continue
            if neighbor not in cost or cost.get(neighbor,0) > cost[current]+1:
                came_from[neighbor] = current
                cost[neighbor] = cost[current]+1
                priority = cost[neighbor] + heuristic(neighbor,princess)
                heapq.heappush(open_list,(priority,neighbor))
m = int(raw_input())
grid = []
for i in range(0, m):
    grid.append(raw_input().strip())

displayPathtoPrincess(m,grid)
