from rrtBase import RRTMap 
from rrtBase import RRTGraph
import pygame
import math

import random
import math
import pygame

# [The RRTMap and RRTGraph classes remain the same as in the previous implementation]

def get_user_input():
    start_x = int(input("Enter start X coordinate: "))
    start_y = int(input("Enter start Y coordinate: "))
    goal_x = int(input("Enter goal X coordinate: "))
    goal_y = int(input("Enter goal Y coordinate: "))
    return (start_x, start_y), (goal_x, goal_y)

def add_obstacle(map, graph):
    x = int(input("Enter obstacle X coordinate: "))
    y = int(input("Enter obstacle Y coordinate: "))
    new_obs = pygame.Rect(x, y, map.obsdim, map.obsdim)
    if not new_obs.collidepoint(map.start) and not new_obs.collidepoint(map.goal):
        graph.obstacles.append(new_obs)
        pygame.draw.rect(map.map, map.grey, new_obs)
        pygame.display.update()
    else:
        print("Cannot place obstacle on start or goal. Try again.")

def main():
    dimensions = (600, 1000)
    start, goal = get_user_input()
    obsdim = 30
    obsnum = 50
    iteration = 0
    t1 = 0

    pygame.init()
    map = RRTMap(start, goal, dimensions, obsdim, obsnum)
    graph = RRTGraph(start, goal, dimensions, obsdim, obsnum)

    obstacles = graph.makeobs()
    map.drawMap(obstacles)

    running = True
    path_found = False

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    add_obstacle(map, graph)

        if not path_found:
            elapsed = pygame.time.get_ticks() - t1
            t1 = pygame.time.get_ticks()
            
            if elapsed > 10000:
                print('Timeout. Re-initiating the calculations.')
                graph = RRTGraph(start, goal, dimensions, obsdim, obsnum)
                iteration = 0

            if iteration % 10 == 0:
                X, Y, Parent = graph.bias(goal)
                pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad+2, 0)
                pygame.draw.line(map.map, map.Blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                                 map.edgeThickness)
            else:
                X, Y, Parent = graph.expand()
                pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad+2, 0)
                pygame.draw.line(map.map, map.Blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                                 map.edgeThickness)

            if iteration % 5 == 0:
                pygame.display.update()
            iteration += 1

            if graph.path_to_goal():
                path_found = True
                map.drawPath(graph.getPathCoords())
                pygame.display.update()

        pygame.display.update()

    pygame.quit()

if __name__ == '__main__':
    main()