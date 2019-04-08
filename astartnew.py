import time
import os
import cv2
from PIL import Image

from scipy import ndimage
import skimage
from skimage import data
import numpy as np
from skimage.morphology import watershed
from skimage.feature import peak_local_max
from skimage import measure
from skimage.segmentation import random_walker
from scipy import ndimage

input_file = 'lat.png'
im = cv2.imread(input_file)
cv2.imshow('DIY convolution', im)
print im
im = ndimage.distance_transform_edt(im)
print im
cv2.waitKey(0)
cv2.destroyAllWindows()

def AStar(start, goal, neighbor_nodes, distance, cost_estimate):
    def reconstruct_path(came_from, current_node):
        path = []
        while current_node is not None:
            path.append(current_node)
            current_node = came_from[current_node]
        return list(reversed(path))
    g_score = {start: 0}
    f_score = {start: g_score[start] }
    openset = {start}
    closedset = set()
    came_from = {start: None}

    while openset:
        current = min(openset, key=lambda x: f_score[x])
        if is_blocked1(current) == True:
            goal = current
            print 'Loco'
        if current == goal:
            return reconstruct_path(came_from, goal)
        openset.remove(current)
        closedset.add(current)
        for neighbor in neighbor_nodes(current):
            if neighbor in closedset:
                continue
            if neighbor not in openset:
                openset.add(neighbor)
            tentative_g_score = g_score[current] + distance(current, neighbor)
            if tentative_g_score >= g_score.get(neighbor, float('inf')):
                continue
            came_from[neighbor] = current
            g_score[neighbor] = tentative_g_score
            f_score[neighbor] = tentative_g_score
    return []

import sys
from PIL import Image
def is_blocked(p):
    x,y = p
    pixel = path_pixels[x,y]
    if any(c < 127 for c in pixel ):
        return True
def is_blocked1(p):
    x,y = p
    print x,y
    pixel = path_pixels[x,y]
    #print pixel
    if any(c == 128 for c in pixel):
        return True

def von_neumann_neighbors(p):
    x, y = p
    neighbors = [(x-1, y-1),(x-1, y), (x, y-1), (x+1, y), (x, y+1),(x-1, y+1),(x+1, y-1),(x+1, y+1)]
    #neighbors = [(x - 1, y), (x, y - 1), (x + 1, y), (x, y + 1)]
    #print (neighbors)
    return [p for p in neighbors if not is_blocked(p)]
def manhattan(p1, p2):
    return abs(p1[0]-p2[0]) + abs(p1[1]-p2[1])
def squared_euclidean(p1, p2):
    return (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2

#start = (239,18)
#goal = [(124, 295), (146, 426), (465, 321)]

start = [(237, 49),(220, 79),(257,84),(220,100)]
goal = [(250, 46)]

argv = ['lat.png', 'w_neigh.png' ]
# invoke: python mazesolver.py <mazefile> <outputfile>[.jpg|.png|etc.]
t0 = time.time()


path_img = Image.open(argv[0])

path_pixels = path_img.load()

distance = manhattan
heuristic = manhattan
for i in start:
    path = AStar(i, goal, von_neumann_neighbors, distance, heuristic)
    for position in path:
        x,y = position
        path_pixels[x,y] = (0,0,255) # red

path_img.save(argv[1])
t1 = time.time()
total = t1-t0
print ("Time elapsed:", total, "\n")
