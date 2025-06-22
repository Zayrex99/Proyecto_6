import os
import pygame
import random
import math
import imageio.v2 as imageio
from grafos4daa import *

class QuadTree:
    def __init__(self, boundary, capacity):
        self.boundary = boundary
        self.capacity = capacity
        self.points = []
        self.divided = False

    def subdivide(self):
        x, y, w, h = self.boundary
        nw = pygame.Rect(x, y, w / 2, h / 2)
        ne = pygame.Rect(x + w / 2, y, w / 2, h / 2)
        sw = pygame.Rect(x, y + h / 2, w / 2, h / 2)
        se = pygame.Rect(x + w / 2, y + h / 2, w / 2, h / 2)
        self.northwest = QuadTree(nw, self.capacity)
        self.northeast = QuadTree(ne, self.capacity)
        self.southwest = QuadTree(sw, self.capacity)
        self.southeast = QuadTree(se, self.capacity)
        self.divided = True

    def insert(self, point):
        if not self.boundary.collidepoint(point[0], point[1]):
            return False

        if len(self.points) < self.capacity:
            self.points.append(point)
            return True
        else:
            if not self.divided:
                self.subdivide()
            if self.northwest.insert(point):
                return True
            elif self.northeast.insert(point):
                return True
            elif self.southwest.insert(point):
                return True
            elif self.southeast.insert(point):
                return True

    def query(self, range, found):
        if not self.boundary.colliderect(range):
            return
        else:
            for p in self.points:
                if range.collidepoint(p[0], p[1]):
                    found.append(p)
            if self.divided:
                self.northwest.query(range, found)
                self.northeast.query(range, found)
                self.southwest.query(range, found)
                self.southeast.query(range, found)

def barnes_hut_layout(grafo, width, height, positions, iterations=50, theta=0.5, c=5.2):
    k = math.sqrt((width/4 * height/4) / len(grafo.nodos))
    disp = {nodo: [0, 0] for nodo in grafo.nodos}

    for _ in range(iterations):
        boundary = pygame.Rect(0, 0, width, height)
        qtree = QuadTree(boundary, 4)
        for v in positions:
            qtree.insert((positions[v][0], positions[v][1], v))

        for v in grafo.nodos:
            disp[v] = [0, 0]
            found = []
            qtree.query(pygame.Rect(positions[v][0] - width, positions[v][1] - height, 2 * width, 2 * height), found)
            for u in found:
                if v != u[2]:
                    delta = [positions[v][0] - u[0], positions[v][1] - u[1]]
                    distance = math.sqrt(delta[0]**2 + delta[1]**2)
                    if distance < 0.01:
                        distance = 0.01
                    repulsive_force = k * k / distance
                    disp[v][0] += delta[0] / distance * repulsive_force
                    disp[v][1] += delta[1] / distance * repulsive_force

        for v in grafo.aristas:
            for arista in grafo.aristas[v]:
                u = arista.destino.nombre
                delta = [positions[v][0] - positions[u][0], positions[v][1] - positions[u][1]]
                distance = math.sqrt(delta[0]**2 + delta[1]**2)
                if distance < 0.01:
                    distance = 0.01
                attractive_force = (distance * distance) / k
                disp[v][0] -= delta[0] / distance * attractive_force
                disp[v][1] -= delta[1] / distance * attractive_force
                disp[u][0] += delta[0] / distance * attractive_force
                disp[u][1] += delta[1] / distance * attractive_force

        for v in grafo.nodos:
            delta = disp[v]
            distance = math.sqrt(delta[0]**2 + delta[1]**2)
            if distance > 0:
                positions[v][0] += (delta[0] / distance) * min(c, distance)
                positions[v][1] += (delta[1] / distance) * min(c, distance)
            positions[v][0] = min(width, max(0, positions[v][0]))
            positions[v][1] = min(height, max(0, positions[v][1]))

    return positions

def draw_graph_barnes_hut(grafo, positions, width=800, height=600, delay=10, iterations_per_frame=10):
    pygame.init()
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Grafo - Barnes-Hut")
    clock = pygame.time.Clock()
    
    if not os.path.exists('frames'):
        os.makedirs('frames')

    frame_count = 0
    running = True

    def draw():
        screen.fill((255, 255, 255))
        
        for v in grafo.aristas:
            for arista in grafo.aristas[v]:
                if arista.origen.nombre in positions and arista.destino.nombre in positions:
                    pygame.draw.line(screen, (0, 0, 0), positions[arista.origen.nombre], positions[arista.destino.nombre], 1)
        
        for nodo in grafo.nodos:
            if nodo in positions:
                pygame.draw.circle(screen, (0, 0, 255), (int(positions[nodo][0]), int(positions[nodo][1])), 10)
                font = pygame.font.SysFont(None, 24)
                img = font.render(str(nodo), True, (0, 0, 0))
                screen.blit(img, (positions[nodo][0] - 10, positions[nodo][1] - 10))

        pygame.display.flip()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        positions = barnes_hut_layout(grafo, width, height, positions, iterations=iterations_per_frame)
        draw()
        pygame.time.delay(delay)

        frame_filename = f'frames/frame_{frame_count:04d}.png'
        pygame.image.save(screen, frame_filename)
        frame_count += 1

        clock.tick()

    pygame.quit()

    with imageio.get_writer('BH_ErdosRenyi500.mp4', fps=10) as writer:
        for i in range(frame_count):
            frame_filename = f'frames/frame_{i:04d}.png'
            image = imageio.imread(frame_filename)
            writer.append_data(image)

    for i in range(frame_count):
        frame_filename = f'frames/frame_{i:04d}.png'
        os.remove(frame_filename)
    os.rmdir('frames')

    print("Video guardado como 'BarnesHut.mp4'")

if __name__ == "__main__":
    grafo = grafoErdosRenyi(500, 530)
    
    width, height = 960, 800
    positions = {nodo: [random.uniform(0, width), random.uniform(0, height)] for nodo in grafo.nodos}
    draw_graph_barnes_hut(grafo, positions, width, height, delay=10, iterations_per_frame=50)
