import os
import pygame
import random
import math
import imageio.v2 as imageio
from grafos4daa import *

def fruchterman_reingold_layout(grafo, width, height, positions, iterations=50, k=None):
    if k is None:
        k = math.sqrt((width * height) / len(grafo.nodos))
    t = width / 10
    dt = t / iterations
    area = width * height
    for _ in range(iterations):
        disp = {nodo: [0, 0] for nodo in grafo.nodos}
        
        for v in grafo.nodos:
            for u in grafo.nodos:
                if v != u:
                    delta = [positions[v][0] - positions[u][0], positions[v][1] - positions[u][1]]
                    distance = math.sqrt(delta[0]**2 + delta[1]**2)
                    if distance < 0.01:
                        distance = 0.01
                    force = k * k / distance
                    disp[v][0] += delta[0] / distance * force
                    disp[v][1] += delta[1] / distance * force
        
        for v in grafo.aristas:
            for arista in grafo.aristas[v]:
                u = arista.destino.nombre
                delta = [positions[v][0] - positions[u][0], positions[v][1] - positions[u][1]]
                distance = math.sqrt(delta[0]**2 + delta[1]**2)
                if distance < 0.01:
                    distance = 0.01
                force = (distance * distance) / k
                disp[v][0] -= delta[0] / distance * force
                disp[v][1] -= delta[1] / distance * force
                disp[u][0] += delta[0] / distance * force
                disp[u][1] += delta[1] / distance * force

        for v in grafo.nodos:
            disp[v][0] = min(width / 2, max(-width / 2, disp[v][0]))
            disp[v][1] = min(height / 2, max(-height / 2, disp[v][1]))
            length = math.sqrt(disp[v][0]**2 + disp[v][1]**2)
            if length > 0:
                positions[v][0] += (disp[v][0] / length) * min(t, length)
                positions[v][1] += (disp[v][1] / length) * min(t, length)
            positions[v][0] = min(width, max(0, positions[v][0]))
            positions[v][1] = min(height, max(0, positions[v][1]))
        
        t -= dt
    
    return positions

def draw_graph_fruchterman_reingold(grafo, positions, width=800, height=600, delay=10, iterations_per_frame=10):
    pygame.init()
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Grafo - Fruchterman y Reingold")
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

        positions = fruchterman_reingold_layout(grafo, width, height, positions, iterations=iterations_per_frame)
        draw()
        pygame.time.delay(delay)

        frame_filename = f'frames/frame_{frame_count:04d}.png'
        pygame.image.save(screen, frame_filename)
        frame_count += 1

        clock.tick(30)

    pygame.quit()

    with imageio.get_writer('FR_Geografico500.mp4', fps=10) as writer:
        for i in range(frame_count):
            frame_filename = f'frames/frame_{i:04d}.png'
            image = imageio.imread(frame_filename)
            writer.append_data(image)

    for i in range(frame_count):
        frame_filename = f'frames/frame_{i:04d}.png'
        os.remove(frame_filename)
    os.rmdir('frames')

    print("Video guardado como 'FruchtermanReingold.mp4'")

if __name__ == "__main__":
    grafo = grafoGeografico(500, 0.01)
    
    width, height = 960, 800
    positions = {nodo: [random.uniform(0, width), random.uniform(0, height)] for nodo in grafo.nodos}
    draw_graph_fruchterman_reingold(grafo, positions, width, height, delay=10, iterations_per_frame=50)
