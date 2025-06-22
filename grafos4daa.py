import random

class Nodo:
    def __init__(self, nombre):
        self.nombre = nombre

    def __repr__(self):
        return str(self.nombre)

class Arista:
    def __init__(self, origen, destino, peso=1):
        self.origen = origen
        self.destino = destino
        self.peso = peso

class Grafo:
    def __init__(self, dirigido=False):
        self.nodos = {}
        self.aristas = {}
        self.dirigido = dirigido

    def agregar_nodo(self, nodo):
        self.nodos[nodo.nombre] = nodo

    def agregar_arista(self, arista):
        if arista.origen.nombre not in self.nodos or arista.destino.nombre not in self.nodos:
            raise ValueError("Los nodos de la arista deben estar en el grafo.")
        if arista.origen.nombre not in self.aristas:
            self.aristas[arista.origen.nombre] = []
        self.aristas[arista.origen.nombre].append(arista)
        if not self.dirigido:
            reversed_arista = Arista(arista.destino, arista.origen, arista.peso)
            #reversed_arista = Arista(arista.destino, arista.origen, random.randint(10, 100))
            if arista.destino.nombre not in self.aristas:
                self.aristas[arista.destino.nombre] = []
            self.aristas[arista.destino.nombre].append(reversed_arista)


    

    def KruskalD(self):
        def encontrar(padre, i):
            if padre[i] == i:
                return i
            return encontrar(padre, padre[i])

        def union(padre, rango, x, y):
            raiz_x = encontrar(padre, x)
            raiz_y = encontrar(padre, y)
            if rango[raiz_x] < rango[raiz_y]:
                padre[raiz_x] = raiz_y
            elif rango[raiz_x] > rango[raiz_y]:
                padre[raiz_y] = raiz_x
            else:
                padre[raiz_y] = raiz_x
                rango[raiz_x] += 1


        # Paso 1: Ordenar todas las aristas en orden no decreciente de su peso
        aristas = []
        for origen in self.aristas:
            for arista in self.aristas[origen]:
                aristas.append(arista)
        
        aristas = sorted(aristas, key=lambda item: item.peso)

        padre = {}
        rango = {}

        # Crear subconjuntos con elementos individuales
        for nodo in self.nodos.values():
            padre[nodo.nombre] = nodo.nombre
            rango[nodo.nombre] = 0

        mst = []
        total_peso = 0
        for arista in aristas:
            u = arista.origen.nombre
            v = arista.destino.nombre
            set_u = encontrar(padre, u)
            set_v = encontrar(padre, v)

            if set_u != set_v:
                mst.append(arista)
                total_peso += arista.peso
                union(padre, rango, set_u, set_v)

        return mst, total_peso

    def KruskalI(self):
        def find(parent, i):
            if parent[i] == i:
                return i
            return find(parent, parent[i])

        def union(parent, rank, x, y):
            root_x = find(parent, x)
            root_y = find(parent, y)
            if rank[root_x] < rank[root_y]:
                parent[root_x] = root_y
            elif rank[root_x] > rank[root_y]:
                parent[root_y] = root_x
            else:
                parent[root_y] = root_x
                rank[root_x] += 1

        # Inicializar todas las aristas
        aristas = []
        for origen in self.aristas:
            for arista in self.aristas[origen]:
                aristas.append(arista)

        # Ordenar todas las aristas en orden no decreciente de su peso
        #aristas = sorted(aristas, key=lambda item: item.peso, reverse=True)
        aristas = sorted(aristas, key=lambda item: item.peso)

        parent = {}
        rank = {}

        for nodo in self.nodos.values():
            parent[nodo.nombre] = nodo.nombre
            rank[nodo.nombre] = 0

        mst = aristas.copy()
        total_peso = sum(arista.peso for arista in mst)
        for arista in aristas:
            u = arista.origen.nombre
            v = arista.destino.nombre
            set_u = find(parent, u)
            set_v = find(parent, v)

            if set_u != set_v:
                union(parent, rank, set_u, set_v)
            else:
                mst.remove(arista)
                total_peso -= arista.peso

        return mst, total_peso

    def Prim(self):
        from queue import PriorityQueue

        if not self.nodos:
            return []

        start = next(iter(self.nodos))  # tomar un nodo inicial de manera aleatoria 
        visitados = set()
        pq = PriorityQueue()
        mst = []
        total_peso = 0 

        def add_edges(nodo):
            visitados.add(nodo)
            for arista in self.aristas.get(nodo, []):
                if arista.destino.nombre not in visitados:
                    pq.put((arista.peso, nodo, arista.destino.nombre))

        add_edges(start)
        while not pq.empty() and len(visitados) < len(self.nodos):
            peso, origen, destino = pq.get()
            if destino not in visitados:
                mst.append(Arista(self.nodos[origen], self.nodos[destino], peso))
                total_peso += peso
                add_edges(destino)

        return mst, total_peso

    def guardar_grafo_mst(self, nombre_archivo, mst):
        with open(nombre_archivo, 'w') as f:
            f.write("graph {\n")
            for arista in mst:
                f.write(f"  {arista.origen.nombre} -- {arista.destino.nombre} [label=\"{arista.peso}\"];\n")
            f.write("}")

    def BFS(self, s):
        visitados = {s: None}
        nivel = {s: 0}
        i = 1
        frontera = [s]
        while frontera:
            siguiente = []
            for u in frontera:
                for v in self.aristas[u]:
                    if v.destino.nombre not in visitados:
                        visitados[v.destino.nombre] = u
                        nivel[v.destino.nombre] = i
                        siguiente.append(v.destino.nombre)
            frontera = siguiente
            i += 1
        return visitados, nivel
    
    
    def guardar_grafo(self, nombre_archivo):
        with open(nombre_archivo, 'w') as f:
            f.write("graph {\n")
            for u in self.aristas:
                for v in self.aristas[u]:
                    f.write(f"  {u} -- {v.destino.nombre};\n")
            f.write("}")

    def guardar_grafo2(self, nombre_archivo):
        with open(nombre_archivo, 'w') as f:
            f.write("graph {\n")
            for u in self.aristas:
                for v in self.aristas[u]:
                    peso_arista = v.peso
                    f.write(f"  {u} -- {v.destino.nombre} [label=\"{peso_arista}\"];\n")
            f.write("}")

   
    def generar_arbol_bfs_gv(self, s, nombre_archivo):
        visitados, nivel = self.BFS(s)
        with open(nombre_archivo, 'w') as f:
            f.write("digraph {\n")
            for nodo, padre in visitados.items():
                if padre is not None:
                    f.write(f"  {padre} -> {nodo};\n")
            f.write("}")
    
    def generar_arbol_dfsr_gv(self, s, nombre_archivo):
        visitados = self.DFS_R(s)
        with open(nombre_archivo, 'w') as f:
            f.write("digraph {\n")
            for nodo in self.nodos.values():
                f.write(f"  {nodo.nombre};\n")
            for i in range(1, len(visitados)):
                f.write(f"  {visitados[i-1]} -> {visitados[i]};\n")
            f.write("}")
    
    def generar_arbol_dfsi_gv(self, s, nombre_archivo):
        visitados = self.DFS_I(s)
        with open(nombre_archivo, 'w') as f:
            f.write("digraph {\n")
            for nodo in self.nodos.values():
                f.write(f"  {nodo.nombre};\n")
            for i in range(1, len(visitados)):
                f.write(f"  {visitados[i-1]} -> {visitados[i]};\n")
            f.write("}")

    def Dijkstra(self, s):
        # Inicializar las distancias desde el nodo fuente
        distancias = {nodo.nombre: float('inf') for nodo in self.nodos.values()}
        distancias[s] = 0

        # Conjunto de nodos visitados
        visitados = set()

        # Aristas del árbol de caminos más cortos
        aristas_arbol = {}

        while len(visitados) < len(self.nodos):
            # Encontrar el nodo no visitado con la distancia más corta
            nodo_actual = min((nodo for nodo in distancias.items() if nodo[0] not in visitados), key=lambda x: x[1])[0]

            # Marcar el nodo como visitado
            visitados.add(nodo_actual)

            # Actualizar las distancias a los vecinos del nodo actual
            for arista in self.aristas.get(nodo_actual, []):
                vecino = arista.destino.nombre
                peso_arista = arista.peso
                nueva_distancia = distancias[nodo_actual] + peso_arista
                if nueva_distancia < distancias.get(vecino, float('inf')):
                    distancias[vecino] = nueva_distancia
                    aristas_arbol[vecino] = (nodo_actual, nueva_distancia)

        return distancias, aristas_arbol

    def guardar_grafo_dijkstra(self, s, nombre_archivo):
        distancias, aristas_arbol = self.Dijkstra(s)

        with open(nombre_archivo, 'w') as f:
            f.write("graph {\n")
            for nodo, (origen, distancia) in aristas_arbol.items():
                peso_arista = distancias[nodo] - distancias[origen]  # Calcular el peso de la arista
                f.write(f"  {origen} -- {nodo} [label=\"nodo_{nodo} ({peso_arista})\"];\n")
            f.write("}")

    

#>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Fin de clase GRAFO <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


def grafoMalla(m, n, dirigido=False):
    grafo = Grafo(dirigido)
    # Crear nodos
    for i in range(m):
        for j in range(n):
            nombre_nodo = f"{i}-{j}"
            nodo = Nodo(nombre_nodo)
            grafo.agregar_nodo(nodo)
    # Agregar aristas
    for i in range(m):
        for j in range(n):
            if i < m - 1:
                arista_v = Arista(grafo.nodos[f"{i}-{j}"], grafo.nodos[f"{i+1}-{j}"], random.randint(1, 20))
                grafo.agregar_arista(arista_v)
            if j < n - 1:
                arista_h = Arista(grafo.nodos[f"{i}-{j}"], grafo.nodos[f"{i}-{j+1}"], random.randint(1, 20))
                grafo.agregar_arista(arista_h)
    return grafo

def grafoErdosRenyi(n, m, dirigido=False):
    if m < n - 1:
        raise ValueError("El número de aristas debe ser al menos n-1.")
    grafo = Grafo(dirigido)
    nodos = [Nodo(i) for i in range(n)]
    grafo.nodos = {nodo.nombre: nodo for nodo in nodos}
    aristas_disponibles = [(i, j) for i in range(n) for j in range(i+1, n)]
    aristas_seleccionadas = random.sample(aristas_disponibles, m)
    for i, j in aristas_seleccionadas:
        arista = Arista(nodos[i], nodos[j],  random.randint(1, 20))
        grafo.agregar_arista(arista)
    return grafo

def grafoGilbert(n, p, dirigido=False):
    grafo = Grafo(dirigido)
    nodos = [Nodo(i) for i in range(n)]
    grafo.nodos = {nodo.nombre: nodo for nodo in nodos}
    for i in range(n):
        for j in range(i+1, n):
            if random.random() < p:
                arista = Arista(nodos[i], nodos[j], random.randint(1, 20))
                grafo.agregar_arista(arista)
    return grafo

def grafoGeografico(n, r, dirigido=False):
    grafo = Grafo(dirigido)
    nodos = [Nodo(i) for i in range(n)]
    grafo.nodos = {nodo.nombre: nodo for nodo in nodos}
    for i in range(n):
        for j in range(i+1, n):
            if random.uniform(0, 1) < r:
                arista = Arista(nodos[i], nodos[j], random.randint(1, 20))
                grafo.agregar_arista(arista)
    return grafo

def grafoBarabasiAlbert(n, d, dirigido=False):
    if d < 1:
        raise ValueError("El grado máximo esperado debe ser al menos 1.")
    grafo = Grafo(dirigido)
    nodos = [Nodo(i) for i in range(d)]
    grafo.nodos = {nodo.nombre: nodo for nodo in nodos}
    total_aristas = 0
    for i in range(d):
        for j in range(i+1, d):
            arista = Arista(nodos[i], nodos[j], random.randint(1, 20))
            grafo.agregar_arista(arista)
            total_aristas += 2  # Se cuenta el doble porque es no dirigido
    for i in range(d, n):
        nodos.append(Nodo(i))
        grafo.nodos[i] = Nodo(i)
        for _ in range(d):
            vecino = random.choice(nodos[:i])
            arista = Arista(nodos[i], vecino, random.randint(1, 20))
            grafo.agregar_arista(arista)
            total_aristas += 1
    return grafo

def grafoDorogovtsevMendes(n, dirigido=False):
    if n < 3:
        raise ValueError("El número de nodos debe ser al menos 3 para este modelo.")
    
    grafo = Grafo(dirigido)
    nodos = {}
    aristas_iniciales = []

    # Crear los primeros tres nodos y las aristas iniciales para formar un triángulo
    for i in range(3):
        nodo = Nodo(i)
        nodos[i] = nodo
        grafo.agregar_nodo(nodo)

    aristas_iniciales.extend([(0, 1), (0, 2), (1, 2)])

    # Agregar nodos restantes uno por uno
    for i in range(3, n):
        nodo_nuevo = Nodo(i)
        nodos[i] = nodo_nuevo  # Agregar el nuevo nodo al diccionario de nodos
        grafo.agregar_nodo(nodo_nuevo)
        # Verificar que haya aristas iniciales disponibles
        if aristas_iniciales:
            arista_elegida = random.choice(aristas_iniciales)
            for nodo_extremo in arista_elegida:
                arista = Arista(nodo_nuevo, nodos[nodo_extremo], random.randint(1, 2))
                grafo.agregar_arista(arista)
            aristas_iniciales.extend([(arista_elegida[0], i), (arista_elegida[1], i)])
        else:
            raise ValueError("No hay aristas iniciales disponibles para seleccionar.")

    return grafo

def generar_grafo_dijkstra(grafo_original, s):
    distancias, aristas_arbol = grafo_original.Dijkstra(s)
    nuevo_grafo = Grafo()

    for nodo, (origen, _) in aristas_arbol.items():
        nuevo_grafo.agregar_nodo(Nodo(nodo))
        nuevo_grafo.agregar_nodo(Nodo(origen))
        nuevo_grafo.agregar_arista(Arista(Nodo(origen), Nodo(nodo)))

    return nuevo_grafo