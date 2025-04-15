
## Búsqueda no informada

### Un problema de búsqueda consiste en:
- _Un espacio de estado:_ Es un conjunto de todos los estados posibles en los que puede estar.
- _Un estado de inicio:_ El estado desde donde comienza la búsqueda
- _Un estado final(objetivo):_ Una función que mira el estado actual devuelve si es el estado objetivo o no. 

### Tipos de algoritmos de búsqueda
Hay varios algoritmos de búsqueda muy eficientes y poderosos , los mas elementales son:
![[Pasted image 20250407174338.png]]

![[Pasted image 20250407135127.png]]

#### Primero en Profundidad(DFS - Búsqueda en profundidad)
1. _Completitud:_ DFS es completo si y solo si usamos búsqueda basada en grafos en espacios de estado finitos.

2. _Optimalidad:_ DFS en ningún caso asegura la optimalidad , pues puede encontrar una solución más profunda que otro en una rama que todavía no ha sido expandida.     

3. _Complejidad Temporal:_ En el pero caso , es **O(b^m)** ,siendo b el factor de ramificación y m la máxima profundidad del espacio de estados.

4. _Complejidad espacial:_ **O(b^d)** , siendo b el factor de ramificación y d la profundidad de la solución menos costosa.

##### Pseudocódigo:
``` python
DFS(grafo G)
     PARA CADA vértice u ∈ V[G] HACER
             estado[u] ← NO_VISITADO
             padre[u] ← NULO
     tiempo ← 0
    PARA CADA vértice u ∈ V[G] HACER
             SI estado[u] = NO_VISITADO ENTONCES
                     DFS_Visitar(u,tiempo)


   DFS_Visitar(nodo u, int tiempo)
     estado[u] ← VISITADO
     tiempo ← tiempo + 1
     d[u] ← tiempo
     PARA CADA v ∈ Vecinos[u] HACER
             SI estado[v] = NO_VISITADO ENTONCES
                     padre[v] ← u
                     DFS_Visitar(v,tiempo)
     estado[u] ← TERMINADO
     tiempo ← tiempo + 1
     f[u] ← tiempo
```

##### Variante -> DLS (Depth-Limited Search - Búsqueda en Profundidad Limitada)

- **Definición:** Es una versión de DFS que impone un límite de profundidad (_depth limit_ `L`) para evitar que la búsqueda se adentre infinitamente en una rama del árbol/grafo.
    
- **Características:**
    
    - Explora nodos hasta una profundidad máxima predefinida (`L`).
        
    - Si la solución está más allá de `L`, no la encontrará.
        
    - Evita el problema de ciclos infinitos en grafos no acotados.
        
- **Complejidad:**
    
    - **Tiempo:** `O(b^L)` (donde `b` es el factor de ramificación).
        
    - **Espacio:** `O(b * L)` (similar a DFS, pero limitado por `L`).
        
- **Ventaja:** Útil cuando se conoce una cota razonable para la profundidad de la solución.
    
- **Desventaja:** Si `L` es demasiado pequeño, puede no encontrar la solución.


###### Pseudocódigo:
``` python
función DLS(nodo_actual, objetivo, profundidad_actual, L):
    # Caso base 1: Se encontró el objetivo
    si nodo_actual == objetivo:
        return True
    
    # Caso base 2: Se alcanzó el límite de profundidad
    si profundidad_actual >= L:
        return False
    
    # Explorar nodos hijos
    para cada hijo en nodo_actual.vecinos:|
        si DLS(hijo, objetivo, profundidad_actual + 1, L) == True:
            return True
    
    # Si no se encontró en esta rama
    return False
```


##### Variante -> IDS (Iterative Deepening Search - Búsqueda en Profundidad Iterativa)
- **Definición:** Combina las ventajas de DFS (bajo consumo de memoria) y BFS (optimalidad para búsqueda no ponderada). Realiza múltiples ejecuciones de DLS incrementando progresivamente el límite de profundidad (`L = 0, 1, 2, ...`) hasta encontrar la solución.
    
- **Características:**
    
    - **Optimal:** Encuentra la solución más cercana (menor profundidad) como BFS.
        
    - **Completo:** Siempre encuentra la solución si existe.
        
    - Eficiente en memoria (usa DFS en cada iteración).
        
- **Complejidad:**
    
    - **Tiempo:** `O(b^d)` (donde `d` es la profundidad de la solución).
        
    - **Espacio:** `O(b * d)` (similar a DFS).
        
- **Ventajas:**
    
    - No requiere ajustar manualmente el límite de profundidad (a diferencia de DLS).
        
    - Ideal para espacios de búsqueda grandes con profundidad desconocida.
        
- **Desventaja:** Repite cálculos en cada iteración (aunque esto suele ser menos crítico de lo que parece).

###### Pseudocódigo:
``` python
función IDS(nodo_inicial, objetivo):
    L = 0  # Límite inicial de profundidad
    
    mientras True:
        si encontrado == True:
            return True
        si no hay más nodos por explorar en profundidad L:
            return False
        L += 1  # Incrementa el límite para la siguiente iteración  # Incrementa el límite para la próxima iteración
función DLS(nodo_actual, objetivo, L):
    si L == 0 y nodo_actual == objetivo:
        return True
    si L > 0:
        para cada hijo en nodo_actual.vecinos:
            si DLS(hijo, objetivo, L - 1) == True:
                return True
    return False
```






#### Primero en Amplitud(BFS - Búsqueda en anchura)
1. _Completitud_
    
    - **Sí es completo** en grafos finitos y con un factor de ramificación finito (bb), ya que eventualmente explorará todos los nodos.
        
    - En grafos infinitos, no es completo si la solución está en una rama inalcanzable.
        
2. _Optimalidad:_
    
    - **Es óptimo** para encontrar el **camino más corto** en grafos **no ponderados** (todos los arcos tienen el mismo costo).
        
    - En grafos ponderados, no garantiza optimalidad (para eso se usa Dijkstra o A*).
        
3. _Complejidad Temporal:_
    
    - **O(b^d)**, donde:
        
        - bb = factor de ramificación.
            
        - dd = profundidad de la solución menos profunda (óptima).
            
4. _Complejidad Espacial:_
    
    - **O(b^d)** (almacena todos los nodos de un nivel en la cola).
        
    - Es más demandante en memoria que DFS para grafos anchos.
        

---

##### Pseudocódigo:
```python
BFS(grafo G, nodo_inicial s):
    # Inicialización
    PARA CADA vértice u ∈ V[G] HACER:
        estado[u] ← NO_VISITADO
        distancia[u] ← INFINITO
        padre[u] ← NULO
    
    # Configuración del nodo inicial
    estado[s] ← VISITADO
    distancia[s] ← 0
    cola = COLA_VACÍA()
    ENCOLAR(cola, s)

    # Exploración por niveles
    MIENTRAS cola NO ESTÉ VACÍA HACER:
        u ← DESENCOLAR(cola)
        
        PARA CADA v ∈ Vecinos[u] HACER:
            SI estado[v] = NO_VISITADO ENTONCES:
                estado[v] ← VISITADO
                distancia[v] ← distancia[u] + 1
                padre[v] ← u
                ENCOLAR(cola, v)
        
        estado[u] ← TERMINADO  # Opcional (depende del problema)
```






#### Búsqueda Costo uniforme
La **Búsqueda de Costo Uniforme (UCS)** es un algoritmo que extiende **BFS** para grafos con aristas de **costos variables**. Su objetivo es encontrar el camino de **menor costo acumulado** desde el nodo inicial hasta el nodo objetivo.

1. _Completitud_
    
    - **Sí es completo** (si el costo de cada arista es positivo y el grafo es finito).
        
    - En grafos infinitos, puede no terminar si el costo crece exponencialmente.
        
2. _Optimalidad:_
    
    - **Sí es óptimo** (encuentra siempre el camino de menor costo).
        
    - Funciona como **Dijkstra** sin heurísticas.
        
3. _Complejidad Temporal:_
    
    - **O(b^1+[C∗/ϵ])**, donde:
        
        - b = factor de ramificación.
            
        - C∗ = costo de la solución óptima.
            
        - ϵ = costo mínimo de una arista.
            
4. _Complejidad Espacial:_
    
    - **O(b^1+[C∗/ϵ])** (similar a la temporal).


##### Pseudocódigo:

```python
UCS(grafo G, nodo_inicial s, nodo_objetivo t):
    # Inicialización
    PARA CADA vértice u ∈ V[G] HACER:
        costo[u] ← INFINITO
        padre[u] ← NULO
    
    costo[s] ← 0
    cola_prioridad = COLA_PRIORIDAD_VACÍA()
    ENCOLAR(cola_prioridad, (s, 0))  # (nodo, costo_acumulado)

    MIENTRAS cola_prioridad NO ESTÉ VACÍA HACER:
        u, costo_u ← DESENCOLAR(cola_prioridad)
        
        SI u = t ENTONCES:
            RECONSTRUIR_CAMINO(padre, t)  # Devuelve el camino óptimo
            TERMINAR
        
        PARA CADA v ∈ Vecinos[u] HACER:
            nuevo_costo ← costo_u + peso(u, v)
            SI nuevo_costo < costo[v] ENTONCES:
                costo[v] ← nuevo_costo
                padre[v] ← u
                ENCOLAR(cola_prioridad, (v, nuevo_costo))
```






## Búsqueda Informada
#### Búsqueda Voraz
Un **algoritmo voraz** (también conocido como **codicioso**, **goloso**, **ávido**, **devorador** o **greedy**) es una estrategia de [búsqueda](https://es.wikipedia.org/wiki/Algoritmo_de_b%C3%BAsqueda "Algoritmo de búsqueda") por la cual se sigue una [heurística](https://es.wikipedia.org/wiki/Heur%C3%ADstica "Heurística") consistente en elegir la opción óptima en cada paso local con la esperanza de llegar a una solución general óptima. Este esquema algorítmico es el que menos dificultades plantea a la hora de diseñar y comprobar su funcionamiento.

```python
función algoritmo_voraz(conjunto_de_entrada):
    solución = []  # Inicializar solución vacía
    mientras conjunto_de_entrada no esté vacío:
        # 1. Seleccionar el mejor elemento localmente (según un criterio)
        elemento = seleccionar_mejor_elemento(conjunto_de_entrada)
        
        # 2. Verificar si el elemento es válido para la solución
        si es_válido(elemento, solución):
            solución.agregar(elemento)
        
        # 3. Eliminar el elemento procesado del conjunto de entrada
        conjunto_de_entrada.eliminar(elemento)
    
    retornar solución
```


#### Búsqueda A*
La eficacia del algoritmo A* procede de su evaluación inteligente de trayectorias mediante tres componentes clave: g(n), h(n) y f(n). Estos componentes trabajan juntos para guiar el proceso de búsqueda hacia los caminos más prometedores.

![algoritmo a](https://media.datacamp.com/cms/google/ad_4nxdurjsqymkb8cxyk0jl0tq0m82pqowydssekbtc4ltysic-tp59lttv22izzlvnfqowfcodbanjb2l6fzpbf7akwmplmogbav8o3norwakys9ayugjoq-v32fmpiw3khvokhnuvxa.jpeg)

##### Comprender las funciones de coste

###### Coste del camino g(n)

La función de coste del camino g(n) representa la distancia exacta y conocida desde el nodo inicial de partida hasta la posición actual en nuestra búsqueda. A diferencia de los valores estimados, este coste es preciso y se calcula sumando todos los pesos individuales de las aristas que se han recorrido a lo largo del camino elegido. 

Matemáticamente, para una trayectoria a través de los nodos n0(nodo inicial) a nk​ (nodo actual), podemos expresar g(n) como

![[Pasted image 20250409183412.png]]

Dónde:

- _w(ni,ni+1)_ representa el peso de la arista que conecta el nodo ni al nodo ni+1​.

A medida que avanzamos por el gráfico, este valor se acumula, dándonos una medida clara de los recursos reales (ya sea distancia, tiempo o cualquier otra métrica) que hemos gastado para llegar a nuestra posición actual.

###### Función heurística h(n)

La función heurística h(n) proporciona un coste estimado desde el nodo actual hasta el nodo objetivo, actuando como "conjetura informada" del algoritmo sobre el camino restante. 

Matemáticamente, para cualquier nodo n dado, la estimación heurística debe satisfacer la condición h(n)≤h*(n) donde h*(n) es el coste real del objetivo, lo que lo hace admisible al no sobrestimar nunca el coste real.

En los problemas basados en cuadrículas o en mapas, las funciones heurísticas comunes incluyen la [distancia Manhattan](https://www.datacamp.com/es/tutorial/manhattan-distance.) y [distancia euclidiana](https://www.datacamp.com/es/tutorial/euclidean-distance). Para las coordenadas (x1,y1) del nodo actual y (x2,y2) del nodo meta, estas distancias se calculan como

![[Pasted image 20250409183545.png]]


###### Coste total estimado f(n)

El coste total estimado f(n) es la piedra angular del proceso de toma de decisiones del algoritmo A*, que combina tanto el coste real de la ruta como la estimación heurística para evaluar el potencial de cada nodo. Para cualquier nodo n, este coste se calcula como

![[Pasted image 20250409183616.png]]


Dónde:

- g(n) representa el coste real desde el inicio hasta el nodo actual,
- h(n) representa el coste estimado desde el nodo actual hasta el objetivo.

El algoritmo [algoritmo](https://www.datacamp.com/es/blog/what-is-an-algorithm) utiliza este valor combinado para elegir estratégicamente qué nodo explorar a continuación, seleccionando siempre el nodo con la menor f(n) más bajo de la lista abierta, garantizando así un equilibrio óptimo entre los costes conocidos y las distancias restantes estimadas.

```python
función A_Estrella(inicio, objetivo):
    # Inicializar listas abierta y cerrada
    listaAbierta = [inicio]          # Nodos por evaluar
    listaCerrada = []                # Nodos ya evaluados
    
    # Inicializar propiedades del nodo inicial
    inicio.g = 0                     # Costo desde inicio hasta inicio es 0
    inicio.h = heuristica(inicio, objetivo)  # Estimación hasta el objetivo
    inicio.f = inicio.g + inicio.h   # Costo total estimado
    inicio.padre = null              # Para reconstruir el camino
    
    mientras listaAbierta no esté vacía:
        # Obtener nodo con el valor f más bajo (usar cola de prioridad para eficiencia)
        actual = nodo en listaAbierta con el valor f más bajo
        
        # Verificar si se alcanzó el objetivo
        si actual == objetivo:
            retornar reconstruir_camino(actual)
            
        # Mover nodo actual de listaAbierta a listaCerrada
        eliminar actual de listaAbierta
        agregar actual a listaCerrada
        
        # Evaluar todos los vecinos del nodo actual
        para cada vecino de actual:
            si vecino en listaCerrada:
                continuar  # Saltar nodos ya evaluados
                
            # Calcular puntuación g tentativa
            g_tentativo = actual.g + distancia(actual, vecino)
            
            si vecino no está en listaAbierta:
                agregar vecino a listaAbierta

            sino si g_tentativo >= vecino.g:
                continuar  # Esta ruta no es mejor
                
            # Esta ruta es la mejor hasta ahora
            vecino.padre = actual
            vecino.g = g_tentativo
            vecino.h = heuristica(vecino, objetivo)
            vecino.f = vecino.g + vecino.h
    
    retornar "fallo"  # No existe camino

función reconstruir_camino(actual):
    camino = []
    mientras actual no sea null:
        agregar actual al inicio de camino
        actual = actual.padre
    retornar camino
```

#### 15-Puzzle - A*

```python
# Función principal
función resolver_15_puzzle(estado_inicial, estado_objetivo):
    # Inicialización
    listaAbierta = ColaPrioridad()
    listaCerrada = Diccionario()  # Para almacenar estados visitados y sus costos g(n)
    
    # Propiedades del estado inicial
    estado_inicial.g = 0
    estado_inicial.h = heuristica_manhattan(estado_inicial, estado_objetivo)
    estado_inicial.f = estado_inicial.g + estado_inicial.h
    estado_inicial.padre = None
    
    listaAbierta.encolar(estado_inicial, estado_inicial.f)
    
    mientras listaAbierta no esté vacía:
        actual = listaAbierta.desencolar()
        
        si actual == estado_objetivo:
            retornar reconstruir_camino(actual)
        
        listaCerrada[actual] = actual.g
        
        para cada vecino en generar_vecinos(actual):
            si vecino en listaCerrada y vecino.g <= actual.g + 1:
                continuar  # Ignorar si ya existe un camino mejor
            
            # Calcular nuevos valores
            g_tentativo = actual.g + 1  # Cada movimiento tiene costo 1
            h_tentativo = heuristica_manhattan(vecino, estado_objetivo)
            f_tentativo = g_tentativo + h_tentativo
            
            si vecino no está en listaAbierta o g_tentativo < vecino.g:
                vecino.padre = actual
                vecino.g = g_tentativo
                vecino.h = h_tentativo
                vecino.f = f_tentativo
                
                si vecino no está en listaAbierta:
                    listaAbierta.encolar(vecino, vecino.f)
    
    retornar "No hay solución"

# Función para calcular la heurística Manhattan
función heuristica_manhattan(estado, estado_objetivo):
    h = 0
    para cada ficha en 1..15:
        x_actual, y_actual = encontrar_posición(estado, ficha)
        x_objetivo, y_objetivo = encontrar_posición(estado_objetivo, ficha)
        h += abs(x_actual - x_objetivo) + abs(y_actual - y_objetivo)
    retornar h

# Generar vecinos (movimientos válidos del espacio vacío)
función generar_vecinos(estado):
    vecinos = []
    x_vacio, y_vacio = encontrar_posición(estado, 0)  # Posición del espacio vacío (0)
    
    # Movimientos posibles: arriba, abajo, izquierda, derecha
    para cada (dx, dy) en [(0, 1), (0, -1), (1, 0), (-1, 0)]:
        x_nuevo, y_nuevo = x_vacio + dx, y_vacio + dy
        si 0 <= x_nuevo < 4 y 0 <= y_nuevo < 4:
            nuevo_estado = copiar(estado)
            # Intercambiar espacio vacío con ficha adyacente
            nuevo_estado[x_vacio][y_vacio] = nuevo_estado[x_nuevo][y_nuevo]
            nuevo_estado[x_nuevo][y_nuevo] = 0
            vecinos.agregar(nuevo_estado)
    retornar vecinos

# Reconstruir el camino desde el objetivo hasta el inicio
función reconstruir_camino(estado):
    camino = []
    mientras estado no sea None:
        camino.insertar(0, estado)  # Insertar al inicio
        estado = estado.padre
    retornar camino
```