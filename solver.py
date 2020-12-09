from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import networkx as nx
import matplotlib.pyplot as plt
import multiprocessing

#Inicializar parametros
mymatrix = [
    [0, 1, 1, 10000],
    [1, 0, 1, 1],
    [1, 1, 0, 10000],
    [1, 1, 10000, 0],
]

manager = None
data = None

def create_data_model(basematrix):
        """Stores the data for the problem."""
        data = {}
        data['distance_matrix'] = basematrix
        data['num_vehicles'] = 1
        data['depot'] = 0
        return data

def distance_counter(routing, solution):
        route_distance = 0
        index = routing.Start(0)

        while not routing.IsEnd(index):
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
        return route_distance

def print_solution(manager, routing, solution):
        """Prints solution on console."""
        print('Objective: {} miles'.format(solution.ObjectiveValue()))
        index = routing.Start(0)
        plan_output = 'Route:\n'
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} ->'.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
        plan_output += ' {}\n'.format(manager.IndexToNode(index))
        print(plan_output)
        plan_output += 'Route distance: {}miles\n'.format(route_distance)

def calculate_max_distance(matrix):
        return (len(matrix)*(len(matrix)-1))/2


def padding(matrix):
  for i in range(len(matrix)):
    for j in range(len(matrix[i])):
      if matrix[i][j]==0 and i!=j:
        matrix[i][j]=pow(10,6)
  return matrix

def solver(n,p):

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    G = nx.erdos_renyi_graph(n, p)

    #nx.draw(G, with_labels=True)
    #plt.show()

    m = nx.linalg.graphmatrix.adjacency_matrix(G).toarray()
    mymatrix = padding(m)
    data = create_data_model(mymatrix)
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    solution = routing.SolveWithParameters(search_parameters)

    maxdistance = calculate_max_distance(mymatrix)

    if solution and distance_counter(routing, solution) < maxdistance:
        return 1
    return 0


def loop(iteration):
    return solver(iteration[0],iteration[1])


if __name__ == "__main__":

    pool = multiprocessing.Pool(processes=multiprocessing.cpu_count())

    option = 0

    print("¿Que desea realizar?")
    print("1. Analizar la probabilidad de acierto de un grafo aleatorio con distintas probabilidades de conexión")
    print("2. Analizar la probabilidad de acierto de un grafo aleatorio con distinto número de nodos")
    print("3. Analizar la probabilidad de acierto de un grafo aleatorio con una probabilidad de conexión dependiente del número de nodos del mismo\n")
    while option<1 or option > 3:
        option = int(input("Ingrese la opción: "))

    if option == 1:
        n = -1
        while not (n > 0):
            n = int(input("Ingrese el número de nodos: "))

        p = -1
        while not (p > 0):
            p = int(input("Ingrese el número de probabilidades que desea probar: "))

        probabilidades = []

        print("A continuación, ingrese las probabilidades que desea utilizar:")
        for i in range(p):
            probabilidades.append(float(input("")))

        niteraciones = -1
        while not (niteraciones > 0):
            niteraciones = int(input("Ingrese el número de iteraciones que desea realizar para probar cada grafo aleatorio: "))

        probabilidades.sort()
        aciertos = []

        print("\nA continuación se imprimirán los aciertos asociados con cada probabilidad de conexión del grafo:")

        for prob in probabilidades:
            actualGraph = []

            for i in range(niteraciones):
                actualGraph.append((n, prob))

            casosPosibles = pool.map(loop, actualGraph)
            aux = sum(casosPosibles)
            print("P=", prob, " , A=", aux)
            aciertos.append(aux)

        plt.plot(probabilidades, aciertos, 'bo')
        #plt.axis([-0.1, 1.1, -(1 / 100) * niteraciones, niteraciones + (1 / 100) * niteraciones])
        plt.ylabel('Número de aciertos alcanzados')
        plt.xlabel('Probabilidad de conexión del grafo')
        plt.suptitle('Relación Probabilidad del Grafo Vs Número de aciertos')
        plt.show()

        aciertos = list(map((lambda x: x / niteraciones), aciertos))

        plt.plot(probabilidades, aciertos, 'ro')
        #plt.axis([-0.1, 1.1, -0.1, 1.1])
        plt.ylabel('Probabilidad de tener un ciclo Hamiltoniano')
        plt.xlabel('Probabilidad de conexión del grafo')
        plt.suptitle('Relación Probabilidad del Grafo Vs Probabilidad de acierto')
        plt.show()

    elif option == 2:
        p = -1
        while not (p > 0):
            p = float(input("Ingrese la probabilidad de conexión de los grafos: "))

        n = -1
        while not (n > 0):
            n = int(input("Ingrese el número de grafos que desea probar: "))

        nodos = []

        print("A continuación, ingrese el número de nodos que desea utilizar para cada grafo:")
        for i in range(n):
            nodos.append(int(input("")))

        niteraciones = -1
        while not (niteraciones > 0):
            niteraciones = int(input("Ingrese el número de iteraciones que desea realizar para probar cada grafo aleatorio: "))

        nodos.sort()
        aciertos = []

        print("\nP=", p)

        for nodo in nodos:
            actualGraph = []

            for i in range(niteraciones):
                actualGraph.append((nodo, p))

            casosPosibles = pool.map(loop, actualGraph)
            aux = sum(casosPosibles)
            print("N=", nodo, " , A=", aux)
            aciertos.append(aux)

        plt.plot(nodos, aciertos, 'bo')
        #plt.axis([-0.1, 1.1, -(1 / 100) * niteraciones, niteraciones + (1 / 100) * niteraciones])
        plt.ylabel('Número de aciertos alcanzados')
        plt.xlabel('Número de nodos del grafo aleatorio')
        plt.suptitle('Relación Número de nodos Vs Número de aciertos')
        plt.show()

        aciertos = list(map((lambda x: x / niteraciones), aciertos))

        plt.plot(nodos, aciertos, 'ro')
        #plt.axis([-0.1, 1.1, -0.1, 1.1])
        plt.ylabel('Probabilidad de tener un ciclo Hamiltoniano')
        plt.xlabel('Número de nodos del grafo aleatorio')
        plt.suptitle('Relación Número de nodos Vs Probabilidad de acierto')
        plt.show()

    elif option == 3:
        n = -1
        while not (n > 0):
            n = int(input("Ingrese el número de grafos que desea probar: "))

        nodos = []

        print("A continuación, ingrese el número de nodos que desea utilizar para cada grafo:")
        for i in range(n):
            nodos.append(int(input("")))

        niteraciones = -1
        while not (niteraciones > 0):
            niteraciones = int(input("Ingrese el número de iteraciones que desea realizar para probar cada grafo aleatorio: "))

        nodos.sort()
        aciertos = []
        probabilidades = []

        print("\nA continuación se imprimirán los aciertos de cada grafo, cuya probabilidad depende del número de nodos del mismo:")

        for nodo in nodos:
            p = pow(nodo, -1+0.651)  ################################################## FUNCIÓN A CAMBIAR ##################################################
            actualGraph = []

            for i in range(niteraciones):
                actualGraph.append((nodo, p))

            casosPosibles = pool.map(loop, actualGraph)
            aux = sum(casosPosibles)
            print("P= ", p, " , N=", nodo, " , A=", aux)
            aciertos.append(aux)
            probabilidades.append(p)

        plt.plot(nodos, probabilidades, 'bo')
        # plt.axis([-0.1, 1.1, -(1 / 100) * niteraciones, niteraciones + (1 / 100) * niteraciones])
        plt.ylabel('Probabilidad de conexión del grafo')
        plt.xlabel('Número de nodos del grafo')
        plt.suptitle('Relación Número de nodos Vs Probabilidad de conexión del grafo')
        plt.show()

        plt.plot(probabilidades, aciertos, 'bo')
        # plt.axis([-0.1, 1.1, -(1 / 100) * niteraciones, niteraciones + (1 / 100) * niteraciones])
        plt.ylabel('Número de aciertos alcanzados')
        plt.xlabel('Probabilidad de conexión del grafo')
        plt.suptitle('Relación Probabilidad de conexión del grafo Vs Número de aciertos')
        plt.show()

        plt.plot(nodos, aciertos, 'bo')
        # plt.axis([-0.1, 1.1, -(1 / 100) * niteraciones, niteraciones + (1 / 100) * niteraciones])
        plt.ylabel('Número de aciertos alcanzados')
        plt.xlabel('Número de nodos del grafo aleatorio')
        plt.suptitle('Relación Número de nodos Vs Número de aciertos')
        plt.show()

        aciertos = list(map((lambda x: x / niteraciones), aciertos))

        plt.plot(nodos, aciertos, 'ro')
        # plt.axis([-0.1, 1.1, -0.1, 1.1])
        plt.ylabel('Probabilidad de tener un ciclo Hamiltoniano')
        plt.xlabel('Número de nodos del grafo aleatorio')
        plt.suptitle('Relación Número de nodos Vs Probabilidad de acierto')
        plt.show()
