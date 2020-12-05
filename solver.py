from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import networkx as nx
import matplotlib.pyplot as plt

#Inicializar parametros
mymatrix = [
    [0, 1, 1, 10000],
    [1, 0, 1, 1],
    [1, 1, 0, 10000],
    [1, 1, 10000, 0],
]

manager = None
data = None
casosPosibles = 0

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

        global casosPosibles
        casosPosibles = casosPosibles + 1

        #print_solution(manager, routing, solution)

if __name__ == "__main__":


    n=-1
    while not(n>0):
        n = int(input("Ingrese el número de nodos: "))

    p=-1
    while not(p<=1 and p>=0):
        p = float(input("Ingrese la probabilidad de conexión entre nodos: "))

    for i in range(1000):
        solver(n,p)
    print(casosPosibles)
