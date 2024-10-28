import data_models
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

data = data_models.create_data_model()
print(data)
manager = pywrapcp.RoutingIndexManager(
    len(data['distance']), data['num_vehicles'], data['depot']
)
routing = pywrapcp.RoutingModel(manager)

# print(data['demands'])
# print(data['deliveries'])
# print(data['vehicle_capacity'])

# Add Pickup and Delivery constraints
for pickup, delivery in data['deliveries']:
    pickup_index = manager.NodeToIndex(pickup)
    delivery_index = manager.NodeToIndex(delivery)
    routing.AddPickupAndDelivery(pickup_index, delivery_index)
    routing.solver().Add(routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))


def distance_callback(from_index, to_index):
    '''Returns the distance between the two nodes.'''
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return data['distance'][from_node][to_node]


transit_callback_index = routing.RegisterTransitCallback(distance_callback)


def demand_callback(from_index):
    '''Returns the demand of the node.'''
    from_node = manager.IndexToNode(from_index)
    return data['demands'][from_node]


demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

routing.AddDimensionWithVehicleCapacity(
    demand_callback_index,
    0,  # null capacity slack
    data['vehicle_capacity'],  # vehicle maximum capacities
    True,  # start cumul to zero
    'Capacity',
)

# Define cost of each arc.
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# Setting first solution heuristic.
search_parameters = pywrapcp.DefaultRoutingSearchParameters()


search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
)

# Solve the problem.
solution = routing.SolveWithParameters(search_parameters)

data_models.printSolution(data, manager, routing, solution)