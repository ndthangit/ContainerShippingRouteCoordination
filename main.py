# import data_models
import sys

from ortools.constraint_solver import pywrapcp, routing_enums_pb2
def create_data_model():
    data = {}
    # Dòng đầu tiên: Lấy số điểm (Points N)
    _, num_points = [x for x in sys.stdin.readline().split()]
    data['num_points'] = int(num_points)

    # Dòng tiếp theo: Khoảng cách giữa các điểm (DISTANCES N^2)
    _, n_squared = [x for x in sys.stdin.readline().split()]
    n = int(num_points)
    distance = [[0] * n for _ in range(n)]

    # Đọc các dòng khoảng cách
    for _ in range(int(n_squared)):
        i, j, d = [int(x) for x in sys.stdin.readline().split()]
        distance[i - 1][j - 1] = d
    data['distance'] = distance

    # Đọc thông tin rơ-mooc (TRAILER p d)
    _, trailer_location, romooc_time = [x for x in sys.stdin.readline().split()]
    data['Romooc_location'] = int(trailer_location)-1
    data['Romooc_time'] = int(romooc_time)

    # Đọc số đầu kéo (TRUCK m)
    _, num_trucks = [x for x in sys.stdin.readline().split()]
    data['num_trucks'] = int(num_trucks)
    data['truck_location'] = {}


    #v trí đầu kéo của các xe
    for _ in range(int(num_trucks)):
        truck_id, location = [int(x) for x in sys.stdin.readline().split()]
        data['truck_location'][truck_id-1] = location-1

    data['requests'] = {}
    while True:
        line = sys.stdin.readline().strip()
        if line == '#':
            break
        tokens = line.split()
        req_id = int(tokens[1])-1
        size = int(tokens[2])
        p1 = int(tokens[3])-1
        pickup_action = tokens[4]
        pickup_duration = int(tokens[5])
        p2 = int(tokens[6])-1
        drop_action = tokens[7]
        drop_duration = int(tokens[8])

        # Lưu vào từ điển requests
        data['requests'][req_id] = {
            'size': size,
            'pickup_location': p1,
            'pickup_action': pickup_action,
            'pickup_duration': pickup_duration,
            'drop_location': p2,
            'drop_action': drop_action,
            'drop_duration': drop_duration
        }
    return data

data = create_data_model()
print(data)
manager = pywrapcp.RoutingIndexManager(
    len(data['distance']), data['num_trucks'], data['truck_location']
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

# data_models.printSolution(data, manager, routing, solution)
