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
    data['delivery_time'] = distance

    # Đọc thông tin rơ-mooc (TRAILER p d)
    _, trailer_location, romooc_time = [x for x in sys.stdin.readline().split()]
    data['Romooc_location'] = int(trailer_location) - 1
    data['Romooc_time'] = int(romooc_time)

    # Đọc số đầu kéo (TRUCK m)
    _, num_trucks = [x for x in sys.stdin.readline().split()]
    data['num_trucks'] = int(num_trucks)
    data['truck_location'] = {}

    # vị trí đầu kéo của các xe
    for _ in range(int(num_trucks)):
        truck_id, location = [int(x) for x in sys.stdin.readline().split()]
        data['truck_location'][truck_id - 1] = location - 1
    # vị tr bắt đầu và kết thúc của các xe
    data['starts'] = [data['truck_location'][i] for i in range(data['num_trucks'])]
    data['ends'] = [data['truck_location'][i] for i in range(data['num_trucks'])]

    data['truck_capacities'] = [40 for i in range(data['num_trucks'])]

    data['requests'] = {}
    while True:
        line = sys.stdin.readline().strip()
        if line == '#':
            break
        tokens = line.split()
        req_id = int(tokens[1]) - 1
        size = int(tokens[2])
        p1 = int(tokens[3]) - 1
        pickup_action = tokens[4]
        pickup_duration = int(tokens[5])
        p2 = int(tokens[6]) - 1
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
        # data['demands'] = [0] * data['num_points']
        # for req in data['requests'].values():
        #     data['demands'][req['pickup_location']] = req['size']
        #     data['demands'][req['drop_location']] = -req['size']

    return data
def print_solution(manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()} miles")
    index = routing.Start(0)
    plan_output = "Route for vehicle 0:\n"
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += f" {manager.IndexToNode(index)} ->"
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += f" {manager.IndexToNode(index)}\n"
    print(plan_output)
    plan_output += f"Route distance: {route_distance}miles\n"


data = create_data_model()
# print(data[''])
manager = pywrapcp.RoutingIndexManager(
    len(data['delivery_time']), data['num_trucks'], data['starts'], data['ends']
)
routing = pywrapcp.RoutingModel(manager)


# Add Pickup and Delivery constraints
for req in data['requests'].values():
    pickup_index = manager.NodeToIndex(req['pickup_location'])
    delivery_index = manager.NodeToIndex(req['drop_location'])
    routing.AddPickupAndDelivery(pickup_index, delivery_index)
    routing.solver().Add(routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))

def distance_callback(from_index, to_index):
    '''Returns the delivery-time between the two nodes.'''
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return data['delivery_time'][from_node][to_node]


transit_callback_index = routing.RegisterTransitCallback(distance_callback)

# Define cost of each arc.
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# def demand_callback(from_index):
#     '''Returns the demand of the node.'''
#     # error
#     from_node = manager.IndexToNode(from_index)
#
#     # Check if the node is a pickup or drop location
#     for req_id, req in data['requests'].items():
#         if req['pickup_location'] == from_node:
#             return req['size']  # Positive demand for pickup
#         elif req['drop_location'] == from_node:
#             return -req['size']  # Negative demand for drop
#
#     return 0  # No demand for other nodes
#
#
# #
# demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
# #
# routing.AddDimensionWithVehicleCapacity(
#     demand_callback_index,
#     0,  # null capacity slack
#     data['truck_capacities'],  # vehicle maximum capacities
#     True,  # start cumul to zero
#     'Capacity',
# )

# Setting first solution heuristic.
search_parameters = pywrapcp.DefaultRoutingSearchParameters()

#
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
)

# Solve the problem.
solution = routing.SolveWithParameters(search_parameters)
#
# # data_models.printSolution(data, manager, routing, solution)
