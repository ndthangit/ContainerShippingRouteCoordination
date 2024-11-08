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

        # requests
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


# def print_solution(data,manager, routing, solution):
#     """Prints solution on console."""
#     plan_output = f'ROUTES {manager.GetNumberOfVehicles()}\n'
#     max_route_distance = 0
#     for vehicle_id in range(manager.GetNumberOfVehicles()):
#         index = routing.Start(vehicle_id)
#         plan_output += f'TRUCK {vehicle_id + 1}\n'
#         route_distance = 0
#         while not routing.IsEnd(index):
#             node_index = manager.IndexToNode(index)
#             action = ''
#             request_id = ''
#             for req_id, req in data['requests'].items():
#                 if req['pickup_location'] == node_index:
#                     action = req['pickup_action']
#                     request_id = req_id + 1
#                 elif req['drop_location'] == node_index:
#                     action = req['drop_action']
#                     request_id = req_id + 1
#             if action:
#                 plan_output += f'{node_index + 1} {action} {request_id}\n'
#             else:
#                 plan_output += f'{node_index + 1} STOP\n'
#             previous_index = node_index
#             node_index = solution.Value(routing.NextVar(node_index))
#             route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
#         plan_output += f'Total distance of the route: {route_distance}m\n'
#         max_route_distance = max(route_distance, max_route_distance)
#         plan_output += '\n'
#
#         plan_output += '#\n'
#     plan_output += f'Maximum of the route distances: {max_route_distance}m\n'
#
#     print(plan_output)
#     return plan_output

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    max_route_distance = 0
    capacity_dimension = routing.GetDimensionOrDie("Capacity")
    for vehicle_id in range(data['num_trucks']):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id + 1}:\n"
        route_distance = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            load = solution.Value(capacity_dimension.CumulVar(index))
            plan_output += f" {node_index + 1} (Load: {load}) -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        node_index = manager.IndexToNode(index)
        load = solution.Value(capacity_dimension.CumulVar(index))
        plan_output += f"{node_index + 1} (Load: {load})\n"
        plan_output += f"Distance of the route: {route_distance}m\n"
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    print(f"Maximum of the route distances: {max_route_distance}m")
    # for vehicle_id in range(manager.GetNumberOfVehicles()):
    #     for from_index in range(manager.GetNumberOfNodes()):
    #         for to_index in range(manager.GetNumberOfNodes()):
    #             if from_index != to_index:
    #                 cost = routing.GetArcCostForVehicle(from_index, to_index, vehicle_id)
    #                 print(f"Arc cost from {from_index} to {to_index} for vehicle {vehicle_id}: {cost}")


def main():
    data = create_data_model()
    # print(data[''])
    manager = pywrapcp.RoutingIndexManager(
        len(data['delivery_time']), data['num_trucks'], data['starts'], data['ends']
    )
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        '''Returns the delivery-time between the two nodes.'''
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['delivery_time'][from_node][to_node]

    # print(time_callback(2, 5))
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        30000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name,
    )
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)
    # print(data['requests'].values())

    # Add Pickup and Delivery constraints
    for req in data['requests'].values():
        pickup_index = manager.NodeToIndex(req['pickup_location'])
        delivery_index = manager.NodeToIndex(req['drop_location'])
        pickup_time = req['pickup_duration']
        drop_time = req['drop_duration']
        # print(pickup_index, delivery_index)
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        routing.solver().Add(routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))

        routing.solver().Add(distance_dimension.SlackVar(pickup_index).SetRange(0, pickup_time))
        routing.solver().Add(distance_dimension.SlackVar(delivery_index).SetRange(0, drop_time))

        # new_delivery_cumlVar = distance_dimension.CumulVar(delivery_index) + drop_time
        # routing.solver().Add(distance_dimension.SetCumulVarSoftUpperBound(delivery_index, new_delivery_cumlVar, 1))

        routing.solver().Add(distance_dimension.CumulVar(pickup_index) <= distance_dimension.CumulVar(delivery_index))

    # Thêm dimension phụ để đánh dấu trạng thái lấy/trả hàng (0 = chưa lấy, 1 = đã lấy)
    status_evaluator_index = routing.RegisterUnaryTransitCallback(
        lambda index: 1)  # Chuyển trạng thái sau khi đi qua điểm
    routing.AddDimension(
        status_evaluator_index,
        0,  # Slack_max = 0 vì không có thời gian chờ
        1,  # Giới hạn trạng thái (0 -> 1)
        True,  # Fix_start_cumul_to_zero để khởi đầu bằng 0 (chưa lấy)
        "Status"  # Tên dimension cho trạng thái
    )
    status_dimension = routing.GetDimensionOrDie("Status")

    for req in data['requests'].values():
        pickup_index = manager.NodeToIndex(req['pickup_location'])
        delivery_index = manager.NodeToIndex(req['drop_location'])

        # Ràng buộc trạng thái tại điểm lấy và trả hàng
        routing.solver().Add(
            status_dimension.CumulVar(pickup_index) == 0  # Chưa lấy tại điểm lấy hàng
        )
        routing.solver().Add(
            status_dimension.CumulVar(delivery_index) == 1  # Đã lấy trước khi đến điểm trả hàng
        )

    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)

        # Kiểm tra trạng thái của điểm dựa trên dimension phụ "Status"
        status_value = status_dimension.CumulVar(from_node).Min()
        if status_value == 0:
            # Nếu chưa qua điểm lấy hàng, coi đây là điểm lấy hàng
            for req in data['requests'].values():
                if req['pickup_location'] == from_node:
                    return req['size']
        else:
            # Nếu đã qua điểm lấy hàng, coi đây là điểm trả hàng
            for req in data['requests'].values():
                if req['drop_location'] == from_node:
                    return -req['size']
        return 0

    # Đăng ký hàm callback cho tải trọng
    demand_evaluator_index = routing.RegisterUnaryTransitCallback(demand_callback)

    # Thêm dimension cho tải trọng của xe
    routing.AddDimensionWithVehicleCapacity(
        demand_evaluator_index,
        0,  # Không có thời gian chờ (slack)
        data['truck_capacities'],
        True,
        "Capacity"
    )

    # Định nghĩa dimension cho trạng thái thiết bị trên xe
    def truck_status_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        status_value = status_dimension.CumulVar(from_node).Min()

        if from_node == data['Romooc_location']:
            return 1  # quay về ấy romooc
        if status_value == 0:
            for req in data['requests'].values():
                if req['pickup_location'] == from_node:
                    if req['pickup_action'] == 'PICKUP_CONTAINER_TRAILER':
                        return 1
        else:
            for req in data['requests'].values():
                if req['drop_location'] == from_node:
                    if req['drop_action'] == 'PICKUP_CONTAINER_TRAILER':
                        return -1
        return 0

    truck_status_index = routing.RegisterUnaryTransitCallback(truck_status_callback)
    routing.AddDimension(
        truck_status_index,
        0,  # Slack max
        1,  # Xe chỉ có thể mang 1 thiết bị
        True,  # Thiết lập trạng thái khởi đầu không có thiết bị
        "Truck"
    )

    truck_dimension = routing.GetDimensionOrDie("Truck")
    # Ràng buộc xe phải quay lại trả thiết bị về bãi nếu còn còn romooc cuối hành trình
    for vehicle_id in range(data['num_trucks']):
        end_index = routing.End(vehicle_id)
        routing.solver().Add(
            truck_dimension.CumulVar(end_index) == 0  # Thiết bị phải được trả trước khi kết thúc hành trình
        )






    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    # search_parameters.local_search_metaheuristic = (
    #     routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    # )
    search_parameters.time_limit.FromSeconds(30)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print('No solution found !')


if __name__ == "__main__":
    main()
