import sys
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

time_max = 100000

def create_data_model():
    data = {}

    # Đọc số điểm (Points N)
    _, num_points = [x for x in sys.stdin.readline().split()]
    num_points = int(num_points)
    data['num_points'] = num_points

    # Tổng số điểm mới sau khi tách các điểm thành pickup và delivery
    total_points = 2 * num_points
    data['total_points'] = total_points

    # Đọc khoảng cách giữa các điểm (DISTANCES N^2)
    _, n_squared = [x for x in sys.stdin.readline().split()]
    distance = [[float('inf')] * total_points for _ in range(total_points)]

    # Đọc các dòng khoảng cách
    for _ in range(int(n_squared)):
        i, j, d = [int(x) for x in sys.stdin.readline().split()]
        distance[i - 1][j - 1] = d
        distance[i - 1 + num_points][j - 1] = d
        distance[i - 1][j - 1 + num_points] = d
        distance[i - 1 + num_points][j - 1 + num_points] = d

    for i in range(num_points):
        pickup_index = i
        delivery_index = i + num_points
        distance[pickup_index][delivery_index] = time_max
        distance[delivery_index][pickup_index] = time_max

    data['delivery_time'] = distance

    # Đọc thông tin rơ-mooc (TRAILER p d)
    _, trailer_location, romooc_time = [x for x in sys.stdin.readline().split()]
    data['Romooc_location'] = int(trailer_location) - 1
    data['Romooc_time'] = int(romooc_time)

    # Đọc số đầu kéo (TRUCK m)
    _, num_trucks = [x for x in sys.stdin.readline().split()]
    data['num_trucks'] = int(num_trucks)
    data['truck_location'] = {}

    # Vị trí đầu kéo của các xe
    for _ in range(int(num_trucks)):
        truck_id, location = [int(x) for x in sys.stdin.readline().split()]
        data['truck_location'][truck_id - 1] = location - 1
    # Vị trí bắt đầu và kết thúc của các xe
    data['starts'] = [data['truck_location'][i] for i in range(data['num_trucks'])]
    data['ends'] = [data['truck_location'][i] for i in range(data['num_trucks'])]

    data['truck_capacities'] = [40 for _ in range(data['num_trucks'])]


    data['num_req_required_trailer'] =0
    data['num_req_remaining_trailer_after_dropping'] = 0

    data['requests'] = {}

    # Đọc thông tin các yêu cầu
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

        # Lưu yêu cầu với các điểm mới (pickup là p1 và delivery là p2 + num_points)
        data['requests'][req_id] = {
            'size': size,
            'pickup_location': p1,
            'pickup_action': pickup_action,
            'pickup_duration': pickup_duration,
            'drop_location': p2 + num_points,  # Sử dụng chỉ số đã chuyển đổi
            'drop_action': drop_action,
            'drop_duration': drop_duration
        }

        if pickup_action == 'PICKUP_CONTAINER':
            data['num_req_required_trailer'] += 1

        if drop_action == 'DROP_CONTAINER':
            data['num_req_remaining_trailer_after_dropping'] += 1

    num_romooc_copies = data['num_req_required_trailer']  + data['num_req_remaining_trailer_after_dropping']# Số bản sao
    romooc_copies_indices = list(range(total_points, total_points + num_romooc_copies))
    total_points += num_romooc_copies
    data['total_points'] = total_points


    # Cập nhật khoảng cách cho các bản sao
    new_distance = [[float('inf')] * total_points for _ in range(total_points)]
    for i in range(len(distance)):
        for j in range(len(distance[i])):
            new_distance[i][j] = distance[i][j]

    # Đặt khoảng cách giữa romooc_location, romooc_location_numpoints và các bản sao
    romooc_location = data['Romooc_location']
    romooc_location_numpoints = romooc_location + num_points

    # Cập nhật khoảng cách từ các bản sao đến các điểm khác
    for copy_idx in romooc_copies_indices:
        for point in range(data['total_points']):
            # Nếu điểm không nằm trong danh sách các bản sao
            if point not in romooc_copies_indices and point != romooc_location and point != romooc_location_numpoints:
                # Khoảng cách từ bản sao đến điểm khác được sao chép từ romooc_location
                new_distance[copy_idx][point] = new_distance[data['Romooc_location']][point]
                new_distance[point][copy_idx] = new_distance[point][data['Romooc_location']]

    # Đặt khoảng cách giữa các bản sao với romooc_location và romooc_location_numpoints
    for copy_idx in romooc_copies_indices:
        new_distance[romooc_location][copy_idx] = time_max
        new_distance[copy_idx][romooc_location] = time_max

        new_distance[romooc_location_numpoints][copy_idx] = time_max
        new_distance[copy_idx][romooc_location_numpoints] = time_max

    # Đặt khoảng cách giữa các bản sao với nhau
    for i in romooc_copies_indices:
        for j in romooc_copies_indices:
            if i != j:
                new_distance[i][j] = time_max
            else:
                new_distance[i][j] = 0
    # Gán lại giá trị cập nhật
    data['delivery_time'] = new_distance

    #danh sách các điểm lấy romooc
    data['list_romooc_indices'] = romooc_copies_indices[:data['num_req_required_trailer']]
    data['list_romooc_indices'].append(romooc_location)
    data['list_romooc_indices'].append(romooc_location_numpoints)
    data['list_romooc_removing'] = romooc_copies_indices[data['num_req_required_trailer']:]

    data['checkPickUP'] = [False] * total_points
    data['checkDrop'] = [False] * total_points
    for req in data['requests'].values():
        if req['pickup_action'] == 'PICKUP_CONTAINER_TRAILER':
            data['checkPickUP'][req['pickup_location']] = True
        if req['drop_action'] == 'DROP_CONTAINER_TRAILER':
            data['checkDrop'][req['drop_location']] = True
    data['list_points_must_go'] = []
    for req in data['requests'].values():
        data['list_points_must_go'].append(req['pickup_location'])
        data['list_points_must_go'].append(req['drop_location'])

    data['demands'] = [0 for i in range(total_points)]
    for req in data['requests'].values():
        data['demands'][req['pickup_location']] = req['size']
        data['demands'][req['drop_location']] = -req['size']

    data['wait_time'] = [0 for i in range(total_points)]
    data['wait_time'][data['Romooc_location']] = data['Romooc_time']
    data['wait_time'][data['Romooc_location'] + num_points] = data['Romooc_time']
    for req in data['requests'].values():
        data['wait_time'][req['pickup_location']] = req['pickup_duration']
        data['wait_time'][req['drop_location']] = req['drop_duration']
    data['romooc_capacity'] = [1 for i in range(data['num_trucks'])]

    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    max_route_distance = 0
    capacity_dimension = routing.GetDimensionOrDie("Capacity")
    romooc_dimension = routing.GetDimensionOrDie("Romooc")
    time_dimension = routing.GetDimensionOrDie("Time")

    for vehicle_id in range(data['num_trucks']):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id + 1}:\n"
        total_time = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            load = solution.Value(capacity_dimension.CumulVar(index))
            romooc_state = solution.Value(romooc_dimension.CumulVar(index))
            plan_output += f"{node_index + 1} (Load:{load}, Romooc:{romooc_state}, total_time: {total_time}) -> "

            previous_index = index
            index = solution.Value(routing.NextVar(index))
            total_time += solution.Value(time_dimension.CumulVar(previous_index))
        node_index = manager.IndexToNode(index)
        load = solution.Value(capacity_dimension.CumulVar(index))
        romooc_state = solution.Value(romooc_dimension.CumulVar(index))
        total_time += solution.Min(time_dimension.CumulVar(index))
        plan_output += f"{node_index + 1} (Load:{load}, Romooc:{romooc_state})\n"

        plan_output += f"Time of the route: {total_time}m\n"
        print(plan_output)


def main():
    data = create_data_model()
    print(len(data['demands']))
    print(len(data['checkPickUP']))
    print(data['checkDrop'])
    print(data['Romooc_time'])
    # print(data['wait_time'])
    # print(data['num_req_required_trailer'])
    print(data['list_romooc_indices'])
    print(data['list_romooc_removing'])
    # print([i for i in range(data['num_points']*2, data['total_points'])])
    for i in range(len(data['delivery_time'])):
        print(data['delivery_time'][i])

    manager = pywrapcp.RoutingIndexManager(
        len(data['delivery_time']), data['num_trucks'], data['starts'], data['ends']
    )
    routing = pywrapcp.RoutingModel(manager)

    def time_callback(from_index, to_index):
        '''Returns the delivery-time between the two nodes.'''
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['delivery_time'][from_node][to_node]

    # print(time_callback(2, 5))
    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    time_name = "Time"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        time_max,  # vehicle maximum travel distance
        True,  # start cumul to zero
        time_name,
    )
    time_dimension = routing.GetDimensionOrDie(time_name)
    time_dimension.SetGlobalSpanCostCoefficient(100)

    # Add Pickup and Delivery constraints
    for req in data['requests'].values():
        pickup_index = manager.NodeToIndex(req['pickup_location'])
        delivery_index = manager.NodeToIndex(req['drop_location'])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        routing.solver().Add(routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))
        routing.solver().Add(time_dimension.CumulVar(pickup_index) <= time_dimension.CumulVar(delivery_index))

    solver = routing.solver()
    intervals = []
    for req in data['requests'].values():
        pickup_index = manager.NodeToIndex(req['pickup_location'])
        delivery_index = manager.NodeToIndex(req['drop_location'])

        pickup_interval = solver.FixedDurationIntervalVar(
            time_dimension.CumulVar(pickup_index),
            req['pickup_duration'],
            f"pickup_interval_{pickup_index}"
        )
        intervals.append(pickup_interval)

        delivery_interval = solver.FixedDurationIntervalVar(
            time_dimension.CumulVar(delivery_index),
            req['drop_duration'],
            f"delivery_interval_{delivery_index}"
        )
        intervals.append(delivery_interval)

    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    #
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

    romooc_name = "Romooc"

    def romooc_callback(to_index):
        to_node = manager.IndexToNode(to_index)
        if data['checkPickUP'][to_node]:
            data['checkPickUP'][to_node] = False
            return 1
        if data['checkDrop'][to_node]:
            data['checkDrop'][to_node] = False
            return -1
        if to_node in data['list_romooc_indices']:
            return 1
        if to_node in data['list_romooc_removing']:
            return -1

        return 0

    romooc_evaluator_index = routing.RegisterUnaryTransitCallback(romooc_callback)

    routing.AddDimensionWithVehicleCapacity(
        romooc_evaluator_index,
        0,
        data['romooc_capacity'],
        True,
        romooc_name
    )
    romooc_dimension = routing.GetDimensionOrDie(romooc_name)

    for i in data['list_romooc_indices']:
        node_index = manager.NodeToIndex(i)
        romooc_interval = solver.FixedDurationIntervalVar(
            time_dimension.CumulVar(node_index),
            data['Romooc_time'],
            f"romooc_interval_indices_{i}"
        )
        intervals.append(romooc_interval)
    for i in data['list_romooc_removing']:
        node_index = manager.NodeToIndex(i)
        romooc_interval = solver.FixedDurationIntervalVar(
            time_dimension.CumulVar(node_index),
            data['Romooc_time'],
            f"romooc_interval_removing_{i}"
        )
        intervals.append(romooc_interval)

    # for i in data['list_romooc_indices']:
    #     routing.AddDisjunction([manager.NodeToIndex(i)], 0)
    # for i in data['list_romooc_removing']:
    #     routing.AddDisjunction([manager.NodeToIndex(i)], 0)
    for i in range(data['total_points']):
        if i not in data['list_points_must_go']:
            routing.AddDisjunction([manager.NodeToIndex(i)], 600)


    for req in data['requests'].values():
        pickup_index = manager.NodeToIndex(req['pickup_location'])
        if req['pickup_action'] == 'PICKUP_CONTAINER':
            routing.solver().Add(romooc_dimension.CumulVar(pickup_index) == 1)
        if req['pickup_action'] == 'PICKUP_CONTAINER_TRAILER':
            routing.solver().Add(romooc_dimension.CumulVar(pickup_index) == 0)

    # before end routing model all vehicle must return to romooc location
    for vehicle_id in range(data['num_trucks']):
        index = routing.End(vehicle_id)
        # romooc_state = routing.VehicleVar(index)
        routing.solver().Add(romooc_dimension.CumulVar(index) == 1)


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

    # print(routing.DebugOutputAssignment(solution,dimension_name))
    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print('No solution found !')


if __name__ == "__main__":
    main()
