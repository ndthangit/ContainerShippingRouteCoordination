import sys
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

def printSolution(data, manager, routing, solution):
    print(f"ROUTES {data['num_trucks']}")
    for vehicle_id in range(data['num_trucks']):
        index = routing.Start(vehicle_id)
        plan_output = f"TRUCK {vehicle_id + 1}\n"
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            next_node_index = manager.IndexToNode(routing.NextVar(index))
            action = "STOP"  # Default action if no specific action is found
            request_id = -1  # Default request_id if no specific request is found

            # Determine the action and request_id based on the node_index
            for req_id, req in data['requests'].items():
                if req['pickup_location'] == node_index:
                    action = req['pickup_action']
                    request_id = req_id + 1
                elif req['drop_location'] == node_index:
                    action = req['drop_action']
                    request_id = req_id + 1

            plan_output += f"{node_index + 1} {action}"
            if action != "STOP":
                plan_output += f" {request_id}"
            plan_output += "\n"
            index = routing.NextVar(index)

        plan_output += "#\n"
        print(plan_output)


# data = create_data_model()
# print(data)
# print(len(data['distance'][0]))
