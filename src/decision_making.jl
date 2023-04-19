
const default_speed = 8.0
const turn_speed = 5.0


"""
PID state
"""
mutable struct PIDState
    error::Float64
    prev_error::Float64
    integral_error::Float64
end


"""
@KEV DO COMMENTS BRUH
"""
function decision_making(localization_state_channel,
    perception_state_channel,
    map,
    socket,
    controls)

    """REMOVE ME EVENTUALLY"""
    target_road_segment_id = 94
    @info "Target: $target_road_segment_id"


    x = -1
    p = -1
    curr_segment = -1
    path = []

    is_setup = false
    while !is_setup
        sleep(2) # worked with gt at .05 sleep

        if (isready(localization_state_channel))
            x = take!(localization_state_channel)

            @info "localization state: $x"

            # GET SEGMENTS FROM LOCALIZATION DOESN'T ALWAYS WORK (think breaks on 101) NEED TO FIX
            curr_segments = get_segments_from_localization(x.position[1], x.position[2], map)
            curr_segment = curr_segments[1]

            shortest_path = shortest_path_bfs(map, curr_segments[1].id, target_road_segment_id)
            min_dist = length(shortest_path)
            for i in eachindex(curr_segments[2:end])
                new_path = shortest_path_bfs(map, curr_segments[1].id, target_road_segment_id)
                new_dist = lenth(new_path)
                if new_dist < min_dist
                    shortest_path = new_path
                    min_dist = new_dist
                    curr_segment = curr_segments[i]
                end
            end

            path = shortest_path
            @info path
            is_setup = true
        end
    end


    next_path_index = 2
    pid_state_straight = PIDState(0.0, 0.0, 0.0)
    stopped = false
    target_speed = default_speed
    steering_angle = 0.0

    @async while isopen(socket)
        sleep(0.001)

        is_localization_updated = false
        is_perception_updated = false
        if (isready(localization_state_channel))
            x = take!(localization_state_channel)
            is_localization_updated = true
        end
        if (isready(perception_state_channel))
            p = take!(perception_state_channel)
            is_perception_updated = true
        end



        if (is_localization_updated)
            next_segment = map[path[next_path_index]]
            if in_segment(x.position[1], x.position[2], next_segment)
                curr_segment = next_segment
                next_path_index += 1
                @info "curr segment: $(curr_segment)"
            end

            if VehicleSim.stop_sign in curr_segment.lane_types
                distance_to_stop_sign = -1
                if curr_segment.lane_boundaries[1].pt_b[1] == curr_segment.lane_boundaries[2].pt_b[1]
                    distance_to_stop_sign = abs(x.position[1] - curr_segment.lane_boundaries[1].pt_b[1])
                else
                    m, b = find_line_equation(curr_segment.lane_boundaries[1].pt_b, curr_segment.lane_boundaries[2].pt_b)
                    distance_to_stop_sign = abs(m * x.position[1] - x.position[2] + b) / sqrt(m^2 + 1)
                end
                if distance_to_stop_sign < 13.0 && !stopped
                    controls.target_speed = 0
                    sleep(2)
                    controls.target_speed = default_speed
                    stopped = true
                end
            end

            # pull out
            if curr_segment.id == target_road_segment_id && VehicleSim.loading_zone in curr_segment.lane_types
                controls.target_speed = 3.5
                controls.steering_angle = -0.4
                sleep(3.5)
                controls.steering_angle = 0.4
                sleep(3.5)
                controls.steering_angle = 0.0
                controls.target_speed = 0.0
                break
            else
                target_speed, steering_angle = update_steering(x.position[1], x.position[2], curr_segment.lane_boundaries, pid_state_straight)
            end
        end

        if (is_perception_updated)
            # [p1, p2, heading, velocity, height, lenght, width], cov]

            our_vel = [curr_speed * sin(curr_angle), curr_speed * cos(curr_angle)]
            enemy_speed = p[3]
            enemy_angle = p[4]
            enemy_vel = [enemy_speed * sin(enemy_angle), enemy_speed * cos(enemy_angle)]

            epsilon = 0.1

            if dot(our_vel, enemy_vel) - norm(our_vel) * norm(enemy_vel) < epsilon
                target_speed = p[4] - 0.5
            else
                sleep(rand(1.0:0.1:5.0))
                target_speed = 0
                sleep(rand(1.0:0.1:5.0))
                target_speed = 0.5
                sleep(rand(1.0:0.1:5.0))
                target_speed = default_speed
            end
        end


        if (is_localization_updated || is_perception_updated)
            controls.target_speed = target_speed
            controls.steering_angle = steering_angle
        end
    end
end


"""
Returns boolean indicating that we are in segment
"""
function in_segment(x, y, segment)

    x_values = []
    y_values = []
    for b in segment.lane_boundaries
        push!(x_values, b.pt_a[1], b.pt_b[1])
        push!(y_values, b.pt_a[2], b.pt_b[2])
    end

    x_min = minimum(x_values)
    y_min = minimum(y_values)
    x_max = maximum(x_values)
    y_max = maximum(y_values)

    if (x >= x_min && x <= x_max && y >= y_min && y <= y_max)
        return true
    end
    return false
end


"""
Get segment ID from localization x and y state and map
"""
function get_segments_from_localization(x, y, map)
    segments = []
    for (id, segment) in map
        lane_boundaries_left = segment.lane_boundaries[1]
        lane_boundaries_right = segment.lane_boundaries[2]

        if (segment.lane_boundaries[1].curvature == 0)
            left_normal_vector = lane_boundaries_right.pt_a - lane_boundaries_left.pt_a
            inside_left_boundary = dot(left_normal_vector, [x; y]) >= dot(left_normal_vector, lane_boundaries_left.pt_a)
            right_normal_vector = lane_boundaries_left.pt_a - lane_boundaries_right.pt_a
            inside_right_boundary = dot(right_normal_vector, [x; y]) >= dot(right_normal_vector, lane_boundaries_right.pt_a)
            start_normal_vector = lane_boundaries_left.pt_b - lane_boundaries_left.pt_a
            inside_start_boundary = dot(start_normal_vector, [x; y]) >= dot(start_normal_vector, lane_boundaries_left.pt_a)
            end_normal_vector = lane_boundaries_left.pt_a - lane_boundaries_left.pt_b
            inside_end_boundary = dot(end_normal_vector, [x; y]) >= dot(end_normal_vector, lane_boundaries_left.pt_b)
            if (inside_left_boundary && inside_right_boundary && inside_start_boundary && inside_end_boundary)
                if (VehicleSim.intersection in segment.lane_types)
                    push!(segments, segment)
                else
                    return [segment]
                end
            end
        else
            left_r = abs(1 / lane_boundaries_left.curvature)
            right_r = abs(1 / lane_boundaries_right.curvature)
            big_radius = -1.0
            small_radius = -1.0

            if left_r < right_r
                small_center_one, small_center_two = find_circle_center(lane_boundaries_left.pt_a, lane_boundaries_left.pt_b, left_r)
                big_radius = right_r
                small_radius = left_r
            else
                small_center_one, small_center_two = find_circle_center(lane_boundaries_right.pt_a, lane_boundaries_right.pt_b, right_r)
                big_radius = left_r
                small_radius = right_r
            end

            if norm([x; y] - small_center_one) < norm([x; y] - small_center_two)
                circle_center = small_center_two
            else
                circle_center = small_center_one
            end

            dist_to_center = norm([x; y] - circle_center)
            inside_curves = dist_to_center >= small_radius && dist_to_center <= big_radius
            start_normal_vector = lane_boundaries_left.pt_b - lane_boundaries_left.pt_a
            inside_start_boundary = dot(start_normal_vector, [x; y]) >= dot(start_normal_vector, lane_boundaries_left.pt_a)
            end_normal_vector = lane_boundaries_left.pt_a - lane_boundaries_left.pt_b
            inside_end_boundary = dot(end_normal_vector, [x; y]) >= dot(end_normal_vector, lane_boundaries_left.pt_b)
            if (inside_curves && inside_start_boundary && inside_end_boundary)
                if (VehicleSim.intersection in segment.lane_types)
                    push!(segments, segment)
                else
                    return [segment]
                end
            end
        end
    end
    return segments
end

"""
Update steering if we deviate from the center localization_state. Lane_boundaries is a vector of the current segment's lane boundaries. 
"""
function update_steering(x, y, lane_boundaries, pid_state_straight)
    lane_boundaries_left = lane_boundaries[1]
    lane_boundaries_right = lane_boundaries[2]

    kp = 0.1 # proportional gain
    ki = 0.01 # integral gain
    kd = 15 # derivative gain
    max_control_input = pi / 4

    if lane_boundaries[1].curvature == 0
        center_point = (lane_boundaries_left.pt_a + lane_boundaries_right.pt_a) / 2
        normal_vector = lane_boundaries_right.pt_a - center_point
        normal_vector /= norm(normal_vector)

        a = dot(normal_vector, [x; y])
        b = dot(normal_vector, center_point)

        pid_state_straight.error = a - b
        control_input = pid_controller(pid_state_straight, kp, ki, kd, max_control_input)
        return default_speed, control_input
    else
        left_r = abs(1 / lane_boundaries_left.curvature)
        right_r = abs(1 / lane_boundaries_right.curvature)

        big_radius = -1.0
        small_radius = -1.0
        right_turn = false

        if left_r < right_r
            small_center_one, small_center_two = find_circle_center(lane_boundaries_left.pt_a, lane_boundaries_left.pt_b, left_r)
            big_radius = right_r
            small_radius = left_r
        else
            small_center_one, small_center_two = find_circle_center(lane_boundaries_right.pt_a, lane_boundaries_right.pt_b, right_r)
            big_radius = left_r
            small_radius = right_r
            right_turn = true
        end

        circle_center = [0, 0]
        if norm([x; y] - small_center_one) < norm([x; y] - small_center_two)
            circle_center = small_center_two
        else
            circle_center = small_center_one
        end

        lane_center_radius = (left_r + right_r) / 2
        dist_to_center = norm([x; y] - circle_center)

        angle_nominal = 1.1 * atan(7.75 / lane_center_radius)
        angle = angle_nominal
        if right_turn
            angle *= -1
            angle += 0.1 * (lane_center_radius - dist_to_center)
        else
            angle -= 0.1 * (lane_center_radius - dist_to_center)
        end

        return turn_speed, angle
    end
end


"""
PID controller
"""
function pid_controller(pid_state, kp, ki, kd, max_control_input)
    proportional_term = kp * pid_state.error
    integral_term = ki * (pid_state.integral_error + pid_state.error)
    derivative_term = kd * (pid_state.error - pid_state.prev_error)

    control_input = proportional_term + integral_term + derivative_term
    if abs(control_input) > max_control_input
        integral_term = 0
        control_input = sign(control_input) * max_control_input
    end
    pid_state.prev_error = pid_state.error
    pid_state.integral_error += pid_state.error

    return control_input
end


"""
Given two coordinates, returns the equation of the line in y = mx + b form. 
Note: tested and should work as intended
"""
function find_line_equation(coord1, coord2)
    x1, y1 = coord1
    x2, y2 = coord2
    m = (y2 - y1) / (x2 - x1)
    b = y1 - m * x1

    return [m, b]
end

"""
Finds the center coordinates of an implied circle given 2 xy coordinates of points on the circumference and the circle's radius. 
"""
function find_circle_center(p1, p2, r)
    q = sqrt((p2[1] - p1[1])^2 + (p2[2] - p1[2])^2)
    avg = (p1 + p2) / 2

    # There are 2 possible circles to go through 2 points. We will need to figure out how to choose which one we want
    center_one = [avg[1] + sqrt(r^2 - (q / 2)^2) * (p1[2] - p2[2]) / q
        avg[2] + sqrt(r^2 - (q / 2)^2) * (p2[1] - p1[1]) / q]

    center_two = [avg[1] - sqrt(r^2 - (q / 2)^2) * (p1[2] - p2[2]) / q
        avg[2] - sqrt(r^2 - (q / 2)^2) * (p2[1] - p1[1]) / q]

    return center_one, center_two
end

"""
Update the velocity of the vehicle so that we are as close to the speed limit as possible
TODO: Eventually optimize
"""
function get_target_speed(target_speed, speed_limit)
    # target_speed = speed_limit
    return 3.0
end

"""
Find shortest path from start_segment_id to end_segment_id
"""
# function shortest_path(start_segment, end_segment)
function shortest_path_bfs(map, start_segment_id, end_segment_id)
    # initialize the visited set and the queue
    visited = Set()
    queue = Deque{Any}(10)

    # add the initial path to the queue
    push!(queue, [start_segment_id])

    # loop through the queue until it's empty
    while !isempty(queue)
        # get the first path from the queue
        path = popfirst!(queue)

        # get the last segment id from the path
        curr_segment_id = last(path)

        # check if we've reached the goal
        if curr_segment_id == end_segment_id
            return path
        end

        # check if we've already visited this segment
        if !(curr_segment_id in visited)
            # mark the segment as visited
            push!(visited, curr_segment_id)

            # get the children of the current segment
            children = map[curr_segment_id].children

            # loop through the children of the current segment
            for child in children
                # create a new path by appending the child to the current path
                new_path = copy(path)
                push!(new_path, child)

                # add the new path to the queue
                push!(queue, new_path)
            end
        end
    end

    # if we reach this point, there is no path from start to goal
    return nothing
end
# end


