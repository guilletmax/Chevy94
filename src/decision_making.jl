
const CROSSED_CONFIRMED_COUNT = 5
const v_step = 1.0
const s_step = 0.314


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
    curr_segment = -1
    path = []

    is_setup = false
    while !is_setup
        sleep(0.01)
        if (isready(localization_state_channel))
            x = take!(localization_state_channel)

            """TODO"""
            # STEP 1 -> fix get_segments_from_localization - maybe fixed! could produce bugs once we get the car moving though.
            curr_segments = get_segments_from_localization(x.position[1], x.position[2], map)
            curr_segment = curr_segments[1]

            """TODO"""
            # STEP 2 -> fix get_path - maybe fixed!
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
            is_setup = true
        end
    end

    #@info x.position[1]
    #@info x.position[2]
    #@info curr_segment
    #@info path
    #@info "done with setup"

    next_path_index = 2
    epsilon = 0.1
    crossed_segment_count = 0
    controls.target_speed = 4.0

    pid_state_straight = PIDState(0.0, 0.0, 0.0)
    pid_state_curved = PIDState(0.0, 0.0, 0.0)
	
	stopped = false # tells us if we have ever stopped at any point in the segment. Useful for stop signs
    
	@async while isopen(socket)
        sleep(0.005)

        if curr_segment.id == target_road_segment_id
            controls.target_speed = 0
            break
        end

		if Int(curr_segment.lane_types[1]) == 3
			distance_to_stop_sign = -1
			if curr_segment.lane_boundaries[1].pt_b[1] == curr_segment.lane_boundaries[2].pt_b[1]
				distance_to_stop_sign = abs(x.position[1] - curr_segment.lane_boundaries[1].pt_b[1])
			else
            	m, b = find_line_equation(curr_segment.lane_boundaries[1].pt_b, curr_segment.lane_boundaries[2].pt_b)
            	distance_to_stop_sign = abs(m * x.position[1] - x.position[2] + b) / sqrt(m^2 + 1)
			end
			if distance_to_stop_sign < 10.0 && !stopped
				controls.target_speed = 0
				sleep(2)
				controls.target_speed = 4
				stopped = true
			end
		else
			stopped = false
		end

        if (isready(localization_state_channel))
            x = take!(localization_state_channel)
            #@info "x_position: $(x.position[1])"
            #@info "y_position: $(x.position[2])"

            """TODO"""
            # STEP 4 -> see if we can get this working, supposed to verify when we cross into a new segment 
            #           and update curr_seg and crossed_segment_index
            segments = get_segments_from_localization(x.position[1], x.position[2], map)

            for seg in segments
                #@info "considering seg $(seg.id)"
                if seg.id == path[next_path_index]
					#stopped = false
                    curr_segment = seg
                    next_path_index += 1
                    #@info "chose $(curr_segment.id)"
                    break
                end
            end

            #@info "curr segment: $(curr_segment.id)"
            #@info "desired next segment: $(path[next_path_index])"

            #@info "curr_segment: $(curr_segment.id)"
            #@info "new_segments: $(new_segments)"

            # if !(curr_segment in new_segments)
            #     #@info "new segment found"
            #     if (crossed_segment_count < CROSSED_CONFIRMED_COUNT)
            #         #@info "crossed_segment_count: $crossed_segment_count"
            #         crossed_segment_count += 1
            #     else
            #         curr_segment = map[path[next_path_index]]
            #         #@info "new curr_segment: $curr_segment"
            #         # sanity check: checks that the next segment in the path is actually one of the one's our get_segment function found - syntax issues rn, not working
            #         #if !(curr_segment.id in map(new_segment -> new_segment.id, new_segments))
            #         #    @info "curr_segment not in new_segments"
            #         #end
            #         next_path_index += 1
            #         crossed_segment_count = 0
            #     end
            # end

            #@info "curr_segment: $(curr_segment.id)"
            # STEP 5 -> fix get_steering_angle - maybe fixed!
            update_steering_angle(controls, x.position[1], x.position[2], curr_segment.lane_boundaries, pid_state_straight, pid_state_curved)

            # STEP 6 -> fix get_target_speed
            # controls.target_speed = get_target_speed(controls.target_speed, curr_seg.speed_limit)
            # controls.target_speed = 8.0 #comment me out when ready

            #@info "target speed: $(controls.target_speed)"
            #@info "steering angle: $(controls.steering_angle)"
        end

        # NOTE: HOLD OFF ON PERCEPTION RELATED STUFF, lets figure out navigating with ground truth first
        # p_state = []
        # if (isready(perception_state_channel))
        #     p_state = fetch(perception_state_channel)
        # end

    end

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
Update steering_angle if we deviate from the center localization_state. Lane_boundaries is a vector of the current segment's lane boundaries. 
"""
function update_steering_angle(controls, x, y, lane_boundaries, pid_state_straight, pid_state_curved)
    lane_curve = lane_boundaries[1].curvature != 0 # True if curved, false if straight, 0 if straight, negative if curve right, positive if curve left. 
    #@info "in function"
    lane_boundaries_left = lane_boundaries[1]
    lane_boundaries_right = lane_boundaries[2]

    kp = 0.5 # proportional gain
    ki = 0.01 # integral gain
    kd = 20 # derivative gain
    max_control_input = pi / 4

    #@info x
    #@info y
    if !lane_curve
        controls.target_speed = 5
        center_point = (lane_boundaries_left.pt_a + lane_boundaries_right.pt_a) / 2
        normal_vector = lane_boundaries_right.pt_a - center_point
        normal_vector /= norm(normal_vector)

        a = dot(normal_vector, [x; y])
        b = dot(normal_vector, center_point)


        # curr_angle = controls.steering_angle
        # curr_speed = controls.target_speed #maybe update this to localization later
        # vel_vec = [curr_speed * cos(curr_angle), curr_speed * sin(curr_angle)]
        # vel_towards_center = dot(-normal_vector, vel_vec)


        # left of center
        # - faster we are approcahing center == larger this value is curr_speed * cos(curr_angle) == less right we turn == more left we turn == increment steering angle
        # right of center
        # - faster we are approcahing center == smaller this value is curr_speed * cos(curr_angle) == less left we turn == more right we turn == decrement steering angle

        # horizontal_velocity = controls.target_speed * sin(controls.steering_angle)
        #speed_dampening_factor = -0.0005

        # if a != b
        #     controls.steering_angle = 0.005 * (a - b) #+ (horizontal_velocity * speed_dampening_factor)
        # else
        #     controls.steering_angle = 0
        # end

        #@info pid_state_straight
        #@info a - b
        pid_state_straight.error = a - b
        #@info pid_state_straight.error
        control_input = pid_controller(pid_state_straight, kp, ki, kd, max_control_input)
        #@info control_input
        #@info pid_state_straight

        controls.steering_angle = control_input

    else
        controls.target_speed = 3
        #@info "sup"
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


        # pid_state_curved.error = dist_to_center
        # @info pid_state_curved.error
        # control_input = pid_controller(pid_state_curved, kp, ki, kd, max_control_input)
        # @info control_input
        # @info pid_state_curved

        # controls.steering_angle = control_input


        controls.steering_angle = angle
    end
end


"""
PID controller
"""
function pid_controller(pid_state, kp, ki, kd, max_control_input)
    proportional_term = kp * pid_state.error
    #@info proportional_term
    integral_term = ki * (pid_state.integral_error + pid_state.error)
    #@info integral_term
    derivative_term = kd * (pid_state.error - pid_state.prev_error)
    #@info derivative_term
    control_input = proportional_term + integral_term + derivative_term
    #@info control_input
    if abs(control_input) > max_control_input
        #@info "here"
        integral_term = 0
        control_input = sign(control_input) * max_control_input
        #@info control_input
    end
    #@info "outside loop"
    pid_state.prev_error = pid_state.error
    #@info pid_state.prev_error
    pid_state.integral_error += pid_state.error
    #@info pid_state.integral_error

    return control_input
end


"""
Given two coordinates, returns the equation of the line in y = mx + b form. 
Note: tested and should work as intended
"""
function find_line_equation(coord1, coord2)
    # Unpack coordinates
    x1, y1 = coord1
    x2, y2 = coord2

    # Calculate slope (m)
    m = (y2 - y1) / (x2 - x1)

    # Calculate y-intercept (b)
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


