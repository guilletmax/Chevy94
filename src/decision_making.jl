
const CROSSED_CONFIRMED_COUNT = 0
const v_step = 1.0
const s_step = 0.314

"""
@KEV DO COMMENTS BRUH
"""
function decision_making(localization_state_channel,
    perception_state_channel,
    map,
    socket,
    controls)

    """REMOVE ME EVENTUALLY"""
    target_road_segment_id = 44


    x = -1
    curr_segment = -1
    path = []

    is_setup = false
    while !is_setup
        sleep(0.01)
        if (isready(localization_state_channel))
            x = take!(localization_state_channel)
            @info x

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

    @info x.position[1]
    @info x.position[2]
    @info curr_segment
    @info path
    @info "done with setup"

    next_path_index = 2
    epsilon = 0.1
    crossed_segment_count = 0

    @async while isopen(socket)
        sleep(0.01)

        if (isready(localization_state_channel))
            x = take!(localization_state_channel)
            @info "x_position: $(x.position[1])"
            @info "y_position: $(x.position[2])"

            """TODO"""
            # STEP 4 -> see if we can get this working, supposed to verify when we cross into a new segment 
            #           and update curr_seg and crossed_segment_index
            new_segments = get_segments_from_localization(x.position[1], x.position[2], map)

            @info "curr_segment: $(curr_segment.id)"
            @info "new_segments: $(new_segments)"

            if (new_segments[1] != curr_segment)
                @info "new segment found"
                if (crossed_segment_count < CROSSED_CONFIRMED_COUNT)
                    @info "crossed_segment_count: $crossed_segment_count"
                    crossed_segment_count += 1
                else
                    curr_segment = map[path[next_path_index]]
                    @info "new curr_segment: $curr_segment"
                    # sanity check: checks that the next segment in the path is actually one of the one's our get_segment function found - syntax issues rn, not working
                    #if !(curr_segment.id in map(new_segment -> new_segment.id, new_segments))
                    #    @info "curr_segment not in new_segments"
                    #end
                    next_path_index += 1
                    crossed_segment_count = 0
                end
            end

            @info curr_segment
            # STEP 5 -> fix get_steering_angle - maybe fixed!
            controls.steering_angle = get_steering_angle(controls.steering_angle, x.position[1], x.position[2], curr_segment.lane_boundaries, epsilon)

            # STEP 6 -> fix get_target_speed
            # controls.target_speed = get_target_speed(controls.target_speed, curr_seg.speed_limit)
            controls.target_speed = 2.0 #comment me out when ready

            @info "target speed: $(controls.target_speed)"
            @info "steering angle: $(controls.steering_angle)"
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
function get_steering_angle(steering_angle, x, y, lane_boundaries, epsilon)
    lane_curve = lane_boundaries[1].curvature != 0 # True if curved, false if straight, 0 if straight, negative if curve right, positive if curve left. 
    @info "in function"
    lane_boundaries_left = lane_boundaries[1]
    lane_boundaries_right = lane_boundaries[2]

    @info x
    @info y
    if !lane_curve
        normal_vector = lane_boundaries_right.pt_a - lane_boundaries_left.pt_a
        center_point = (lane_boundaries_left.pt_a + lane_boundaries_right.pt_a) / 2

        a = dot(normal_vector, [x; y])
        b = dot(normal_vector, center_point)
        @info a
        @info b
        if a > b
            angle = abs(a - b) * 0.001
        elseif a < b
            angle = -1 * abs(a - b) * 0.001
        else
            angle = 0
        end
        return angle
    else
        @info "sup"
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
        @info circle_center

        lane_center_radius = (left_r + right_r) / 2
        @info lane_center_radius
        dist_to_center = norm([x; y] - circle_center)
        @info dist_to_center
        @info "distance to end: "
        @info "avg: $((lane_boundaries_left.pt_b + lane_boundaries_right.pt_b) / 2)"
        dist_end = norm([x; y] - ((lane_boundaries_left.pt_b + lane_boundaries_right.pt_b) / 2))
        @info dist_end

        #angle = asin(dist_end / (2 * lane_center_radius))
        angle = lane_boundaries_left.curvature
        @info angle
        # if we are turning too tightly
        if lane_center_radius > dist_to_center
            if right_turn
                angle += abs(lane_center_radius - dist_to_center) * 20
            else
                angle -= abs(lane_center_radius - dist_to_center) * 20
            end
        else
            if right_turn
                angle -= abs(lane_center_radius - dist_to_center) * 20
            else
                angle += abs(lane_center_radius - dist_to_center) * 20
            end
        end
        @info "angle: $angle"
        return angle
    end
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


