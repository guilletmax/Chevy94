
const CROSSED_CONFIRMED_COUNT = 10
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

    curr_segments = []
    path = []

    is_setup = false
    while !is_setup
        sleep(0.001)
        if (isready(localization_state_channel))
            x = fetch(localization_state_channel)
            @info x

            """TODO"""
            # STEP 1 -> fix get_segments_from_localization - maybe fixed! could produce bugs once we get the car moving though.
            curr_segments = get_segments_from_localization(x.position[1], x.position[2], map)
            @info curr_segments

            """TODO"""
            # STEP 2 -> fix get_path - STATUS: for me, going from seg 32 (starting segment of car) -> seg 44 (random target segment) results in julia program stalling indefinitley.
            @info curr_segments[1]
            @info target_road_segment_id
            best_path = get_path(map, curr_segments[1], target_road_segment_id)
            @info best_path
            min_dist = length(best_path)
            @info min_dist
            for i in eachindex(curr_segments[2:end])
                @info i
                new_path = get_path(map, curr_segments[i], target_road_segment_id)
                @info new_path
                new_dist = lenth(new_path)
                @info new_dist
                if new_dist < min_dist
                    best_path = new_dist
                    min_dist = new_dist
                end
            end

            @info best_path
            path = best_path
            is_setup = true
        end
    end

    next_path_index = 2
    epsilon = 0.1
    crossed_segment_count = 0

    @async while isopen(socket)
        sleep(0.5)

        """TODO"""
        # STEP 3 -> make sure this causes no issues when used in the loop. localization_state_channel 
        #           should always be filled here, i don't think you no need to check if it's ready
        # x = fetch(localization_state_channel)
        # curr_seg = get_segment_from_localization(x[1], x[2], map)
        # @info curr_seg

        # NOTE: HOLD OFF ON PERCEPTION RELATED STUFF, lets figure out navigating with ground truth first
        # p_state = []
        # if (isready(perception_state_channel))
        #     p_state = fetch(perception_state_channel)
        # end

        """TODO"""
        # STEP 4 -> see if we can get this working, supposed to verify when we cross into a new segment 
        #           and update curr_seg and crossed_segment_index
        # if (get_segment_from_localization(x[1], x[2], map) != curr_seg)
        #     if (crossed_segment_count < CROSSED_CONFIRMED_COUNT)
        #         crossed_segment_count+=1
        #     else
        #         curr_seg = path[next_path_index]
        #         next_path_index+= 1
        #         crossed_segment_count = 0
        # 	end
        # end

        """TODO"""
        # STEP 5 -> fix get_steering_angle, note* we want to return the new steering angle from the function and set it here
        # controls.steering_angle = get_steering_angle(controls.steering_angle, x[1], x[2], curr_seg.lane_boundaries, epsilon)
        controls.steering_angle = 0.314 #comment me out when ready

        """TODO"""
        # STEP 6 -> fix get_target_speed, note* we want to return the new speed from the function and set it here
        # controls.target_speed = get_target_speed(controls.target_speed, curr_seg.speed_limit)
        controls.target_speed = 3.0 #comment me out when ready

        @info "decision_making.jl"
        @info "target speed $(controls.target_speed)"
        @info "steering angle $(controls.steering_angle)"
    end

end

"""
Get segment ID from localization x and y state and map
"""
function get_segments_from_localization(x, y, map)
    @info x
    @info y
    @info map
    for (id, segment) in map
        @info id
        @info segment
        lane_boundaries_left = segment.lane_boundaries[1]
        lane_boundaries_right = segment.lane_boundaries[2]
        @info lane_boundaries_left
        @info lane_boundaries_right
        @info segment.lane_types
        is_intersection = intersection in segment.lane_types
        @info is_intersection
        segments = []

        if (segment.lane_boundaries[1].curvature == 0)
            @info "is straight"
            left_normal_vector = lane_boundaries_right.pt_a - lane_boundaries_left.pt_a
            inside_left_boundary = dot(left_normal_vector, [x; y]) >= dot(left_normal_vector, lane_boundaries_left.pt_a)
            @info inside_left_boundary
            right_normal_vector = lane_boundaries_left.pt_a - lane_boundaries_right.pt_a
            inside_right_boundary = dot(right_normal_vector, [x; y]) >= dot(right_normal_vector, lane_boundaries_right.pt_a)
            @info inside_right_boundary
            start_normal_vector = lane_boundaries_left.pt_b - lane_boundaries_left.pt_a
            inside_start_boundary = dot(start_normal_vector, [x; y]) >= dot(start_normal_vector, lane_boundaries_left.pt_a)
            @info inside_start_boundary
            end_normal_vector = lane_boundaries_left.pt_a - lane_boundaries_left.pt_b
            inside_end_boundary = dot(end_normal_vector, [x; y]) >= dot(end_normal_vector, lane_boundaries_left.pt_b)
            @info inside_end_boundary
            if (inside_left_boundary && inside_right_boundary && inside_start_boundary && inside_end_boundary) # if within both boundaries
                @info "found segment"
                if (is_intersection)
                    push!([id, segment], segments)
                    @info "adding to segments:"
                    @info segments
                else
                    @info "returning"
                    return [[id, segment]]
                end
            end
        else
            @info "is curved"
            left_r = abs(1 / lane_boundaries_left.curvature)
            right_r = abs(1 / lane_boundaries_right.curvature)
            @info left_r
            @info right_r

            big_radius = -1.0
            small_radius = -1.0

            if left_r < right_r
                small_center_one, small_center_two = find_circle_center(lane_boundaries_left.pt_a, lane_boundaries_left.pt_b, left_r)
                big_radius = right_r
                small_radius = left_r
                @info "left turn"
                @info "small_radius"
                @info "big_radius"
            else
                small_center_one, small_center_two = find_circle_center(lane_boundaries_right.pt_a, lane_boundaries_right.pt_b, right_r)
                big_radius = left_r
                small_radius = right_r
                @info "right turn"
                @info "small_radius"
                @info "big_radius"
            end

            if norm([x; y] - small_center_one) < norm([x; y] - small_center_two)
                circle_center = small_center_two
            else
                circle_center = small_center_one
            end

            dist_to_center = norm([x; y] - circle_center)
            inside_curves = dist_to_center >= small_radius && dist_to_center <= big_radius
            @info inside_curves

            start_normal_vector = lane_boundaries_left.pt_b - lane_boundaries_left.pt_a
            inside_start_boundary = dot(start_normal_vector, [x; y]) >= dot(start_normal_vector, lane_boundaries_left.pt_a)
            @info inside_start_boundary

            end_normal_vector = lane_boundaries_left.pt_a - lane_boundaries_left.pt_b
            inside_end_boundary = dot(end_normal_vector, [x; y]) >= dot(end_normal_vector, lane_boundaries_left.pt_b)
            @info inside_end_boundary

            if (inside_curves && inside_start_boundary && inside_end_boundary)
                @info "segment found"
                if (is_intersection)
                    push!([id, segment], segments)
                    @info "adding to segments:"
                    @info segments
                else
                    @info "returning"
                    return [[id, segment]]
                end
            end
        end
    end
    @info "for loop complete:"
    @info "segments"
    return segments
end

"""
Update steering_angle if we deviate from the center localization_state. Lane_boundaries is a vector of the current segment's lane boundaries. 
"""
function get_steering_angle(steering_angle, curr_x, curr_y, lane_boundaries, epsilon)
    edge1_coord_start = lane_boundaries[1].pt_a
    edge1_coord_end = lane_boundaries[1].pt_b
    edge2_coord_start = lane_boundaries[2].pt_a
    edge2_coord_end = lane_boundaries[2].pt_b
    lane_curve = lane_boundaries[1].curvature != 0 # True if curved, false if straight, 0 if straight, negative if curve right, positive if curve left. 

    # middle_coord_start and middle_coord_end are the end points of the line/curve that is the middle of the lane
    middle_coord_start = [edge1_coord_start[1] - edge2_coord_start[1], edge1_coord_start[2] - edge2_coord_start[2]]
    middle_coord_end = [edge1_coord_end[1] - edge2_coord_end[1], edge1_coord_end[2] - edge2_coord_end[2]]
    middle_curvature = (abs(abs(lane_boundaries[1].curvature) - abs(lane_boundaries[2].curvature)) / 2) + min(abs(lane_boundaries[1].curvature), abs(lane_boundaries[2].curvature))

    if (lane_curve) # If lane is curved
        # x_shift and y_shift are the x and y coordinates of the center of the circle
        x_shift, y_shift = find_circle_center(middle_coord_end[1], middle_coord_end[2], middle_coord_start[1], middle_coord_start[2], 1 / abs(middle_curvature))

        # All circles have form (x-a)^2+(y-b)^2 = r^2
        if ((curr_x - x_shift)^2 + (curr_y - y_shift)^2 < (1 / middle_curvature)^2) # if we are too close to the inside border
            return steering_angle - s_step
        elseif ((curr_x - x_shift)^2 + (curr_y - y_shift)^2 > (1 / middle_curvature)^2) # if we are too close to the outside border
            return steering_angle + s_step
        end
    else # If lane is straight
        m, b = find_line_equation(middle_coord_start, middle_coord_end)
        if (curr_y < curr_x * m + b - epsilon) # If to the left of the center line
            return steering_angle - s_step
        elseif (curr_y > curr_x * m + b + epsilon) # If to the right of the center line
            return steering_anglea + s_step
        end
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
Returns a sequence of segments to go through to get from start_seg to end_seg. Runs djikstra's algorithm
Map is an object --> map::Dict{Int, RoadSegment}, start_seg and end_seg are integers representing the ID's
"""
function get_path(map, start_seg, end_seg)
    path = []

    n = length(map)
    dist = fill(typemax(Int), n)
    prev = fill(0, n)
    visited = falses(n)

    dist[start_seg] = 0

    while true
        # Find the unvisited vertex with the smallest distance
        u = -1
        mindist = typemax(Int)
        for v in keys(map)
            if !visited[v] && dist[v] < mindist
                u = v
                mindist = dist[v]
            end
        end

        if u == -1 || u == end_seg
            # All vertices have been visited or we have reached the end_seg
            break
        end

        visited[u] = true

        # Update the distances to the neighbors of the current vertex
        for v in map[u].children
            alt = dist[u] + 1 # All road weights are the same
            if alt < dist[v]
                dist[v] = alt
                prev[v] = u
            end
        end
    end

    # Build the sequence of road segments from the start to the end_seg
    path = [end_seg]
    while path[end] != start_seg
        push!(path, prev[path[end]])
    end
    return reverse(path)
end
