
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

    curr_segment = 0
    path = []

    is_setup = false
    while !is_setup
        sleep(0.001)
        if (isready(localization_state_channel))
            x = fetch(localization_state_channel)
            @info x

            """TODO"""
            # STEP 1 -> fix get_segment_from_localization -> currently just goes forever I think --> no, the problem is that x is always an empty array so x[1] errors
            # curr_seg = get_segment_from_localization(x[1], x[2], map)

            """TODO"""
            # STEP 2 -> fix get_path
            #path = get_path(map, curr_seg, target_road_segment_id)
            #@info path

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
function get_segment_from_localization(x, y, map)
    for (id, segment) in map

        # 

        lane_boundaries_left = segment.lane_boundaries[1]
        lane_boundaries_right = segment.lane_boundaries[2]
        if (segment.lane_boundaries[1].curvature == 0) # if straight segment
            m_left, b_left = find_line_equation(lane_boundaries_left.pt_a, lane_boundaries_left.pt_b) # lane boundary 1
            m_right, b_right = find_line_equation(lane_boundaries_right.pt_a, lane_boundaries_right.pt_b) # lane boundary 2
            m_start, b_start = find_line_equation(lane_boundaries_left.pt_a, lane_boundaries_right.pt_a) # start boundary
            m_end, b_end = find_line_equation(lane_boundaries_left.pt_b, lane_boundaries_right.pt_b) # end boundary
            if (m_left * x + b_left < y && m_right * x + b_right > y && m_start * x + b_start < y && m_end * x + b_end > y) # if within both boundaries
                return id, segment
            end
        else # if curved segment
            r1 = 1 / land_boundaries1.curvature
            r2 = 1 / land_boundaries2.curvature
            c1, d1 = find_circle_center(land_boundaries1.pt_a[1], land_boundaries1.pt_a[2], land_boundaries1.pt_a[1], land_boundaries1.pt_b[2], r1)
            c2, d2 = find_circle_center(land_boundaries2.pt_a[1], land_boundaries2.pt_a[2], land_boundaries2.pt_a[1], land_boundaries2.pt_b[2], r2)
            m3, b3 = find_line_equation([land_boundaries1.pt_a[1], land_boundaries2.pt_a[1]], [land_boundaries2.pt_a[2], land_boundaries1.pt_a[2]]) # start boundary
            m4, b4 = find_line_equation([land_boundaries1.pt_b[1], land_boundaries2.pt_b[1]], [land_boundaries2.pt_b[2], land_boundaries1.pt_b[2]]) # end boundary
            if ((x - c1)^2 + (y - d1)^2 < r1^2 && (x - c2)^2 + (y - d2)^2 > r2^2 && m3 * x + b3 < y && m4 * x + b4 > y)
                return segment
            end
        end
    end
    return -1 # should never return -1, we should always be within one of the segments
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
Finds the center coordinates of an implied circle given 2 xy coordiniates of points on the circumference and the circle's radius. 
"""
function find_circle_center(x1, y1, x2, y2, r)
    q = sqrt((x2 - x1)^2 + (y2 - y1)^2)
    y3 = (y1 + y2) / 2
    x3 = (x1 + x2) / 2

    # There are 2 possible circles to go through 2 points. We will need to figure out how to choose which one we want
    xC1 = x3 + sqrt(r^2 - (q / 2)^2) * (y1 - y2) / q
    yC1 = y3 + sqrt(r^2 - (q / 2)^2) * (x2 - x1) / q
    xC2 = x3 - sqrt(r^2 - (q / 2)^2) * (y1 - y2) / q
    yC2 = y3 - sqrt(r^2 - (q / 2)^2) * (x2 - x1) / q

    return xC1, yC1, xC2, yC2
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
