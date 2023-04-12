
"""
@KEV DO COMMENTS BRUH
"""
function decision_making(localization_state_channel, 
        perception_state_channel, 
        map,
        target_road_segment_id, 
        socket)
  
    const CROSSED_CONFIRMED_COUNT = 10
    const v_step = 1.0
    const s_step = .314
    
    curr_seg =  get_current_segment(fetch(localization_state_channel))
    path = get_path(map, curr_seg, target_road_segment_id)
    next_path_index = 2
    steering_angle = 0.0
    epsilon = 0.1
    target_speed = 0.0
    crossed_segment_count = 0

    @info "Press 'q' at anytime stop Chevy94."
    while isopen(socket)
        key = get_c()
        if key == 'q'
            target_velocity = 0.0
            steering_angle = 0.0
            @info "Terminating Chevy94."
        
        x = fetch(localization_state_channel)
        latest_perception_state = fetch(perception_state_channel)

        if (get_current_segment(fetch(x)) != curr_seg)
            if (crossed_segment_count < CROSSED_CONFIRMED_COUNT)
                crossed_segment_count+=1
            else
                curr_seg = path[next_path_index]
                next_path_index+= 1
                crossed_segment_count = 0
        
        update_steering_angle(steering_angle, x[1], x[2], curr_seg.lane_boundaries, epsilon)

        update_speed(target_speed, curr_seg.speed_limit)

        cmd = VehicleCommand(steering_angle, target_velocity)
        serialize(socket, cmd)
    end

end

"""
Get segment ID from localization x and y state and map
"""
function get_segment_from_localization(x, y, map)
    for segment in map
        land_boundaries1 = segment.lane_boundaries[1]
        land_boundaries2 = segment.lane_boundaries[2]
        if(segment.curvature == 0)# if straight segment
            m1, b1 = find_line_equation(land_boundaries1.pt_a, land_boundaries1.pt_b) # lane boundary 1
            m2, b2 = find_line_equation(land_boundaries2.pt_a, land_boundaries2.pt_b) # lane boundary 2
            m3, b3 = find_line_equation(land_boundaries1.pt_a[1], land_boundaries2.pt_a[1], land_boundaries2.pt_a[2], land_boundaries1.pt_a[2]) # start boundary
            m4, b4 = find_line_equation(land_boundaries1.pt_b[1], land_boundaries2.pt_b[1], land_boundaries2.pt_b[2], land_boundaries1.pt_b[2]) # end boundary
            if (m1 * x + b1 < y && m2 * x + b2 > y && m3 * x + b3 < y && m4 * x + b4 > y) # if within both boundaries
                return segment
            end
        else # if curved segment
            r1 = 1/land_boundaries1.curvature
            r2 = 1/land_boundaries2.curvature
            c1, d1 = find_circle_center(land_boundaries1.pt_a[1], land_boundaries1.pt_a[2], land_boundaries1.pt_a[1], land_boundaries1.pt_b[2], r1)
            c2, d2 = find_circle_center(land_boundaries2.pt_a[1], land_boundaries2.pt_a[2], land_boundaries2.pt_a[1], land_boundaries2.pt_b[2], r2)
            m3, b3 = find_line_equation(land_boundaries1.pt_a[1], land_boundaries2.pt_a[1], land_boundaries2.pt_a[2], land_boundaries1.pt_a[2]) # start boundary
            m4, b4 = find_line_equation(land_boundaries1.pt_b[1], land_boundaries2.pt_b[1], land_boundaries2.pt_b[2], land_boundaries1.pt_b[2]) # end boundary
            if((x-c1)^2+(y-d1)^2 < r1^2 && (x-c2)^2+(y-d2)^2 > r2^2 && m3 * x + b3 < y && m4 * x + b4 > y)
                return segment
            end
        end
    end
    return -1 # should never return -1, we should always be within one of the segments
end

"""
Update steering_angle if we deviate from the center localization_state. Lane_boundaries is a vector of the current segment's lane boundaries. 
"""
function update_steering_angle(steering_angle, curr_x, curr_y, lane_boundaries, epsilon)
    edge1_coord_start = lane_boundaries[1].pt_a
    edge1_coord_end = lane_boundaries[1].pt_b
    edge2_coord_start = lane_boundaries[2].pt_a
    edge2_coord_end = lane_boundaries[2].pt_b
    lane_curve = lane_boundaries[1].curvature != 0 # 0 if straight, otherwise curved. True if curved, false if straight

    # middle_coord_start and middle_coord_end are the end points of the line/curve that is the middle of the lane
    middle_coord_start = [edge1_coord_start[1] - edge2_coord_start[1], edge1_coord_start[2] - edge2_coord_start[2]]
    middle_coord_end = [edge1_coord_end[1] - edge2_coord_end[1], edge1_coord_end[2] - edge2_coord_end[2]]
    middle_curvature = (abs(abs(lane_boundaries[1].curvature) - abs(lane_boundaries[2].curvature))/2) + min(abs(lane_boundaries[1].curvature), abs(lane_boundaries[2].curvature))

    if (lane_curve) # If lane is curved
        # x_shift and y_shift are the x and y coordinates of the center of the circle
        x_shift, y_shift = find_circle_center(middle_coord_end[1], middle_coord_end[2], middle_coord_start[1], middle_coord_start[2], 1/abs(middle_curvature))

        # All circles have form (x-a)^2+(y-b)^2 = r^2
        if ((curr_x-x_shift)^2 + (curr_y-y_shift)^2 < (1/middle_curvature)^2) # if we are too close to the inside border
            steering_angle-= s_step
        elseif ((curr_x-x_shift)^2 + (curr_y-y_shift)^2 > (1/middle_curvature)^2) # if we are too close to the outside border
            steering_angle+= s_step
        end
    else # If lane is straight
        m, b = find_line_equation(middle_coord_start, middle_coord_end)
        if(curr_y < curr_x*m + b - epsilon) # If to the left of the center line
            steering_angle-= s_step
        elseif(curr_y > curr_x*m + b + epsilon) # If to the right of the center line
            steering_anglea+= s_step
    end
end

"""
Given two coordinates, returns the equation of the line in y = mx + b form
"""
function find_line_equation(coord1, coord2)
    # Unpack coordinates
    x1, y1 = coord1
    x2, y2 = coord2
        
    # Calculate slope (m)
    m = (y2 - y1) / (x2 - x1)
        
    # Calculate y-intercept (b)
    b = y1 - m*x1

    return[m, b]
end

"""
Finds the center coordinates of an implied circle given 2 xy coordiniates of points on the circumference and the circle's radius
"""
function find_circle_center(x1, y1, x2, y2, radius)
    # Find the midpoint of the line segment connecting the two points
    mid_x = (x1 + x2) / 2
    mid_y = (y1 + y2) / 2

    # Find the distance between the two points
    distance = sqrt((x2 - x1)^2 + (y2 - y1)^2)

    # Calculate the distance from the midpoint to the center of the circle
    circle_distance = sqrt(radius^2 - (distance/2)^2)

    # Find the slope of the line connecting the two points
    if y2 - y1 == 0
        slope = 0
    else
        slope = (x2 - x1) / (y2 - y1)
    end

    # Find the x and y coordinates of the center of the circle
    if slope == 0
        center_x = mid_x
        center_y = mid_y + circle_distance
    elseif isinf(slope)
        center_x = mid_x + circle_distance
        center_y = mid_y
    else
        perp_slope = -1 / slope
        center_x = mid_x + circle_distance / sqrt(1 + perp_slope^2)
        center_y = mid_y + perp_slope * (center_x - mid_x)
    end
    
    return center_x, center_y
end

"""
Update the velocity of the vehicle so that we are as close to the speed limit as possible
TODO: Eventually optimize
"""
function update_speed(target_speed, speed_limit)
    # target_speed = speed_limit
    target_speed = 3.0
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
