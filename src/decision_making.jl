
"""
@KEV DO COMMENTS BRUH
"""
function decision_making(localization_state_channel, 
        perception_state_channel, 
        map,
        target_road_segment_id, 
        socket)

    # do some setup
    curr_seg = -1
    new_seg = latest_localization_state[...]

    # Get the directions to get to the destination segment
    path = get_path(map, new_seg, target_road_segment_id)

    while true
        latest_localization_state = fetch(localization_state_channel)
        latest_perception_state = fetch(perception_state_channel)
        
        # Localization will output the current x and y coordinates, i have to parse the dictionary to find the corresponding segment id
        new_seg = latest_localization_state[...]

        if(new_seg != curr_seg) # if we have crossed into a new segment, make sure that we've crossed into the correct new segment
            new_seg = get_road_segment(map, path[2]) # the new segment will be the next in the djikstra's array
        end

        # Get the directions to get to the destination segment
        path = get_path(map, new_seg, target_road_segment_id)

        # Update steering_angle if we deviate from the center localization_state
        steering_angle = 0.0
        epsilon = 0.1
        callibrate_lane_middle(steering_angle, curr_x, curr_y, new_seg.lane_boundaries, epsilon)

        # Update target_vel if we are not moving at the speed limit (also update if there is a vehicle in front of us to slow down)
        # For ciculrar tracks, if radius is bigger than the center line's radius, we are on the left side, if its less, we are on right side
        target_vel = 0.0
        speed_limit = new_seg.speed_limit
        update_speed(curr_speed, speed_limit, target_vel)

        cmd = VehicleCommand(steering_angle, target_vel, true)
        serialize(socket, cmd)

        curr_seg = new_seg # update curr_seg
    end
end

# gets the road_segment object given the seg_id
function get_road_segment(map, seg_id)
    return map[seg_id]
end

# Update steering_angle if we deviate from the center localization_state. Lane_boundaries is a vector of the current segment's lane boundaries. 
function callibrate_lane_middle(steering_angle, curr_x, curr_y, lane_boundaries, epsilon)
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
            steering_angle = pi/9
        elseif ((curr_x-x_shift)^2 + (curr_y-y_shift)^2 > (1/middle_curvature)^2) # if we are too close to the outside border
            steering_angle = -pi/9
        else # if we are perfectly in the middle of the lane lines
            steering_angle = steering_angle #keep the steering angle the same
        end
    else # If lane is straight
        m,b = find_line_equation(middle_coord_start, middle_coord_end)
        if(curr_y > curr_x*m + b + epsilon) # If to the left of the center line
            steering_angle = pi/9
        elseif(curr_y < curr_x*m + b - epsilon) # If to the right of the center line
            steering_angle = -pi/9
        else # If on the center line (within a epsilon degree of uncertainty)
            steering_angle = steering_angle #keep the steering angle the same
    end
end

# Given two coordinates, returns the equation of the line in y = mx + b form
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

# finds the center coordinates of an implied circle given 2 xy coordiniates of points on the circumference and the circle's radius
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

# Update the velocity of the vehicle so that we are as close to the speed limit as possible
function update_speed(curr_speed, speed_limit, target_vel)
    if(curr_speed != speed_limit){
        target_vel = speed_limit
    }
end

# Returns a sequence of segments to go through to get from start_seg to end_seg. Runs djikstra's algorithm
# Map is an object --> map::Dict{Int, RoadSegment}, start_seg and end_seg are integers representing the ID's
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
