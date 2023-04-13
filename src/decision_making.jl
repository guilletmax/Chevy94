
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
            # STEP 2 -> fix get_path - STATUS: L
            @info "ENTERING SHORTEST PATH"
            shortest_path = shortest_path_bfs(map, curr_segments[1].id, target_road_segment_id)
            @info shortest_path
            min_dist = length(shortest_path)
            @info min_dist
            for i in eachindex(curr_segments[2:end])
                @info i
                new_path = shortest_path_bfs(map, curr_segments[1].id, target_road_segment_id)
                @info new_path
                new_dist = lenth(new_path)
                @info new_dist
                if new_dist < min_dist
                    shortest_path = new_path
                    min_dist = new_dist
                end
            end

            @info shortest_path
            path = shortest_path
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
        controls.steering_angle = get_steering_angle(controls.steering_angle, x[1], x[2], curr_seg.lane_boundaries, epsilon)
        #controls.steering_angle = 0.314 #comment me out when ready

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
                    push!([segment], segments)
                    @info "adding to segments:"
                    @info segments
                else
                    @info "returning"
                    return [segment]
                end
            end
        else
            @info "is curved"
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
			
			circle_center = [0, 0] 
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
                    push!([segment], segments)
                    @info "adding to segments:"
                    @info segments
                else
                    @info "returning"
                    return [segment]
                end
            end
        end
    end
    @info "for loop complete:"
    @info "segments"
    return segments
end

function get_circle_center(lane_boundaries_left, lane_boundaries_right)
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
	
	circle_center = [0, 0] 
    if norm([x; y] - small_center_one) < norm([x; y] - small_center_two)
        circle_center = small_center_two
    else
        circle_center = small_center_one
    end

	circle_center
end

"""
Update steering_angle if we deviate from the center localization_state. Lane_boundaries is a vector of the current segment's lane boundaries. 
"""
function get_steering_angle(steering_angle, curr_x, curr_y, lane_boundaries, epsilon)
    lane_curve = lane_boundaries[1].curvature != 0 # True if curved, false if straight, 0 if straight, negative if curve right, positive if curve left. 
	@info "in function"

	if !lane_curve
	@info "not curved"
   		normal_vector = lane_boundaries_right.pt_a - lane_boundaries_left.pt_a
		center_point = (lane_boundaries_left.pt_a + lane_boundaries_right.pt_a) / 2
		@info "normal vec: $normal_vector"
		@info "center point: $center_point"
    	a = dot(left_normal_vector, [x; y])
		b = dot(normal_vector, center_point)
		@info "dot products: $a and $b"
		if a > b
			@info "We should turn left"
			# turn left
			return 0.314
		elseif a < b
			@info "We should turn right"
			# turn right
			return -0.314
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
   		 
   		circle_center = [0, 0] 
   		if norm([x; y] - small_center_one) < norm([x; y] - small_center_two)
   		    circle_center = small_center_two
   		else
   		    circle_center = small_center_one
   		end
		@info circle_center

		lane_center_radius = (left_r + right_r) / 2

        dist_to_center = norm([x; y] - circle_center)
        closer_to_inner_curve = dist_to_center <= lane_center_radius 

		if closer_to_inner_curve
			if lane_boundaries[1].curvature < 0
				return -0.314
			elseif lane_boundaries[1].curvature > 0
				return 0.314
			end
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
TBD
"""
# function shortest_path(start_segment, end_segment)

# end

function shortest_path_bfs(graph, start::Any, goal::Any)
    # initialize the visited set and the queue
    visited = Set()
    queue = Deque{Any}(10)

    # add the initial path to the queue
    push!(queue, [start])

    # loop through the queue until it's empty
    while !isempty(queue)
        # get the first path from the queue
        path = popfirst!(queue)

        # get the last node from the path
        node = last(path)

        # check if we've reached the goal
        if node == goal
            return path
        end

        # check if we've already visited this node
        if !(node in visited)
            # mark the node as visited
            push!(visited, node)

            # get the children of the current node
            children = graph[node].children

            # loop through the children of the current node
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
