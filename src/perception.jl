
### NOTES
###
### To test this code, just make sure @perception is called in project.jl and connect autonomous_client()
###

"""
Process model
"""
function f(x, Δt)
	[x[1] + Δt*cos(x[3])*x[4],
	x[2] + Δt*sin(x[3])*x[4],
	x[3],
	x[4],
	x[5],
	x[6],
	x[7]]
end

"""
Jacobian of f with respect to x, evaluated at x and Δt
"""
function jac_fx(x, Δt)
	[1 0 -Δt*sin(x[3])*x[4] Δt*cos(x[3]) 0 0 0 ;
	 0 1 Δt*cos(x[3])*x[4] Δt*sin(x[3]) 0 0 0 ;
	 0 0 1 0 0 0 0 ;
	 0 0 0 1 0 0 0 ;
	 0 0 0 0 1 0 0 ;
	 0 0 0 0 0 1 0 ;
	 0 0 0 0 0 0 1
	]
end

"""
Function to determine the 8 corners of a 3D bounding box in world frame, given state of vehicle.
"""
function get_3d_bbox_corners(state)
    corners = []
    for dx in [state[5]/2*cos(state[3]), -state[5]/2*cos(state[3])]
        for dy in [state[6]/2*sin(state[3]), -state[6]/2*sin(state[3])]
            for dz in [state[7]/2, -state[7]/2]
                push!(corners, [state[1]+dx, state[2]+dy, state[7]/2+dz])
            end
        end
    end
    corners
end

"""
Measurement model
"""
function h(x, ego_state, camera_id; focal_len = 0.01, pixel_len = 0.001, image_width = 640, image_height = 480)
    corners_body = get_3d_bbox_corners(x)
    bboxes = []

    T_cam_camrot = get_rotated_camera_transform()
    T_world_body = get_body_transform(ego_state.orientation, ego_state.position)

	transform = -1

	if camera_id == 1
    	T_body_cam1 = get_cam_transform(1)
    	T_body_camrot1 = multiply_transforms(T_body_cam1, T_cam_camrot)
    	T_world_camrot1 = multiply_transforms(T_world_body, T_body_camrot1)
    	T_camrot1_world = invert_transform(T_world_camrot1)
		transform = T_camrot1_world
	else # camera_id == 2
    	T_body_cam2 = get_cam_transform(2)
    	T_body_camrot2 = multiply_transforms(T_body_cam2, T_cam_camrot)
    	T_world_camrot2 = multiply_transforms(T_world_body, T_body_camrot2)
    	T_camrot2_world = invert_transform(T_world_camrot2)
		transform = T_camrot2_world
	end

    other_vehicle_corners = [transform * [pt;1] for pt in corners_body]
    visible = false

    left = image_width/2
    right = -image_width/2
    top = image_height/2
    bot = -image_height/2

	corners = [-1 -1 -1 -1]

    for (num_corner, corner) in enumerate(other_vehicle_corners)
        if corner[3] < focal_len
            break
        end
        px = focal_len*corner[1]/corner[3]
        py = focal_len*corner[2]/corner[3]

		if px < left
			left = px
			corners[1] = num_corner
		end

		if px > right
			right = px
			corners[2] = num_corner
		end

		if py < top
			top = py
			corners[3] = num_corner
		end

		if py > bot
			bot = py
			corners[4] = num_corner
		end
    end
    if !(top ≈ bot || left ≈ right || top > bot || left > right)
        top = convert_to_pixel(image_height, pixel_len, top)
        bot = convert_to_pixel(image_height, pixel_len, bot)
        left = convert_to_pixel(image_width, pixel_len, left)
        right = convert_to_pixel(image_width, pixel_len, right)
		push!(bboxes, top)
		push!(bboxes, left)
		push!(bboxes, bot)
		push!(bboxes, right)
    end

	(; bboxes, corners)
end

"""
Jacobian of h with respect to x, evaluated at x and Δt
"""
function jac_hx(x, ego_state, Δt, corner_id, camera_id, focal_len = 0.01, pixel_len = 0.001)
	l = x[5]
	w = x[6]
	θ = x[3]
	# Find J1 (Jacobian of get_3d_bbox_corners)
	if corner_id == 1
		J1 = [1 0 -l/2*sin(θ) 0 1/2*cos(θ) 0 0
			  0 1 w/2*cos(θ) 0 0 1/2*sin(θ) 0
			  0 0 0 0 0 0 1]
	elseif corner_id == 2
		J1 = [1 0 -l/2*sin(θ) 0 1/2*cos(θ) 0 0
			  0 1 w/2*cos(θ) 0 0 1/2*sin(θ) 0
			  0 0 0 0 0 0 0]
	elseif corner_id == 3
		J1 = [1 0 -l/2*sin(θ) 0 1/2*cos(θ) 0 0
			  0 1 -w/2*cos(θ) 0 0 -1/2*sin(θ) 0
			  0 0 0 0 0 0 1]
	elseif corner_id == 4
		J1 = [1 0 -l/2*sin(θ) 0 1/2*cos(θ) 0 0
			  0 1 -w/2*cos(θ) 0 0 -1/2*sin(θ) 0
			  0 0 0 0 0 0 0]
	elseif corner_id == 5
		J1 = [1 0 l/2*sin(θ) 0 -1/2*cos(θ) 0 0
			  0 1 w/2*cos(θ) 0 0 1/2*sin(θ) 0
			  0 0 0 0 0 0 1]
	elseif corner_id == 6
		J1 = [1 0 l/2*sin(θ) 0 -1/2*cos(θ) 0 0
			  0 1 w/2*cos(θ) 0 0 1/2*sin(θ) 0
			  0 0 0 0 0 0 0]
	elseif corner_id == 7
		J1 = [1 0 l/2*sin(θ) 0 -1/2*cos(θ) 0 0
			  0 1 -w/2*cos(θ) 0 0 -1/2*sin(θ) 0
			  0 0 0 0 0 0 1]
	elseif corner_id == 8
		J1 = [1 0 l/2*sin(θ) 0 -1/2*cos(θ) 0 0
			  0 1 -w/2*cos(θ) 0 0 -1/2*sin(θ) 0
			  0 0 0 0 0 0 0]
	end

	# Find J2 (Jacobian of transform)
    T_cam_camrot = get_rotated_camera_transform()
    T_world_body = get_body_transform(ego_state.orientation, ego_state.position)

	J2 = -1
	if camera_id == 1
    	T_body_cam1 = get_cam_transform(1)
    	T_body_camrot1 = multiply_transforms(T_body_cam1, T_cam_camrot)
    	T_world_camrot1 = multiply_transforms(T_world_body, T_body_camrot1)
    	T_camrot1_world = invert_transform(T_world_camrot1)
		J2 = T_camrot1_world[1:3, 1:3]
	else # camera_id == 2
    	T_body_cam2 = get_cam_transform(2)
    	T_body_camrot2 = multiply_transforms(T_body_cam2, T_cam_camrot)
    	T_world_camrot2 = multiply_transforms(T_world_body, T_body_camrot2)
    	T_camrot2_world = invert_transform(T_world_camrot2)
		J2 = T_camrot2_world[1:3, 1:3]
	end

	# Find J3 (Jacobian of projection)
    corners_body = get_3d_bbox_corners(x)
	corner = corners_body[corner_id]

	if corner[3] < focal_len
		J3 = [0 0 0
			  0 0 0]
	else
		J3 = [focal_len/corner[3] 0 -focal_len*corner[1]*corner[3]^-1;
			  0 focal_len/corner[3] -focal_len*corner[2]*corner[3]^-1]
	end

	# Find J4 (Jacobian of convert_to_pixel)
	### FIXME: Looking at my notes, should this just be [pixel_len 0
	###													 0         pixel_len]?
	### since convert_to_pixel does p' = s*p + s0
	J4 = [1/pixel_len 0
		  0 1/pixel_len]

	# Find J5
	### FIXME: I think this should have two different cases.
	### Something like [1 0] for top/bottom and [0 1] for left/right
	### I forget exactly where/when this was said, or what is meant by this.
	J5 = [1 0]

	return J5 * J4 * J3 * J2 * J1
end

"""
Extended Kalman Filter for the perception module
"""
function perception_filter(latest_localization_state, cam_measurements; μ=[0 0 0 0 8 5 5], Σ=Diagonal([1^2, 1^2, 0.2^2, 0.5^2, 0.003^2, 0.003^2, 0.003^2]), meas_var=Diagonal([0.25,0.25,0.25,0.25]), proc_cov = Diagonal([0.2, 0.1]), dist_cov=Diagonal([0.3,0.3]), rng=MersenneTwister(5), output=true, meas_freq=0.5, meas_jitter=0.025)
    μs = [μ,]
    Σs = Matrix{Float64}[Σ,]
    zs = Vector{Float64}[]

    μ_prev = μ
    Σ_prev = Σ

    results = PerceptionType[]

	# prev_time = cam_measurements[1].time

    for k in 2:length(cam_measurements)

		camera_id = cam_measurements[k].camera_id
		
		### FIXME: It seems like Δt should come from the difference in time between two camera measurements
		### However, when using this formula, it returns 0, possibly because the two measurements are too close
		### in time to each other. How to resolve this?
		# Δ = cam_measurements[k].time - prev_time
		#@info "curr time: $(cam_measurements[k].time)"
		Δ = meas_freq + meas_jitter * (2*rand(rng) - 1) # Δ as used in HW4
		@info "delta: $Δ"

		### Calculations of jac_fx, f, and h.

        A = jac_fx(μ_prev, Δ)
		@info "A: $A"

        μ̂ = f(μ_prev, Δ)
		@info "μ̂ $μ̂"

		h_result = h(μ̂, latest_localization_state, camera_id)
		@info "h result: $h_result"

		# Creates C as a 4x7 Jacobian, by calling jac_hx on each of the 4 3D corners which contribute to the bounding box
		C = []
		for corner_id in h_result.corners
			push!(C, jac_hx(μ̂, latest_localization_state, Δ, corner_id, cam_measurements[k].camera_id))
		end
		C = vcat(C...)
		@info "C: $C"

		### FIXME
		### This is similar to something that was in HW4 - but was this used to determine the "true" measurements of the simulation?
		### z_k is used in calculating μ, so not sure if this and μ should be changed.
		zₖ = h_result.bboxes + sqrt(meas_var) * randn(rng, 4)
		@info "z_k: $zₖ"
		

		### Equations from HW4 for updating Σ
        Σ̂ = A*Σ_prev*A'
		@info "Sigma hat: $Σ̂"
        Σ = inv(inv(Σ̂) + C'*inv(meas_var)*C)
		@info "sigma: $Σ"

		### These equations are similar to those in HW4. I think d makes sense, but am a little confused by μ.
		d = h_result.bboxes - C*μ̂
		@info "d: $d"
        μ = Σ * (inv(Σ̂)*μ̂ + C'*inv(meas_var)*(zₖ-d))
		@info "μ: $μ"

	    μ_prev = μ
	    Σ_prev = Σ

        push!(results, PerceptionType(μ, Σ))
		@info results

		## theoretically, this would be used to help set Δ
		#prev_time = cam_measurements[k].time
    end

    results
end


### FIXME: I'm a little confused by how exactly the filter should run.
### It seems like it's only running once when calling this function.
"""
Goal: Estimate x_k = [p1 p2 θ v l w h]
"""
function perception(cam_meas_channel, localization_state_channel, perception_state_channel)
    # set up stuff
    while true
        fresh_cam_meas = []
        while isready(cam_meas_channel)
			#@info "cam_meas_channel is ready"
            meas = take!(cam_meas_channel)
			if !isempty(meas.bounding_boxes)
            	push!(fresh_cam_meas, meas)
			end
        end

		# sort the camera measurements by timestep
		sorted_cam_meas = sort(fresh_cam_meas, by = x -> x.time)
		#@info "cam_meas are sorted now"

        latest_localization_state = fetch(localization_state_channel)
        #@info "Latest localization (gt) state: $latest_localization_state"
		#@info "localization fetched"

		if !isempty(sorted_cam_meas)
			#@info "cam meas not empty, call filter"
			#@info "calling filter with $latest_localization_state and $sorted_cam_meas"
			results = perception_filter(latest_localization_state, sorted_cam_meas)

			for result in results	
        		if isready(perception_state_channel)
            		take!(perception_state_channel)
        		end
        		put!(perception_state_channel, result)
			end
			@info "done with one pass of filter"
		end
	
    end
end
