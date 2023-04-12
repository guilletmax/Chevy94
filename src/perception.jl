
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
function h(x, ego_state)
    corners_body = get_3d_bbox_corners(x)
    bboxes = []

    T_body_cam1 = get_cam_transform(1)
    T_body_cam2 = get_cam_transform(2)
    T_cam_camrot = get_rotated_camera_transform()

    T_body_camrot1 = multiply_transforms(T_body_cam1, T_cam_camrot)
    T_body_camrot2 = multiply_transforms(T_body_cam2, T_cam_camrot)

    T_world_body = get_body_transform(ego_state.q[1:4], ego_state.q[5:7])
    T_world_camrot1 = multiply_transforms(T_world_body, T_body_camrot1)
    T_world_camrot2 = multiply_transforms(T_world_body, T_body_camrot2)
    T_camrot1_world = invert_transform(T_world_camrot1)
    T_camrot2_world = invert_transform(T_world_camrot2)

    for (camera_id, transform) in zip((1,2), (T_camrot1_world, T_camrot2_world))
        
        other_vehicle_corners = [transform * [pt;1] for pt in corners_body[j]]
        visible = false

        left = image_width/2
        right = -image_width/2
        top = image_height/2
        bot = -image_height/2

		corners = [other_vehicle_corners[1], other_vehicle_corners[2], other_vehicle_corners[3], other_vehicle_corners[4]]

        for (num_corner, corner) in enumerate(other_vehicle_corners)
            if corner[3] < focal_len
                break
            end
            px = focal_len*corner[1]/corner[3]
            py = focal_len*corner[2]/corner[3]

            #left = min(left, px)
			if px < left
				left = px
				corners[1] = num_corner
			end

            #right = max(right, px)
			if px > right
				right = px
				corners[2] = num_corner
			end

            #top = min(top, py)
			if py < top
				top = py
				corners[3] = num_corner
			end

            #bot = max(bot, py)
			if py > bot
				bot = py
				corners[4] = num_corner
			end
        end
        if top ≈ bot || left ≈ right || top > bot || left > right
            # out of frame
            continue
        else 
            top = convert_to_pixel(image_height, pixel_len, top)
            bot = convert_to_pixel(image_height, pixel_len, bot)
            left = convert_to_pixel(image_width, pixel_len, left)
            right = convert_to_pixel(image_width, pixel_len, right)
            push!(bboxes, SVector(top, left, bot, right))
        end
    end

	(; bboxes, corners)
end

"""
Jacobian of h with respect to x, evaluated at x and Δt
"""
function jac_hx(x, Δt, corner)
	# Find J1 (Jacobian of get_3d_bbox_corners)
	if corner == 1
		J1 = [1 0 -l/2*sinθ 0 1/2*cosθ 0 0
			  0 1 w/2*cosθ 0 0 1/2*sinθ 0
			  0 0 0 0 0 0 1]
	elseif corner == 2
		J1 = [1 0 -l/2*sinθ 0 1/2*cosθ 0 0
			  0 1 w/2*cosθ 0 0 1/2*sinθ 0
			  0 0 0 0 0 0 0]
	elseif corner == 3
		J1 = [1 0 -l/2*sinθ 0 1/2*cosθ 0 0
			  0 1 -w/2*cosθ 0 0 -1/2*sinθ 0
			  0 0 0 0 0 0 1]
	elseif corner == 4
		J1 = [1 0 -l/2*sinθ 0 1/2*cosθ 0 0
			  0 1 -w/2*cosθ 0 0 -1/2*sinθ 0
			  0 0 0 0 0 0 0]
	elseif corner == 5
		J1 = [1 0 l/2*sinθ 0 -1/2*cosθ 0 0
			  0 1 w/2*cosθ 0 0 1/2*sinθ 0
			  0 0 0 0 0 0 1]
	elseif corner == 6
		J1 = [1 0 l/2*sinθ 0 -1/2*cosθ 0 0
			  0 1 w/2*cosθ 0 0 1/2*sinθ 0
			  0 0 0 0 0 0 0]
	elseif corner == 7
		J1 = [1 0 l/2*sinθ 0 -1/2*cosθ 0 0
			  0 1 -w/2*cosθ 0 0 -1/2*sinθ 0
			  0 0 0 0 0 0 1]
	elseif corner == 8
		J1 = [1 0 l/2*sinθ 0 -1/2*cosθ 0 0
			  0 1 -w/2*cosθ 0 0 -1/2*sinθ 0
			  0 0 0 0 0 0 0]
	end

	# Find J2 (Jacobian of transform)
	## FIXME: Is there a way to not repeat this code? Maybe put in a different function
	##		  and call in both h and jac_h (note it requires ego_state).
    T_body_cam1 = get_cam_transform(1)
    T_body_cam2 = get_cam_transform(2)
    T_cam_camrot = get_rotated_camera_transform()

    T_body_camrot1 = multiply_transforms(T_body_cam1, T_cam_camrot)
    T_body_camrot2 = multiply_transforms(T_body_cam2, T_cam_camrot)

    T_world_body = get_body_transform(ego_state.q[1:4], ego_state.q[5:7])
    T_world_camrot1 = multiply_transforms(T_world_body, T_body_camrot1)
    T_world_camrot2 = multiply_transforms(T_world_body, T_body_camrot2)
    T_camrot1_world = invert_transform(T_world_camrot1)
    T_camrot2_world = invert_transform(T_world_camrot2)

	## FIXME: how do we deal with having 2 cameras?
	J2 = T_camrot1_world[1:3, 1:3]
	#J2 = T_camrot2_world[1:3, 1:3]

	# Find J3 (Jacobian of projection)
	J3 = [focal_len/corner[3] 0 -focal_len*corner[1]*corner[3]^-1;
		  0 focal_len/corner[3] -focal_len*corner[2]*corner[3]^-1]

	# Find J4 (Jacobian of convert_to_pixel)
	J4 = [1/pixel_len 0
		  0 1/pixel_len]

	# Find J5
	J5 = [1 0]

	return J5 * J4 * J3 * J2 * J1
end

"""
Extended Kalman Filter for the perception module
"""
function perception_filter(; μ=[0 0 0 0 8 5 5], Σ=Diagonal([1^2, 1^2, 0.2^2, 0.5^2, 0.003^2, 0.003^2, 0.003^2]), num_steps=25, meas_freq=0.5, meas_jitter=0.025, meas_var=Diagonal([0.25,0.25]), proc_cov = Diagonal([0.2, 0.1]), dist_cov=Diagonal([0.3,0.3]), rng=MersenneTwister(5), output=true, latest_localization_state)
	timesteps = []
    μs = [μ,]
    Σs = Matrix{Float64}[Σ,]
    zs = Vector{Float64}[]

    μ_prev = μ
    Σ_prev = Σ

    results = PerceptionType[]

    for k = 1:num_steps
        Δ = meas_freq + meas_jitter * (2*rand(rng) - 1)

        A = jac_fx(μ_prev, Δ)
        μ̂ = f(μ_prev, Δ)
        C = jac_hx(μ̂)

        Σ̂ = A*Σ_prev*A'
        Σ = inv(inv(Σ̂) + C'*inv(meas_var)*C)

        d = h(μ̂, latest_localization_state).bboxes - C*μ̂
        μ = Σ * (inv(Σ̂)*μ̂ + C'*inv(meas_var)*(zₖ - d))

	    μ_prev = μ
	    Σ_prev = Σ
        
        #push!(μs, μ)
        #push!(Σs, Σ)

        push!(results, PerceptionType(μ, Σ))

        push!(timesteps, Δ)
        if output
            println("Ttimestep ", k, ":")
            #println("   Ground truth (x,y): ", xₖ[1:2])
            println("   Estimated (x,y): ", μ[1:2])
            #println("   Ground truth v: ", xₖ[3])
            println("   estimated v: ", μ[3])
            #println("   Ground truth θ: ", xₖ[4])
            println("   estimated θ: ", μ[4])
            println("   measurement received: ", zₖ)
            println("   Uncertainty measure (det(cov)): ", det(Σ))
        end
    end

    #(; μs, Σs)
    results
end


"""
Goal: Estimate x_k = [p1 p2 θ v l w h]
"""
function perception(cam_meas_channel, localization_state_channel, perception_state_channel)
    # set up stuff
    while true
        fresh_cam_meas = []
        while isready(cam_meas_channel)
            meas = take!(cam_meas_channel)
            push!(fresh_cam_meas, meas)
        end

        latest_localization_state = fetch(localization_state_channel)
        @info "Latest localization (gt) state: $latest_localization_state"

        perception_state = perception_filter(latest_localization_state)
        if isready(perception_state_channel)
            take!(perception_state_channel)
        end
        put!(perception_state_channel, perception_state)
    end
end
