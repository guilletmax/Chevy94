
function h_gps(x)
    #gps_loc_body is 3x1
    #Tbody is 3x4
    T = get_gps_transform()
    gps_loc_body = T * [zeros(3); 1.0]#3x1
    xyz_body = x[1:3]
    q_body = x[4:8]
    Tbody = get_body_transform(q_body, xyz_body)  #transform of the body
    xyz_gps = Tbody * [gps_loc_body; 1] #gps in map frame is 3x1
    #how does tbody change wrt to q_body and xyz_body
    return xyz_gps[1:2] #2x1
end

function h_gps_jacobian(x)
	ForwardDiff.jacobian(h_gps, x)
#    qw = x[4]
#    qx = x[5]
#    qy = x[6]
#    qz = x[7]
#    T = get_gps_transform()
#    g = T * [zeros(3); 1.0]#3x1
#    g1 = g[1]
#    g2 = g[2]
#    g3 = g[3]
#    [x[1] 0 0 2*qw*g1-2*qz*g2+2*qy*g3 2*qx*g1+2*qy*g2+2*qz*g3 -2*qy*g1+2*qx*g2+2*qw*g3 -2*qz*g1-2*qw*g2+2*qx*g3 0 0 0 0 0 0
#        0 x[2] 0 2*qz*g1+2*qw*g2-2*qx*g3 2*qy*g1-2*qx*g2-2*qw*g3 2*qx*g1+2*qy*g2+2*qz*g3 2*qw*g1-2*qz*g2+2*qy*g3 0 0 0 0 0 0]#Tbody[1:3, 1:3] #2 x 13
end

function h_imu(x)
    T_body_imu = get_imu_transform() #3x4
    T_imu_body = invert_transform(T_body_imu)
    R = T_imu_body[1:3, 1:3] #constant can be passed in or defined in h function
    p = T_imu_body[1:3, end] #constant can be passed in or defined in h function

    v_body = x[8:10] #x[8:10]
    ω_body = x[11:13] #x[11:13]

    ω_imu = R * ω_body #jacobian of output w wrt to input w is just R, output w wrt to vel, pos, and quat is just zero  

    v_imu = R * v_body + p × ω_imu #rotated version of velocity and adding constant p(cross product)w_imu. 
    #derivative should depend on v body and omega body

    return [v_imu; ω_imu] #6x1
end

#figure out how rows of T matrix cnage with respect to x. 
function h_imu_jacobian(x)
	ForwardDiff.jacobian(h_imu, x)
    #T_body_imu = get_imu_transform() #3x4
    #T_imu_body = invert_transform(T_body_imu) #3x4
    #R = T_imu_body[1:3, 1:3] #constant can be passed in or defined in h function
    #p = T_imu_body[1:3, end] #3x1

    ##wi1 wi2 wi3
    ##p1 p2 p3
    ##(r21w1+r22w2+r23w3)p3-(r31w1+r32w2+r33w3)p2  (r11w1+r12w2+r13w3)p2-(r21w1+r22w2+r23w3)p1  (r11w1+r12w2+r13w3)p2-(r21w1+r22w2+r23w3)p1
    ##[p3*r21-p]
    #v_body = x[8:10] #x[8:10]
    #ω_body = x[11:13] #3x1

    #ω_imu = R * ω_body #3x3 * 3x1 = 3x1
    ##[r11w1+r12w2+r13w3; r21w1+r22w2+r23w3 ; r31w1+r32w2+r33w3]
    #jac_w_imu_w_bod = R
    #v_imu = R * v_body + p × ω_imu #3x3 * 3x1 + 3x1 = 3x1 #changes wrt w and v_body
    #jav_v_imu_v = R
    #jac_v_imu_w_bod = [
    #    p[2]*R[3, 1]-p[3]*R[2, 1] p[2]*R[3, 2]-p[3]*R[2, 2] p[2]*R[3, 3]-p[3]*R[2, 3]
    #    p[1]*R[3, 1]-p[3]*R[1, 1] p[1]*R[3, 2]-p[3]*R[1, 2] p[1]*R[3, 3]-p[3]*R[1, 3]
    #    p[1]*R[2, 1]-p[2]*R[1, 1] p[1]*R[2, 2]-p[2]*R[1, 2] p[1]*R[2, 3]-p[2]*R[1, 3]]
    ## println(jac_v_imu_w_bod)
    ## jac_ω_imu = R
    ## jac_v_imu = [R p[1:3, 1:3] * ω_imu * R + R * p]
    ## [v_imu; ω_imu] 6x1

    #[zeros(6, 7) [jav_v_imu_v; zeros(3, 3)] [jac_v_imu_w_bod; jac_w_imu_w_bod]]

end

function localize_filter(μ_prev, Σ_prev, t_prev, Σ_gps, Σ_imu, Σ_proc, measurements)
    t = last(measurements).time
    for k in 1:length(measurements)
        Δ = -1
        if k == 1
            Δ = measurements[k].time - t_prev
        else
            Δ = measurements[k].time - measurements[k-1].time
        end
        μ̂ = rigid_body_dynamics(μ_prev[1:3], μ_prev[4:7], μ_prev[8:10], μ_prev[11:13], Δ) # 13 x 1

        h = -1
        C = -1
        Σ_z = -1

        zₖ = measurements[k]
        if zₖ isa GPSMeasurement
            # @info "it's a GPS!"
            zₖ = [zₖ.lat, zₖ.long]
            h = h_gps(μ̂) # 2 x 1
            C = h_gps_jacobian(μ̂) # 2 x 13
            Σ_z = Σ_gps # 2 x 2
        elseif zₖ isa IMUMeasurement
            # @info "it's an IMU!"
            zₖ = vcat(zₖ.linear_vel, zₖ.angular_vel)
            h = h_imu(μ̂) # 6 x 1
            C = h_imu_jacobian(μ̂) # 6 x 13
            Σ_z = Σ_imu # 6 x 6
        end
        #@info "h: $h"
        #@info "C: $C"
        #@info "Σ_z: $Σ_z"

        A = Jac_x_f(μ_prev, Δ) # 13 x 13
        #@info "A: $A"
        d = h - C * μ̂ # ( 2 x 1 || 6 x 1 ) - ( ( 2 x 13 || 6 x 13 ) * 13 x 1 ) = ( 2 x 1 || 6 x 1 )
        #@info "d: $d"
        Σ̂ = A * Σ_prev * A' + Σ_proc   # 13x13 * 13x13 * 13x13 = 13x13
        #@info "Σ̂: $Σ̂"
        Σ_k = (Σ̂^(-1) + C' * (Σ_z)^(-1) * C)^(-1) # ( 13 x 13 + ( 13 x 2 || 13 x 6 ) * ( 2 x 2 || 6 x 6 ) * ( 2 x 13 || 6 x 13 ) ) = 13 x 13
        #@info "Σ_k: $Σ_k"
        μ_k = Σ_k * (Σ̂^(-1) * μ̂ + C' * (Σ_z)^(-1) * (zₖ - d)) # ( 13 x 13 * ( 13 x 13 * 13 x 1 + 13 x 6 * 6 x 6 * 6 x 1 ) ) = 13 x 1
        #@info "μ_k: $μ_k"
        μ_prev = μ_k
        Σ_prev = Σ_k

		#@info "estimate at k=$k is $(μ_prev[1:2])"
		#@infiltrate
    end
    μ = μ_prev
    Σ = Σ_prev
    return μ, Σ, t
end


function localize(gps_channel, imu_channel, localization_state_channel, gt_channel, controls)
    fresh_gps_meas = []
    fresh_imu_meas = []

    μ_prev = -1
    Σ_prev = -1
    t_prev = -1

    setup = false
    while !setup
        sleep(0.001)
		
		gps_meas = -1
		if isready(gps_channel)
			gps_meas = take!(gps_channel)
		else
			continue
		end

		init_x = gps_meas.long
		init_y = gps_meas.lat
		init_z = 3.2428496460474134

		default_quaternion = [0.7071088264608639, -0.0002105419891602536, 0.0002601612999704231, 0.7071046567017563]
        default_velocity = [0.0030428557537161595, -0.0021233391786533917, -0.1077977346828422]
        default_angular_velocity = [0.0013151135040768936, 0.012796697753244386, -0.00010083551663550507]

		init_t = gps_meas.time

		μ_prev = [init_x, init_y, init_z, default_quaternion[1], default_quaternion[2], default_quaternion[3], default_quaternion[4], default_velocity[1], default_velocity[2], default_velocity[3], default_angular_velocity[1], default_angular_velocity[2], default_angular_velocity[3]]

		Σ_prev = Diagonal([50, 50, 0.0001, 50, 50, 50, 50, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001])

		t_prev = init_t

		controls.target_speed = 5
		sleep(1)

		while length(fresh_gps_meas) < 15 && length(fresh_imu_meas) < 15
			sleep(0.00001)
			while isready(gps_channel)
				sleep(0.00001)
				meas = take!(gps_channel)
				if meas.time >= t_prev
					push!(fresh_gps_meas, meas)
				end
			end

			while isready(imu_channel)
				sleep(0.00001)
				meas = take!(imu_channel)
				if meas.time >= t_prev
					push!(fresh_imu_meas, meas)
				end
			end
		end

		measurements = vcat(fresh_gps_meas, fresh_imu_meas)
		sort!(measurements, by = x -> x.time)

		Σ_gps = Diagonal([10, 10]) ### TODO: add a third item when we merge Forrest's changes with yaw
		Σ_imu = Diagonal([0.1, 0.1, 0.1, 1, 1, 1])
		Σ_proc = Diagonal([1, 1, 0.0001, 20, 20, 20, 20, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001])

		μ_prev, Σ_prev, t_prev = localize_filter(μ_prev, Σ_prev, t_prev, Σ_gps, Σ_imu, Σ_proc, measurements)

		fresh_gps_meas = []
		fresh_imu_meas = []

		setup = true
	end

	## END OF SETUP

	error = 0
	n = 0

    while true
        sleep(0.00001)
        while isready(gps_channel)
            sleep(0.000001)
            meas = take!(gps_channel)
            if meas.time >= t_prev
                push!(fresh_gps_meas, meas)
            end
        end
        while isready(imu_channel)
            sleep(0.000001)
            meas = take!(imu_channel)
            if meas.time >= t_prev
                push!(fresh_imu_meas, meas)
            end
        end

        if isempty(fresh_gps_meas) || isempty(fresh_imu_meas)
            continue
        end

        measurements = vcat(fresh_imu_meas, fresh_gps_meas)
        sort!(measurements, by=x -> x.time)
        #@info "length of measurements: $(length(measurements))"

        Σ_gps = Diagonal([10, 10])
        Σ_imu = Diagonal([1,1,1,1,1,1])
		Σ_proc = Diagonal([0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])

        #@info "μ_prev: $μ_prev"
        #@info "Σ_prev: $Σ_prev"
        #@info "t_prev: $t_prev"
        #@info "Σ_gps: $Σ_gps"
        #@info "Σ_imu: $Σ_imu"
        #@info "measurements: $measurements"
        μ, Σ, t = localize_filter(μ_prev, Σ_prev, t_prev, Σ_gps, Σ_imu, Σ_proc, measurements)
        #@info "localize filter results:"
        #@info "μ: $μ"
        #@info "Σ: $Σ"
        #@info "t: $t"

        localization_state = LocalizationType(μ[1:3])
        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, localization_state)
        #@info "localization state: $(localization_state.position[1:2])"

		if isready(gt_channel)
			gt_meas = take!(gt_channel)
		#	@info "ground truth: $(gt_meas.position[1:2])"
		end

		error = ((localization_state.position[1]-gt_meas.position[1])^2 + (localization_state.position[2]-gt_meas.position[2])^2)/2

        μ_prev = μ
        Σ_prev = Σ
        t_prev = t

        fresh_gps_meas = []
        fresh_imu_meas = []
    end
end
