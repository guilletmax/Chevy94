
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
            zₖ = [zₖ.lat, zₖ.long, zₖ.heading]
            h = h_gps(μ̂) # 3 x 1
            C = Jac_h_gps(μ̂) # 3 x 13
            Σ_z = Σ_gps # 3 x 3
        elseif zₖ isa IMUMeasurement
            # @info "it's an IMU!"
            zₖ = vcat(zₖ.linear_vel, zₖ.angular_vel)
            h = h_imu(μ̂) # 6 x 1
            C = h_imu_jacobian(μ̂) # 6 x 13
            Σ_z = Σ_imu # 6 x 6
        end
        # @info "h: $h"
        # @info "C: $C"
        # @info "Σ_z: $Σ_z"

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
    # @info "localization started"
    fresh_gps_meas = []
    fresh_imu_meas = []

    μ_prev = -1
    Σ_prev = -1
    t_prev = -1

    @info "waiting for setup"
    @info "setting target speed to 1"

    t_prev = -1
    while length(fresh_gps_meas) < 10
        sleep(0.00001)
        if isready(gps_channel)
            sleep(0.000001)
            meas = take!(gps_channel)
            if meas.time >= t_prev
                push!(fresh_gps_meas, meas)
                t_prev = meas.time
            end
        end
    end

    mean_lat = mean(meas.lat for meas in fresh_gps_meas)
    mean_long = mean(meas.long for meas in fresh_gps_meas)

    @info "mean lat/long: $mean_lat, $mean_long"

    init_x = mean_lat
    init_y = mean_long
    init_z = 3.2428496460474134

    default_quaternion = [0.7071088264608639, -0.0002105419891602536, 0.0002601612999704231, 0.7071046567017563]
    default_velocity = [0, 1, 0]
    default_angular_velocity = [0.0013151135040768936, 0.012796697753244386, -0.00010083551663550507]

    init_t = t_prev

    μ_prev = [init_x, init_y, init_z, default_quaternion[1], default_quaternion[2], default_quaternion[3], default_quaternion[4], default_velocity[1], default_velocity[2], default_velocity[3], default_angular_velocity[1], default_angular_velocity[2], default_angular_velocity[3]]

    Σ_prev = Diagonal([3, 3, 0.0001, 5, 5, 5, 5, 1, 0.001, 0.001, 0.001, 0.001, 0.001])

    t_prev = init_t


    ## END OF SETUP
    localization_state = LocalizationType(μ_prev[1:3])
    if isready(localization_state_channel)
        take!(localization_state_channel)
    end
    put!(localization_state_channel, localization_state)

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

        Σ_gps = Diagonal([1.0, 1.0, 0.1])^2
        Σ_imu = Diagonal([0.001, 0.001, 0.001, 0.001, 0.001, 0.001])^2
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

        error = ((localization_state.position[1] - gt_meas.position[1])^2 + (localization_state.position[2] - gt_meas.position[2])^2) / 2

        μ_prev = μ
        Σ_prev = Σ
        t_prev = t

        fresh_gps_meas = []
        fresh_imu_meas = []
    end
end
