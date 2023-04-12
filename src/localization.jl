function h_gps(x, gps_loc_body)
    xyz_body = x[1:3]
    q_body = x[4:8]
    Tbody = get_body_transform(q_body, xyz_body)  #transform of the body
    xyz_gps = Tbody * [gps_loc_body; 1] #gps in map frame
    #how does tbody change wrt to q_body and xyz_body
    return xyz_gps[1:2]
end

function h_gps_jacobian(x, gps_loc_body)
    Tbody = get_body_transform(q_body, xyz_body)  #transform of the body
    return Tbody[1:3, 1:3]
end

function h_imu(x)
    T_body_imu = get_imu_transform()
    T_imu_body = invert_transform(T_body_imu)
    R = T_imu_body[1:3, 1:3] #constant can be passed in or defined in h function
    p = T_imu_body[1:3, end] #constant can be passed in or defined in h function

    v_body = x[8:10] #x[8:10]
    ω_body = x[11:13] #x[11:13]

    ω_imu = R * ω_body #jacobian of output w wrt to input w is just R, output w wrt to vel, pos, and quat is just zero  

    v_imu = R * v_body + p × ω_imu #rotated version of velocity and adding constant p(cross product)w_imu. 
    #derivative should depend on v body and omega body

    return [v_imu; ω_imu]
end

#figure out how rows of T matrix cnage with respect to x. 
function h_imu_jacobian(x)
    T_body_imu = get_imu_transform()
    T_imu_body = invert_transform(T_body_imu)
    R = T_imu_body[1:3, 1:3] #constant can be passed in or defined in h function
    p = T_imu_body[1:3, end]

    jac_ω_imu = R
    jac_v_imu = 

end

function f_jacobian(x)
end

function localize_filter()
end

function localize(gps_channel, imu_channel, localization_state_channel)
    # Set up algorithm / initialize variables
    while true
        fresh_gps_meas = []
        while isready(gps_channel)
            meas = take!(gps_channel)
            push!(fresh_gps_meas, meas)
        end
        fresh_imu_meas = []
        while isready(imu_channel)
            meas = take!(imu_channel)
            push!(fresh_imu_meas, meas)
        end
        
        # process measurements

        localization_state = MyLocalizationType(0,0.0)
        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, localization_state)
    end 
end