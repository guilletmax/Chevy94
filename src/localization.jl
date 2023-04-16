struct LocalizationType
    state::Vector{Float64}
    state_covariance::Matrix{Float64}
end

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
    return xyz_gps[1:2]' #2x1
end

function h_gps_jacobian(x)
    qw = x[4]
    qx = x[5]
    qy = x[6]
    qz = x[7]

    [x[1] 0 0 2*qw-2*qz+2*qy 2*qx+2*qy+2*qz -2*qy+2*qx+2*qw -2*qz-2*qw+2*qx 0 0 0 0 0 0
        0 x[2] 0 2*qz+2*qw-2*qx 2*q2-2*qx-2*qw 2*qx+2*qy+2*qz 2*qw-2*qz+2*qy 0 0 0 0 0 0]#Tbody[1:3, 1:3] #2 x 13
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
    T_body_imu = get_imu_transform() #3x4
    T_imu_body = invert_transform(T_body_imu) #3x4
    R = T_imu_body[1:3, 1:3] #constant can be passed in or defined in h function
    p = T_imu_body[1:3, end] #3x1

    #wi1 wi2 wi3
    #p1 p2 p3
    #(r21w1+r22w2+r23w3)p3-(r31w1+r32w2+r33w3)p2  (r11w1+r12w2+r13w3)p2-(r21w1+r22w2+r23w3)p1  (r11w1+r12w2+r13w3)p2-(r21w1+r22w2+r23w3)p1
    #[p3*r21-p]
    v_body = x[8:10] #x[8:10]
    ω_body = x[11:13] #3x1

    ω_imu = R * ω_body #3x3 * 3x1 = 3x1
    #[r11w1+r12w2+r13w3; r21w1+r22w2+r23w3 ; r31w1+r32w2+r33w3]
    jac_w_imu_w_bod = R
    v_imu = R * v_body + p × ω_imu #3x3 * 3x1 + 3x1 = 3x1 #changes wrt w and v_body
    jav_v_imu_v = R
    jac_v_imu_w_bod = [
        p[2]*R[3][1]-p[3]*R[2][1] p[2]*R[3][2]-p[3]*R[2][2] p[2]*R[3][3]-p[3]*R[2][3]
        p[1]*R[3][1]-p[3]*R[1][1] p[1]*R[3][2]-p[3]*R[1][2] p[1]*R[3][3]-p[3]*R[1][3]
        p[1]*R[2][1]-p[2]*R[1][1] p[1]*R[2][2]-p[2]*R[1][2] p[1]*R[2][3]-p[2]*R[1][3]]
    # jac_ω_imu = R
    # jac_v_imu = [R p[1:3, 1:3] * ω_imu * R + R * p]
    # [v_imu; ω_imu] 6x1

    [zeros(6, 7) [jac_v_imu_v; zeros(3, 3)] [jac_v_imu_w_bod; jac_w_imu_w_bod]]

end

#is x vertical or horizontal
function f_jacobian(x, Δt)
    # jac_mag = sqrt(ωx^2 + ωy^2 + ωz^2) / norm(ω)
    # jac_sr = (-sin(mag * Δt / 2.0)) * (Δt / 2.0)
    # jac_vr = cos(mag * Δt / 2) * (Δt / 2)
    r = x[11:13] #1x3 ? renamed for convenience
    mag = norm(r) #square root of rTr (useful in calculating the derivative)

    sᵣ = cos(mag * Δt / 2.0) #deriv w.r.t mag
    jac_sr = (-sin(mag * Δt / 2.0) * Δt / 2 / mag) * r #this will be 1x3 dimension

    a = sin(mag * Δt / 2.0)
    jac_a = cos(mag * Δt / 2.0) * Δt / 2.0 / mag
    vᵣ = a * r / mag # 1x3 because r is 1x3
    jav_vr = [
        a/mag+r[1]^2*(mag*jac_a-a/mag) r[1]*r[2]*jac_a r[1]*r[3]*jac_a
        r[1]*r[2]*jac_a a/mag+r[2]^2*(mag*jac_a-a/mag) r[3]*r[2]*jac_a
        r[1]*r[3]*jac_a r[3]*r[2]*jac_a a/mag+r[3]^2*(mag*jac_a-a/mag)
    ]
    #3x3

    sₙ = quaternion[1] #1x1
    vₙ = quaternion[2:4] #1x3

    # [jac_sr; jav_vr]#4x3
    s = sₙ * sᵣ - vₙ' * vᵣ
    jac_s_int = [sₙ - vₙ' * vᵣ sₙ * sᵣ - vₙ'] #1x4 
    jac_s = jac_s_int * [jac_sr; jav_vr] #1x3

    v = sₙ * vᵣ + sᵣ * vₙ + vₙ × vᵣ #sn*[vr1 vr2 vr3] + sr*[vn1 vn2 vn3] + [vnr vnr vnr] this is a 1x3
    jac_v_int = [sₙ * vᵣ + sᵣ + vₙ × vᵣ; sₙ + sᵣ * vₙ + [vₙ[3] - vₙ[4] vₙ[2] - vₙ[4] vₙ[2] - vₙ[3]]] # 2x3
    jac_v = jac_v_int' * [jac_sr; jav_vr] #2x4
    jac_v_vn = sᵣ + [vᵣ[3] - vᵣ[2] vᵣ[3] - vᵣ[1] vᵣ[2] - vᵣ[1]]

    # new_position = position + Δt * velocity
    # new_quaternion = [s; v]
    jac_nq_wrt_r = [jac_s 0; jac_v]' #this is w.r.t r with dimensions 4x3 ?
    jac_nq_wrt_quat = [
        sᵣ a/mag*r[1] a/mag*r[2] a/mag*r[3]
        a/mag*r' [sᵣ a/mag*r[3] -a/mag r[2]; a/mag*r[3] sᵣ -a/mag*r[1]; a/mag*r[2] -a/mag*r[1] sᵣ]
    ]#should be a 4x4

    [
        x[1] 0 0 0 0 0 0 Δt 0 0 0 0 0
        0 x[2] 0 0 0 0 0 0 Δt 0 0 0 0
        0 0 x[3] 0 0 0 0 0 0 Δt 0 0 0
        [[0 0 0; 0 0 0; 0 0 0; 0 0 0] jac_nq_wrt_quat [0 0 0; 0 0 0; 0 0 0; 0 0 0]] jac_nq_wrt_r #fix me
        0 0 0 0 0 0 0 1 0 0 0 0 0
        0 0 0 0 0 0 0 0 1 0 0 0 0
        0 0 0 0 0 0 0 0 0 1 0 0 0
        0 0 0 0 0 0 0 0 0 0 1 0 0
        0 0 0 0 0 0 0 0 0 0 0 1 0
        0 0 0 0 0 0 0 0 0 0 0 0 1]

end

function localize_filter(; μ=zeros(13), Σ=Diagonal([5, 5, 3, 1.0, 1, 1, 1, 0.4, 0.4, 0.4, 0.2, 0.2, 0.2]), x0=zeros(13), num_steps=25, meas_freq=0.5, meas_jitter=0.025, Σ_z=Diagonal([0.25, 0.25, 0, 0, 0, 0, 0, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25]), rng=MersenneTwister(5), output=true)
    # u_constant = randn(rng) * [5.0, 0.2]
    # u_prev = zeros(2)
    gt_states = [x0,] # ground truth states that we will try to estimate
    timesteps = []
    μs = [μ,]
    Σs = Matrix{Float64}[Σ,]
    zs = Vector{Float64}[]
    x_prev = x0
    μ_prev = μ
    Σ_prev = Σ # change

    for k = 1:num_steps
        # uₖ = u_constant
        # mₖ = uₖ + sqrt(proc_cov) * randn(rng, 2) # Noisy IMU measurement.
        # ω_true = sqrt(dist_cov) * randn(rng, 2)
        # u_prev = uₖ
        # TODO : perform update on Σ, μ
        #c = f(x_prev,uₖ,0,Δ) - A*x_prev - B*uₖ - L*0 #f(μₖ₋₁, mₖ, 0, Δ) - Aμₖ₋₁ - Bmₖ - L*0
        # B = (μ_prev, mₖ, zeros(2), Δ)#∇ᵤf(μₖ₋₁, mₖ, 0, Δ),
        # println("HERE IS w:",ω_true, zeros(2) )
        # println("inside of for loop here is A", A)

        # L = jac_fu(μ_prev, mₖ, zeros(2), Δ)
        # d = h(μ̂) - C * μ̂
        Δ = meas_freq + meas_jitter * (2 * rand(rng) - 1) #todo - do i keep?
        xₖ = rigid_body_dynamics(x_prev[1:3], x_prev[4:7], x_prev[8:10], x_prev[11:13], Δ)
        x_prev = xₖ
        zₖ = [h_gps(xₖ); 0; 0; 0; 0; 0; h_imu(xₖ)] #13x1 TODO: do i keep this: + sqrt(meas_var) * randn(rng, 2)

        μ̂ = rigid_body_dynamics(μ_prev[1:3], μ_prev[4:7], μ_prev[8:10], μ_prev[11:13], Δ)# 13x1 f(μₖ₋₁, mₖ, 0, Δ)
        A = f_jacobian(μ_prev, Δ) #13x13 ∇ₓf(μₖ₋₁, mₖ, 0, Δ),
        C = [h_gps_jacobian(μ̂); zeros(5, 13); h_imu_jacobian(μ̂)] #13x13

        Σ̂ = A * Σ_prev * A' # 13x13 * 13x13 * 13x13 = 13x13
        Σ_k = (Σ̂^(-1) + C' * (Σ_z)^(-1) * C)^(-1)#Σ_z = 13x13 --> 13x13 * 13x13 * 13x13 = 13x13
        μ_k = Σ_k * (Σ̂^(-1) * μ̂ + C' * (Σ_z)^(-1) * zₖ) #13x13 * (13x1 + 13x13*13x13*13x1) = 13x13*13x1 = 13x1

        μ_prev = μ_k
        Σ_prev = Σ_k
        push!(μs, μ_k)
        push!(Σs, Σ_k)
        push!(zs, zₖ)
        push!(gt_states, xₖ)
        push!(timesteps, Δ)
        if output
            println("Ttimestep ", k, ":")
            println("   Ground truth (x,y,z): ", xₖ[1:3])
            println("   Estimated (x,y,z): ", μ_k[1:3])
            println("   Ground truth quat: ", xₖ[4:7])
            println("   Estimated quat: ", μ_k[4:7])
            println("   Ground truth v: ", xₖ[8:10])
            println("   estimated v: ", μ_k[8:10])
            println("   Ground truth w: ", xₖ[11:13])
            println("   estimated w: ", μ_k[11:13])
            println("   measurement received: ", zₖ)
            println("   Uncertainty measure (det(cov)): ", det(Σ_k))
        end
    end

    (; μs, Σs)
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
        localization_state = localize_filter(; μ=zeros(13), Σ=Diagonal([5, 5, 3, 1.0, 1, 1, 1, 0.4, 0.4, 0.4, 0.2, 0.2, 0.2]), x0=zeros(13), num_steps=25, meas_freq=0.5, meas_jitter=0.025, Σ_z=Diagonal([0.25, 0.25, 0, 0, 0, 0, 0, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25]), rng=MersenneTwister(5), output=true)

        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, localization_state)
    end
end