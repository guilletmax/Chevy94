
# @testset "derivative tests" begin
#     rng = MersenneTwister(1)
#     for i = 1:10
#         x = randn(rng, 4)
#         u = randn(rng, 2)
#         ω = randn(rng, 2)
#         Δ = 0.1

#         xx = rigid_body_dynamics(x, u, ω, Δ)
#         Jx = f_jacobian(x, u, ω, Δ)
#         Jxn = similar(Jx)
#         Jun = similar(Ju)
#         Jωn = similar(Ju)
#         for j = 1:4
#             ej = zeros(4)
#             ej[j] = 1e-6
#             Jxn[:, j] = (HW4.f(x + ej, u, ω, Δ) - xx) / 1e-6
#         end
#         for j = 1:2
#             ej = zeros(2)
#             ej[j] = 1e-6
#             Jun[:, j] = (HW4.f(x, u + ej, ω, Δ) - xx) / 1e-6
#         end
#         for j = 1:2
#             ej = zeros(2)
#             ej[j] = 1e-6
#             Jωn[:, j] = (HW4.f(x, u, ω + ej, Δ) - xx) / 1e-6
#         end
#         zz_gps = h_gps(x)
#         zz_imu = h_imu(x)
#         Jxh_gps = h_gps(x)
#         Jxh_imu = h_imu(x)
#         Jxhn = similar(Jxh)
#         for j = 1:4
#             ej = zeros(4)
#             ej[j] = 1e-6
#             Jxhn[:, j] = (HW4.h(x + ej) - zz) / 1e-6
#         end
#         @test isapprox(Jx, Jxn; atol=1e-6)
#         @test isapprox(Ju, Jun; atol=1e-6)
#         @test isapprox(Jω, Jωn; atol=1e-6)
#         @test isapprox(Jxh, Jxhn; atol=1e-5)
#     end
# end

function test_jac()
    x = randn(n)
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
        0 x[2] 0 2*qz+2*qw-2*qx 2*qy-2*qx-2*qw 2*qx+2*qy+2*qz 2*qw-2*qz+2*qy 0 0 0 0 0 0]#Tbody[1:3, 1:3] #2 x 13
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
        p[2]*R[3, 1]-p[3]*R[2, 1] p[2]*R[3, 2]-p[3]*R[2, 2] p[2]*R[3, 3]-p[3]*R[2, 3]
        p[1]*R[3, 1]-p[3]*R[1, 1] p[1]*R[3, 2]-p[3]*R[1, 2] p[1]*R[3, 3]-p[3]*R[1, 3]
        p[1]*R[2, 1]-p[2]*R[1, 1] p[1]*R[2, 2]-p[2]*R[1, 2] p[1]*R[2, 3]-p[2]*R[1, 3]]
    println(jac_v_imu_w_bod)
    # jac_ω_imu = R
    # jac_v_imu = [R p[1:3, 1:3] * ω_imu * R + R * p]
    # [v_imu; ω_imu] 6x1

    [zeros(6, 7) [jav_v_imu_v; zeros(3, 3)] [jac_v_imu_w_bod; jac_w_imu_w_bod]]

end

#x = [p1 p2 p3 q1 q2 q3 q4 v1 v2 v3 w1 w2 w3]
#is x vertical or horizontal
function f_jacobian(x, Δt)
    # jac_mag = sqrt(ωx^2 + ωy^2 + ωz^2) / norm(ω)
    # jac_sr = (-sin(mag * Δt / 2.0)) * (Δt / 2.0)
    # jac_vr = cos(mag * Δt / 2) * (Δt / 2)
    r = x[11:13] #3x1 ? renamed for convenience
    mag = norm(r) #square root of rTr (useful in calculating the derivative)

    sᵣ = cos(mag * Δt / 2.0) #deriv w.r.t mag
    jac_sr = (-sin(mag * Δt / 2.0) * Δt / 2 / mag) * r #3x1

    a = sin(mag * Δt / 2.0)
    jac_a = cos(mag * Δt / 2.0) * Δt / 2.0 / mag
    vᵣ = a * r / mag # 3x1
    jav_vr = [
        a/mag+r[1]^2*(mag*jac_a-a/mag) r[1]*r[2]*jac_a r[1]*r[3]*jac_a
        r[1]*r[2]*jac_a a/mag+r[2]^2*(mag*jac_a-a/mag) r[3]*r[2]*jac_a
        r[1]*r[3]*jac_a r[3]*r[2]*jac_a a/mag+r[3]^2*(mag*jac_a-a/mag)
    ]
    #3x3

    sₙ = x[4] #1x1
    vₙ = x[5:7] #3x1

    # [jac_sr; jav_vr]#4x3
    s = sₙ * sᵣ - vₙ' * vᵣ #1x1

    # jac_s = [sₙ 0 0] #
    v = sₙ * vᵣ + sᵣ * vₙ + vₙ × vᵣ #1x1*3x1 + 1x1*3x1 + 3x1 = 3x1
    #sn*[vr1 vr2 vr3] + sr*[vn1 vn2 vn3] + [vnr vnr vnr] this is a 3x1
    #v = [sn*vr1+sr*vn1+vnr1 sn*vr2+sr*vn2+vnr2 sn*vr3+sr*vn3+vnr3]
    #jac_v= [  vₙ ]

    # if mag is small: v = vn 3x1 vectr
    #if mag is small: s = sn 
    #if mag is small: new_quat = [sn; vn]
    jac_s_wrt_r = [0 0 0]
    # jac_v = [vₙ[1] 0 0; 0 vₙ[2] 0; 0 0 vₙ[3]] #3x3

    jac_v_wrt_r = [0 0 0; 0 0 0; 0 0 0] #v = vn 3x1 vector then v does not change  wrt to r
    jac_nq_wrt_quat = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] #4x4

    if mag >= 1e-5
        jac_s_int = [sₙ - vₙ' * vᵣ sₙ * sᵣ .- vₙ'] #1x4
        jac_s_wrt_r = jac_s_int * [jac_sr'; jav_vr] #1x4 * 4x3 = 1x3 TODO: Do i need to transpose jac_sr

        # jac_v_int = [[vᵣ[1] 0 0 ; 0 vᵣ[2] 0; 0 0 vᵣ[3]] ; sₙ + [vₙ[3] - vₙ[4] vₙ[2] - vₙ[4] vₙ[2] - vₙ[3]]] # 2x3
        # jac_v_wrt_r = jac_v_int' * [jac_sr; jav_vr] #2x3 * 4x3 = should be a 3x3 wrt to r
        # jac_v_vn = sᵣ + [vᵣ[3] - vᵣ[2] vᵣ[3] - vᵣ[1] vᵣ[2] - vᵣ[1]] #1x3
        b = sᵣ * Δt / 2 - a / mag
        q = x[4:7]
        jac_v_int = [
            r[1]*b*(q[3]*r[3]-q[4]*r[2]) r[2]*b*(q[3]*r[3]-q[4]*r[2])-q[4]*a/mag r[3]*b*(q[3]*r[3]-q[4]*r[2])+q[3]*a/mag
            r[1]*b*(q[2]*r[3]-q[4]*r[1])-q[4]*a/mag r[2]*b*(q[2]*r[3]-q[4]*r[1]) r[3]*b*(q[2]*r[3]-q[4]*r[1])+q[2]*a/mag
            r[1]*b*(q[2]*r[2]-q[3]*r[1])-q[3]*a/mag r[2]*b*(q[2]*r[2]-q[3]*r[1])+q[2]*a/mag r[3]*b*(q[2]*r[2]-q[3]*r[1])
        ]#3x3

        jac_v_wrt_r = [vᵣ[1] 0 0; 0 vᵣ[2] 0; 0 0 vᵣ[3]] * jav_vr .+ vₙ' * jac_sr + jac_v_int #3x3*3x3 +3x1*1*3 + 3x3 = 3x3 #TODO IS THIS RIGHT???
        jac_nq_wrt_quat = [
            sᵣ a/mag*r[1] a/mag*r[2] a/mag*r[3]
            (a / mag * r')' [sᵣ a/mag*r[3] -a/mag*r[2]; a/mag*r[3] sᵣ -a/mag*r[1]; a/mag*r[2] -a/mag*r[1] sᵣ]
        ]#should be a 4x4
    end

    jac_nq_wrt_r = [jac_s_wrt_r; jac_v_wrt_r] #this is w.r.t r with dimensions 4x3 
    println("jac_nq_wrt_r: ", size(jac_nq_wrt_r))
    jac_new_position = [[x[1] 0 0; 0 x[2] 0; 0 0 x[3]] zeros(3, 4) [Δt 0 0; 0 Δt 0; 0 0 Δt] zeros(3, 3)]
    jac_new_quat = [zeros(4, 3) jac_nq_wrt_quat zeros(4, 3) jac_nq_wrt_r] #4x13
    jac_new_velocity = [zeros(3, 7) [1 0 0; 0 1 0; 0 0 1] zeros(3, 3)]
    jac_new_angular = [zeros(3, 10) [1 0 0; 0 1 0; 0 0 1]]

    [
        jac_new_position
        jac_new_quat
        jac_new_velocity
        jac_new_angular
    ]

end

#what are good values for Epsilon
#does a control variable exist
#Line 187
#Line 184
#How should I pass in the imu and gps measurements??? A little confused about the initialization steps... not sure what to do here
#rule of thumb: describe what kind of error you would expect in model you are using - quaternion: normalize to 1 (4 element vector with norm 1)
##can find for Ez elsewhere
function localize_filter(; μ=zeros(13), Σ=Diagonal([5, 5, 3, 1.0, 1, 1, 1, 0.4, 0.4, 0.4, 0.2, 0.2, 0.2]), x0=zeros(13), Σ_z=Diagonal([0.25, 0.25, 0, 0, 0, 0, 0, 0.25]), output=true, fresh_meas)
    # u_constant = randn(rng) * [5.0, 0.2]
    # u_prev = zeros(2)
    # gt_states = [x0,] # ground truth states that we will try to estimate
    timesteps = []
    μs = [μ,]
    Σs = Matrix{Float64}[Σ,]
    zs = Vector{Float64}[]
    x_prev = x0
    μ_prev = μ
    Σ_prev = Σ # change

    #based off of fresh mesh: sort based on time and then loop

    for k = 2:length(fresh_meas)
        Δ = fresh_meas[k] - fresh_meas[k-1] #todo - do i keep?
        xₖ = rigid_body_dynamics(x_prev[1:3], x_prev[4:7], x_prev[8:10], x_prev[11:13], Δ)
        x_prev = xₖ
        zₖ = [fresh_gps_meas[k]; 0; 0; 0; 0; 0; fresh_gps_meas[k]] #Z_k is the fresh measu 13x1 TODO: do i keep this: + sqrt(meas_var) * randn(rng, 2)

        μ̂ = rigid_body_dynamics(μ_prev[1:3], μ_prev[4:7], μ_prev[8:10], μ_prev[11:13], Δ)# 13x1 f(μₖ₋₁, mₖ, 0, Δ)
        A = f_jacobian(μ_prev, Δ) #13x13 ∇ₓf(μₖ₋₁, mₖ, 0, Δ),
        C = [h_gps_jacobian(μ̂); zeros(5, 13); h_imu_jacobian(μ̂)] #13x13
        h = [h_gps(μ̂); zeros(13); h_imu(μ̂)]
        d = h - C * μ̂
        Σ̂ = A * Σ_prev * A'   # define another type of sigma covariance for sigma process 13x13 * 13x13 * 13x13 = 13x13
        Σ_k = (Σ̂^(-1) + C' * (Σ_z)^(-1) * C)^(-1)#Σ_z = 13x13 --> 13x13 * 13x13 * 13x13 = 13x13
        μ_k = Σ_k * (Σ̂^(-1) * μ̂ + C' * (Σ_z)^(-1) * (zₖ - d)) #13x13 * (13x1 + 13x13*13x13*13x1) = 13x13*13x1 = 13x1
        #h_mu^
        μ_prev = μ_k
        Σ_prev = Σ_k
        push!(μs, μ_k)
        push!(Σs, Σ_k)
        push!(zs, zₖ)
        push!(gt_states, xₖ)
        push!(timesteps, Δ)
    end

    (; μs, Σs)
end

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
# if output
#     println("Ttimestep ", k, ":")
#     println("   Ground truth (x,y,z): ", xₖ[1:3])
#     println("   Estimated (x,y,z): ", μ_k[1:3])
#     println("   Ground truth quat: ", xₖ[4:7])
#     println("   Estimated quat: ", μ_k[4:7])
#     println("   Ground truth v: ", xₖ[8:10])
#     println("   estimated v: ", μ_k[8:10])
#     println("   Ground truth w: ", xₖ[11:13])
#     println("   estimated w: ", μ_k[11:13])
#     println("   measurement received: ", zₖ)
#     println("   Uncertainty measure (det(cov)): ", det(Σ_k))
# end

#how is localize called - how often will it return?
#what do I do about time steps?
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

        #wait and get first gps measurement and use that to inform mu zero

        measures = [fresh_imu_meas fresh_gps_meas]
        sort_meas_array(measures) #sort by time
        Tbody = get_body_transform(q_body, xyz_body)
        xyz_gps = Tbody * [gps_loc_body; 1]
        init_mu = [xyz[1] xyz[2] xyz[3]]
        #orientation: what part of the map are you on, what lane segment, which direction is it facing and that will tell us the orientation 
        # process measurements
        (; μs, Σs) = localize_filter(; μ=init_mu, num_steps=5, output=false, fresh_meas=measures)
        localization_state = LocalizationType(μs, Σs)
        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, localization_state)
    end
end

# struct GPSMeasurement <: Measurement
#     time::Float64
#     lat::Float64
#     long::Float64
# end
# struct IMUMeasurement <: Measurement
#     time::Float64
#     linear_vel::SVector{3,Float64}
#     angular_vel::SVector{3,Float64}
# end

function sort_meas_array(meas_array)
    sort(meas_array, by=x -> x.time)
end
