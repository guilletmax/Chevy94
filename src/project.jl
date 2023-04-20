
struct LocalizationType
    position::SVector{3,Float64} # position of center of vehicle
end

struct PerceptionType
    state::Vector{Float64}
    state_covariance::Matrix{Float64}
end

mutable struct Controls
    target_speed::Float64
    steering_angle::Float64
end

function isfull(ch::Channel)
    length(ch.data) ≥ ch.sz_max
end

function autonomous_client(host::IPAddr=IPv4(0), port=4444)
    socket = Sockets.connect(host, port)

    map = training_map()

    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    localization_state_channel = Channel{LocalizationType}(1)
    perception_state_channel = Channel{PerceptionType}(1)

    target_map_segment = 0
    ego_vehicle_id = 0

    msg = deserialize(socket)
    @info msg

    @async while isopen(socket)
        sleep(0.001)
        state_msg = deserialize(socket)
        measurements = state_msg.measurements
        target_map_segment = state_msg.target_segment
        ego_vehicle_id = state_msg.vehicle_id

        for meas in measurements
            if meas isa GPSMeasurement
                !isfull(gps_channel) && put!(gps_channel, meas)
            elseif meas isa IMUMeasurement
                !isfull(imu_channel) && put!(imu_channel, meas)
            elseif meas isa CameraMeasurement
                !isfull(cam_channel) && put!(cam_channel, meas)
            elseif meas isa GroundTruthMeasurement
                !isfull(gt_channel) && put!(gt_channel, meas)
            end
            # @info "meas: $meas"
        end
    end

    controlled = true
    controls = Controls(0.0, 0.0)

	try
    	@async localize(gps_channel, imu_channel, localization_state_channel, gt_channel, controls)
	catch e
		@info "exception: $e"
	end
    #@async perception(cam_channel, localization_state_channel, perception_state_channel)
    @async decision_making(localization_state_channel, perception_state_channel, map, socket, controls)

    while controlled && isopen(socket)
        sleep(0.01)
        cmd = VehicleCommand(controls.steering_angle, controls.target_speed, controlled)
        serialize(socket, cmd)
    end
end
