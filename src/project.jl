
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

    controls = Controls(0.0, 0.0)

    controlled = true
    arrived_channel = Channel{Int}(1)
    put!(arrived_channel, 1)
    terminate = false
    while !terminate
        if isready(arrived_channel)
            take!(arrived_channel)
            println("Please enter your destination segment ID, or q to terminate program:")
            target_road_segment_id = readline()
            if target_road_segment_id == "q"
                terminate = true
                put!(arrived_channel, 1)
                @info "Program terminated."
            elseif !haskey(map, parse(Int, target_road_segment_id))
                @info "Segment $target_road_segment_id does not exist in map."
            else
                target_road_segment_id = parse(Int, target_road_segment_id)
                println("En route to segment $target_road_segment_id")

                #@async localize(gps_channel, imu_channel, localization_state_channel, gt_channel, controls)
                #@async perception(cam_channel, localization_state_channel, perception_state_channel)
                @async decision_making(gt_channel, perception_state_channel, map, socket, controls, target_road_segment, arrived_channel)

                while controlled && isopen(socket)
                    sleep(0.01)
                    cmd = VehicleCommand(controls.steering_angle, controls.target_speed, controlled)
                    serialize(socket, cmd)
                    if isready(arrived_channel)
                        break
                    end
                end
            end
        end
    end
end
