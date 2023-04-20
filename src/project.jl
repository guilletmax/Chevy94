
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

    target_road_segment_id = -1
    prev_segment_id = target_road_segment_id
    ego_vehicle_id = 0

    msg = deserialize(socket)
    @info msg

    ready = false
    @async while isopen(socket)
        sleep(0.001)
        state_msg = deserialize(socket)
        measurements = state_msg.measurements
        if target_road_segment_id == -1 && prev_segment_id != state_msg.target_segment
            target_road_segment_id = state_msg.target_segment
            @info "Changed target road segment"
            ready = true
        end
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
    while true
        sleep(0.5)
        if ready
            take!(arrived_channel)
            @info "En route to segment $target_road_segment_id"

            #@async localize(gps_channel, imu_channel, localization_state_channel, gt_channel, controls)
            #@async perception(cam_channel, localization_state_channel, perception_state_channel)
            @async decision_making(gt_channel, perception_state_channel, map, socket, controls, prev_segment_id, target_road_segment_id, arrived_channel)

            while controlled && isopen(socket)
                sleep(0.0001)
                cmd = VehicleCommand(controls.steering_angle, controls.target_speed, controlled)
                serialize(socket, cmd)
                if isready(arrived_channel)
                    ready = false
                    prev_segment_id = target_road_segment_id
                    target_road_segment_id = -1
                    break
                end
            end
        end
    end
end








#     loading_zone_ids = get_loading_zone_segment_ids(map)
#     arrived_channel = Channel{Int}(1)
#     put!(arrived_channel, 1)
#     terminate = false
#     while !terminate
#         if isready(arrived_channel)
#             take!(arrived_channel)
#             @info "Please enter your destination segment ID, or q to terminate program. Available destinations with loading zones are:"
#             @info loading_zone_ids
#             target_road_segment_id = readline()
#             if target_road_segment_id == "q"
#                 terminate = true
#                 put!(arrived_channel, 1)
#                 @info "Program terminated."
#             elseif !(parse(Int, target_road_segment_id) in loading_zone_ids) #!haskey(map, parse(Int, target_road_segment_id))
#                 @info "Segment $target_road_segment_id is not a suitable destination."
#                 push!(arrived_channel, 1)
#             else
#                 target_road_segment_id = parse(Int, target_road_segment_id)
#                 @info "En route to segment $target_road_segment_id"

#                 #@async localize(gps_channel, imu_channel, localization_state_channel, gt_channel, controls)
#                 #@async perception(cam_channel, localization_state_channel, perception_state_channel)
#                 @async decision_making(gt_channel, perception_state_channel, map, socket, controls, target_road_segment_id, arrived_channel)

#                 while controlled && isopen(socket)
#                     sleep(0.01)
#                     cmd = VehicleCommand(controls.steering_angle, controls.target_speed, controlled)
#                     serialize(socket, cmd)
#                     if isready(arrived_channel)
#                         break
#                     end
#                 end
#             end
#         end
#     end
# end

function get_loading_zone_segment_ids(map)
    return [segment_id for (segment_id, segment) in map if VehicleSim.loading_zone in segment.lane_types]
end
