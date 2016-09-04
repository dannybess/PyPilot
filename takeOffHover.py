from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

drone = connect('/dev/ttyS0', wait_ready=True)


def take_off_and_land(targetAlt):
	# 1.) Arm Drone
	# 2.) Fly to targetAlt
	while not drone.is_armbale:
		print "Waiting for drone to initialize, sleeping ..."
		time.sleep(1)

	print("Arm Motors")
	drone.mode = VehicleMode("GUIDED")
	drone.armed = True

	while not drone.armed:
		print "Drone is not armed, sleeping ..."
		time.sleep(1)

	print("Take Off")
	drone.simple_takeoff(targetAlt)

	while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print "Reached target altitude, landing."
            # Set Mode to Landing
            drone.mode = VehicleMode("LAND")
            break
        time.sleep(1)


def make_velocity_vector(velocity_x, velocity_y, velocity_z, duration):
	# Set up velocity mappings
	# velocity_x > 0 => fly North
	# velocity_x < 0 => fly South
	velocity_y > 0 = > fly East
	# velocity_y < 0 => fly West
	# velocity_z < 0 => ascend
	# velocity_z > 0 => descend
    msg = drone.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to drone on 1 Hz cycle
    for x in range(0,duration):
        drone.send_mavlink(msg)
        time.sleep(1)

take_off_and_land(str(sys.argv[1]))

# make_velocity_vector(-2, 0, 1, 1)
