# docking.py

#import necessary modules.
import krpc
import time
import numpy as np


def acquire_target(conn,vessel,target):
	# conn is a krpc.connect() object, vessel is a vessel object, target is a string.


	# vessel_list is a list of all vessels currently in use.
	vessel_list = conn.space_center.vessels

	# n is the number of vessels currently in use.
	n = len(vessel_list)

	# the loop cycles through all vessels in the list, and if the vessel's name matches the target string provided, it sets it as the target vessel. 
	for i in range(0,n):
		vessel_name = vessel_list[i].name
		if vessel_name == target:
			conn.space_center.target_vessel = vessel_list[i]
			return None


def orbit_finder(conn,vessel):
	# IMPORTANT: The more circular the vessel's initial orbit is, the more accurate the orbit finder will be.

	# conn is a krpc.connect() object, vessel is a vessel object.


	while True:
		# retrieves the target vessel as a vessel object.
		target = conn.space_center.target_vessel

		# finds the period of the current orbit.
		vessel_period = vessel.orbit.period

		# creates an array of times ranging from 0 to the vessel's period to be used to test where best to fire engines for a Hohmann transfer.
		times = np.linspace(0,vessel_period,1000)

		# retrieves the current time.
		current_time = conn.space_center.ut

		# retrieves the vessel's current speed.
		current_v = vessel.orbit.speed

		#retrieves the current orbital radius.
		vessel_radius = vessel.orbit.radius

		# the Hohmann transfer orbit used to intercept the target will have a semi-major axis of the average of the original vessel orbit and the target orbit.
		needed_a = (vessel_radius + target.orbit.radius)/2

		# retrieves the gravitational constant.
		G = conn.space_center.g

		# retrieves the mass of the body the vessel is orbiting.
		M = vessel.orbit.body.mass


		# plugs in the previously defined variables into the vis-viva equation to find the needed velocity.
		needed_v = np.sqrt(G*M*(2/vessel_radius - 1/needed_a))

		# finds the delta V needed for the maneuver.
		delta_v = needed_v - current_v

		# the program loops over each time in the times array and creates a maneuver node at that time with the needed delta V. 
		# Using the orbit created by the node, it then checks how close the target and vessel will be when the vessel is at apoapsis (since that is when the vessel will cross the target's orbit).
		# If the distance is close enough, the node is returned to be used as a variable in later programs.
		# If the distance is not close enough, the node is deleted and the program moves on to the next time.
		for i in times:
			maneuver_time = current_time + i
			node = vessel.control.add_node(maneuver_time, prograde=delta_v) 
			possible_orbit = node.orbit

			time_to_pred_apoapsis = possible_orbit.period / 2
			node_position = np.array(node.position(vessel.orbit.body.reference_frame))
			node_unit_vector = np.array([node_position[0]/np.linalg.norm(node_position),node_position[1]/np.linalg.norm(node_position),node_position[2]/np.linalg.norm(node_position)])
			possible_apoapsis = possible_orbit.apoapsis

			vessel_position_at_apoapsis = possible_apoapsis * -1 * node_unit_vector
			target_position_at_apoapsis = target.orbit.position_at(maneuver_time+time_to_pred_apoapsis,target.orbit.body.reference_frame)
			dist_vector = [vessel_position_at_apoapsis[0] - target_position_at_apoapsis[0],vessel_position_at_apoapsis[1] - target_position_at_apoapsis[1],vessel_position_at_apoapsis[2] - target_position_at_apoapsis[2]]
			dist = np.linalg.norm(dist_vector)
			if dist < 1700:
				return node
			else:
				vessel.control.remove_nodes()

		# If no maneuvers result in a close enough distance, the program warps the vessel forward in time, and then restarts the search.
		# Since the vessel has fast-forwarded, new times will be available to check.
		conn.space_center.rails_warp_factor = 4
		time.sleep(3)
		conn.space_center.rails_warp_factor = 0




def first_slowdown(conn,vessel):
	# conn is a krpc.connect() object, vessel is a vessel object.

	# retrieves the target vessel as a vessel object.
	target = conn.space_center.target_vessel

	# initializes a reference frame centered on the target. This makes it easy to measure how far the vessel is from the target and to get the velocity relative to the target.
	target_ref = target.orbital_reference_frame

	# sets the autopilot reference frame to the target-centered reference frame.
	vessel.auto_pilot.reference_frame = target_ref

	# engages the autopilot.
	vessel.auto_pilot.engage()

	while True:
		# returns the velocity of the vessel relative to the target as a tuple.
		velocity_vector = vessel.flight(target_ref).velocity

		# using the velocity vector, it sets the direction the autopilot should point the vessel to retrograde.
		vessel.auto_pilot.target_direction = (-velocity_vector[0],-velocity_vector[1],-velocity_vector[2])

		# retrieves the current time.
		current_time = conn.space_center.ut

		# returns the position of the vessel relative to the target as a tuple.
		current_position = vessel.orbit.position_at(current_time,target_ref) 

		# finds the current distance by finding the magnitude of the current position.
		current_distance = np.linalg.norm(current_position)

		# when the vessel is 15000 meters away, the engines turn on to slow the vessel relative to the target.
		# once the vessel's relative speed is less than 40 m/s, the engines turn off and this function exits.
		if current_distance < 15000:
			conn.space_center.rails_warp_factor = 0
			vessel.control.throttle = 1

			if vessel.flight(target_ref).speed < 40:
				vessel.control.throttle = 0
				return None

		# when the distance is greater than 30000 meters, the vessel warps to save time. Then it goes back to normal time flow.
		elif current_distance > 30000:
			conn.space_center.rails_warp_factor = 4
		else:
			conn.space_center.rails_warp_factor = 0



def zeroing_inclination(conn,vessel):
	# conn is a krpc.connect() object, vessel is a vessel object.

	# retrieves the target vessel as a vessel object.
	target = conn.space_center.target_vessel

	# sets the autopilot reference frame to the vessel's orbital reference frame (so that the normal/anti-normal directions are basis vectors).
	vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame

	# engages the autopilot.
	vessel.auto_pilot.engage()


	while True:

		# finds the relative incline of the target (in radians).
		incline = vessel.orbit.relative_inclination(target)

		# finds the angular separation from the periapsis to the ascending node (in radians).
		an_location = vessel.orbit.true_anomaly_at_an(target)

		# finds the angular separation between the vessel and the periapsis (in radians).
		vessel_location = vessel.orbit.true_anomaly

		# when the incline is small enough, the engines should turn off and the function will exit.
		if abs(incline) < 0.0005:
			vessel.control.throttle = 0
			return None

		# if the inclination is positive, the vessel points in the anti-normal direction. If the inclination is negative, the vessel points in the normal direction.
		if incline > 0:
			vessel.auto_pilot.target_direction = (0,0,-1)
		else:
			vessel.auto_pilot.target_direction = (0,0,1)

		# if the angular separation of the ascending node and the vessel is low, the engines should fire. Otherwise, the engines should not fire.
		if abs(an_location-vessel_location) < 0.05:
			vessel.control.throttle = 1
		else:
			vessel.control.throttle = 0

		# controls if the vessel should warp forward in time depending on how close the ascending node and the vessel are (to save time).
		if abs(an_location-vessel_location) > np.pi / 12:
			conn.space_center.rails_warp_factor = 4
		else:
			conn.space_center.rails_warp_factor = 0


def execute_transfer_burn(conn,vessel,node):
	# conn is a krpc.connect() object, vessel is a vessel object, and node is a node object.

	# arrived is a boolean that will ensure that if the vessel gets to the maneuver node, it will not stop the engines until the maneuver is complete.
	arrived = False


	while True:
		# sets the reference frame to the vessel's orbital reference frame so that the velocity vector is a basis vector.
		vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame

		# engages the autopilot.
		vessel.auto_pilot.engage()

		# sets the target direction to prograde.
		vessel.auto_pilot.target_direction = (0,1,0)


		# if the vessel has ever been 20 seconds from the node, fire the engines.
		if node.time_to < 20 or arrived == True:
			arrived = True
			vessel.control.throttle = 1

		# once the vessel orbit's apoapsis is higher than the node orbit's apoapsis, the burn is complete and the engines turn off, the node is deleted, and this function exits.
		if node.orbit.apoapsis_altitude < vessel.orbit.apoapsis_altitude:
			vessel.control.throttle = 0
			node.remove()
			return None


		# determines if the vessel should warp forward in time to save time.
		if node.time_to > 400:
			conn.space_center.rails_warp_factor = 4
		else:
			conn.space_center.rails_warp_factor = 0




def second_slowdown(conn,vessel):
	# conn is a krpc.connect() object, vessel is a vessel object.

	# retrieves the target vessel as a vessel object.
	target = conn.space_center.target_vessel

	# initializes a reference frame centered on the target. This makes it easy to measure how far the vessel is from the target and to get the velocity relative to the target.
	target_ref = target.orbital_reference_frame

	# sets the autopilot reference frame to the target-centered reference frame.
	vessel.auto_pilot.reference_frame = target_ref

	# engages the autopilot.
	vessel.auto_pilot.engage()


	while True:
		# returns the velocity of the vessel relative to the target as a tuple.
		velocity_vector = vessel.flight(target_ref).velocity

		# using the velocity vector, it sets the direction the autopilot should point the vessel to prograde.
		vessel.auto_pilot.target_direction = (velocity_vector[0],velocity_vector[1],velocity_vector[2])

		# waits until the vessel is pointed in the right direction.
		vessel.auto_pilot.wait()

		# retrieves the current time.
		current_time = conn.space_center.ut

		# returns the position of the vessel relative to the target as a tuple.
		current_position = vessel.orbit.position_at(current_time,target_ref) 

		# finds the current distance by finding the magnitude of the current position.
		current_distance = np.linalg.norm(current_position)

		# turns on RCS thrusters.
		vessel.control.rcs = True

		# sets the RCS thrusters to fire retrograde.
		vessel.control.forward = -1

		# once the vessel's relative speed is less than 10 m/s, the RCS turns off, the autopilot is disengaged, and the function exits.
		if vessel.flight(target_ref).speed < 10:
			vessel.control.rcs = False
			vessel.auto_pilot.disengage()
			return None



def third_slowdown(conn,vessel):
	# conn is a krpc.connect() object, vessel is a vessel object.

	# retrieves the target vessel as a vessel object.
	target = conn.space_center.target_vessel

	# initializes a reference frame centered on the target. This makes it easy to measure how far the vessel is from the target and to get the velocity relative to the target.
	target_ref = target.orbital_reference_frame

	# sets the autopilot reference frame to the target-centered reference frame.
	vessel.auto_pilot.reference_frame = target_ref

	# engages the autopilot.
	vessel.auto_pilot.engage()

	# initializes the list that will store the differences between the optimal dot product and the actual dot product.
	differences = []

	# initializes the list that will store the responses the system has tried to maximize the dot product.
	responses = []

	# initializes the list that will store the distances the vessel is from the target. Only is useful in the rare case that the vessel starts moving away from the vessel.
	distances = []


	# retrieves the current time.
	current_time = conn.space_center.ut

	# returns the position of the vessel relative to the target as a tuple.
	current_position = vessel.orbit.position_at(current_time,target_ref) 

	# finds the current distance by finding the magnitude of the current position.
	current_distance = np.linalg.norm(current_position)

	# current_position is a vector from the target to the vessel, so the negative of it is a vector from the vessel to the target (the "target" symbol on the navball).
	# this sets the autopilot to point right at the target.
	vessel.auto_pilot.target_direction = (-current_position[0],-current_position[1],-current_position[2])

	# waits until the autopilot is pointing in the right direction.
	vessel.auto_pilot.wait()

	# activates RCS thrusters.
	vessel.control.rcs = True


	while True:
		# retrieves the current time.
		current_time = conn.space_center.ut

		# returns the position of the vessel relative to the target as a tuple.
		current_position = vessel.orbit.position_at(current_time,target_ref) 

		# finds the current distance by finding the magnitude of the current position.
		current_distance = np.linalg.norm(current_position)

		# appends the current distance to the distances list.
		distances.append(current_distance)

		# current_position is a vector from the target to the vessel, so the negative of it is a vector from the vessel to the target (the "target" symbol on the navball).
		# this sets the autopilot to point right at the target.
		vessel.auto_pilot.target_direction = (-current_position[0],-current_position[1],-current_position[2])

		# if the speed is less than 10 m/s and the distance is greater than 1200 meters, the RCS thrusts prograde.
		# if the current distance is less than 1200 meters and the speed is greater than 2 m/s, the RCS thrusts retrograde.
		# otherwise, the RCS does not fire forward nor backward.
		if vessel.flight(target_ref).speed < 10 and current_distance > 1200:
			vessel.control.forward = 1
		elif current_distance < 1200 and vessel.flight(target_ref).speed > 2:
			vessel.control.forward = -1
		else:
			vessel.control.forward = 0

		# finds the current relative velocity of the vessel.
		current_velocity = vessel.flight(target_ref).velocity

		# calculates the dot product between the velocity vector and the target vector.
		dot_prod = np.dot([current_velocity[0],current_velocity[1],current_velocity[2]],[-current_position[0],-current_position[1],-current_position[2]])

		# returns the magnitude of the current velocity.
		current_speed = vessel.flight(target_ref).speed

		# calculates the maximum dot product based on the magnitudes of the velocity and target vectors.
		optimal_dot = current_speed * current_distance

		# finds the difference between the optimal and actual dot.
		difference = optimal_dot - dot_prod

		# appends this difference to the differences list.
		differences.append(difference)

		# initializes the list of actions the vessel can take to align the velocity vector with the target vector.
		corrective_actions = ['up','down','left','right']

		# once the length of differences is greater than 10, if the difference has decreased, the current action is set to what the most recent action was.
		# If the difference has increased, it randomly selects another action to take as the current action.
		# The current action is then appended to the responses list.
		if len(differences) > 10:
			if differences[-1] < differences[-2]:
				current_action = responses[-1]
			else:
				alternatives = []
				for action in corrective_actions:
					if action != responses[-1]:
						alternatives.append(action)
				current_action = np.random.choice(alternatives)
			responses.append(current_action)
		else:
			# if the length of differences is less than 10, it picks random actions no matter what and appends it to the responses list.
			current_action = np.random.choice(corrective_actions)
			responses.append(current_action)

		# based on the distance, this determines the zone around the target vector where the velocity vector is considered "aligned enough".
		# this angle is increased when the vessel is closer because the lower speed means the thrusters have greater effect on the velocity vector.
		if current_distance < 1200:
			min_angle = 6
		else:
			min_angle = 3

		# the minimum dot product sets the cut off for if a dot product is considered "good enough". 
		# Without this, the RCS would continually fire throughout the flight, which is inefficient.
		minimum_dot = optimal_dot * np.cos(np.deg2rad(min_angle))

		# if the vectors are not very closely aligned, the vessel will use RCS according to the last response in the response list.
		if dot_prod < minimum_dot:
			if responses[-1] == 'up':
				vessel.control.up = 1
				vessel.control.right = 0
			elif responses[-1] == 'down':
				vessel.control.up = -1
				vessel.control.right = 0
			elif responses[-1] == 'left':
				vessel.control.right = -1
				vessel.control.up = 0
			elif responses[-1] == 'right':
				vessel.control.right = 1
				vessel.control.up = 0
		else:
			# if the vectors are closely aligned, the up/down and left/right RCS is turned off.
			vessel.control.up = 0
			vessel.control.right = 0


		# if the vessel is less than 180 meters from the target, the RCS is deactivated and this function exits.
		# this happens at 180 meters because in the final approach, the target docking port is selected.
		# the target port can only be selected within 200 meters because that is when the target vessel's part list is loaded.
		if current_distance < 180:
			vessel.control.rcs = False
			return None


		# the time.sleep() functions allow the program to fire the RCS long enough to determine if the change in velocity was beneficial.
		# when closer, the sleep time is decreased because lower velocities mean it takes less time to produce a significant change in velocity.
		if current_distance < 1200:
			time.sleep(0.5)
			# in rare cases, the vessel could start slowing down and eventually start moving away from the target.
			# this ensures that the speed never goes lower than 1.5 m/s.
			if current_speed < 1.5:
				vessel.control.forward = 1
			else:
				vessel.control.forward = 0
		else:
			time.sleep(1)






def final_approach(conn,vessel,vessel_port,tar_port):
	# conn is a krpc.connect() object, vessel is a vessel object, vessel_port is a string, tar_port is a string.

	# retrieves a list of the docking ports on the vessel where the title of the port matches the vessel_port string.
	vessel_port_list = vessel.parts.with_title(vessel_port)

	# cycles through the docking port list and selects one that is not already docked.
	# it sets this port to the controlling port, and makes sure the port's shield (if applicable) is deactivated.
	for port in vessel_port_list:
		if port.docking_port.state != conn.space_center.DockingPortState.docked:
			vessel.parts.controlling = port
			port.docking_port.shielded = False
			controlling_port = port.docking_port

	# retrieves the target vessel as a vessel object.
	target = conn.space_center.target_vessel

	# retrieves a list of all the docking ports on the target with the same title as the tar_port string.
	target_port_list = target.parts.with_title(tar_port)

	# cycles through the target port list and selects one that is not already docked.
	# it sets this port as the target port, and deactivates the port's shield (if applicable).
	for port in target_port_list:
		if port.docking_port.state != conn.space_center.DockingPortState.docked:
			conn.space_center.target_docking_port = port.docking_port
			port.docking_port.shielded = False
	

	# retrieves the target port as a part object (for ease of use later).
	target_port = conn.space_center.target_docking_port

	# creates a reference frame with the origin centered on the target port and the y-axis pointing directly perpendicular to the port.
	target_ref = target_port.part.reference_frame

	# sets the autopilot's reference frame to this new reference frame.
	vessel.auto_pilot.reference_frame = target_ref

	# engages the autopilot.
	vessel.auto_pilot.engage()

	# sets the vessel to point so that its controlling port is parallel to the target port.
	vessel.auto_pilot.target_direction = (0,-1,0)

	# sets the vessel's roll to 0.
	vessel.auto_pilot.target_roll = 0

	# waits until the vessel is pointed in the right direction.
	vessel.auto_pilot.wait()

	# initializes the list that will store the differences between the optimal dot product and the actual dot product.
	differences = []

	# initializes the list that will store the responses the system has tried to maximize the dot product.
	responses = []

	while True:
		# if the docking was successful, the function exits.
		if controlling_port.state == conn.space_center.DockingPortState.docked:
			return None

		# activates the RCS.
		vessel.control.rcs = True

		# finds the position of the controlling port relative to the target port as a tuple.
		vessel_position = controlling_port.position(target_ref)

		# sets the vessel's roll to 0.
		vessel.auto_pilot.target_roll = 0

		# sets the vessel to point so that its controlling port is parallel to the target port.
		vessel.auto_pilot.target_direction = (0,-1,0)


		# finds the current speed of the vessel relative to the target.
		current_speed = vessel.flight(target_ref).speed

		# finds the relative velocity of the vessel as a tuple.
		rel_velocity = vessel.flight(target_ref).velocity

		# sets the target vector to the negative of the vector extending from the target to the vessel (so this vector is from the vessel to the target).
		target_dir = (-vessel_position[0],-vessel_position[1],-vessel_position[2])

		# finds the magnitude of the target vector.
		target_dir_mag = np.linalg.norm(target_dir)

		# finds the dot product of the velocity vector and the target vector.
		dot_prod = np.dot(rel_velocity,target_dir)

		# finds the maximum dot product based on the magnitudes of the vectors.
		optimal_dot = current_speed * target_dir_mag

		# finds the difference between the optimal and actual dot.
		difference = optimal_dot - dot_prod

		# appends the difference to the differences list.
		differences.append(difference)


		# initializes the list of actions the vessel can take to align the velocity vector with the target vector.
		corrective_actions = ['up','down','left','right']

		# once the length of differences is greater than 3, if the difference has decreased, the current action is set to what the most recent action was.
		# If the difference has increased, it randomly selects another action to take as the current action.
		# The current action is then appended to the responses list.
		if len(differences) > 3:
			if differences[-1] < differences[-2]:
				current_action = responses[-1]
			else:
				alternatives = []
				for action in corrective_actions:
					if action != responses[-1]:
						alternatives.append(action)
				current_action = np.random.choice(alternatives)
			responses.append(current_action)
		else:
			# if the length of differences is less than 3, it picks random actions no matter what and appends it to the responses list.
			current_action = np.random.choice(corrective_actions)
			responses.append(current_action)

		# determines at what angle the vectors are considered "close enough". This becomes less as the vessel gets closer to the target, with the minimum being 2 degrees.
		min_angle = 2 + target_dir_mag * 0.025

		# finds the minimum dot product that is considered "good enough".
		minimum_dot = optimal_dot * np.cos(np.deg2rad(min_angle))
		

		# determines the dot product of the target vector with the vector pointing into and perpendicular to the target port.
		# this measurement is used to determine if the vessel is actually coming in backward to the target port, so that can be fixed.
		target_heading_dot = np.dot(target_dir,(0,-1,0))

		# the vessel should only try to dock if it is coming in at less than 60 degrees from the normal of the port.
		target_heading_dot_cutoff = target_dir_mag *np.cos(np.deg2rad(60))
		

		# if the vessel is not on the correct side of the target, and the speed is less than 3 m/s, the RCS fires backward to move the vessel to the correct side.
		if target_heading_dot < target_heading_dot_cutoff:
			if vessel.flight(target_ref).speed < 3:
				vessel.control.forward = -1
				vessel.control.up = 0
				vessel.control.right = 0
			else:
				vessel.control.forward = 0
				vessel.control.up = 0
				vessel.control.right = 0
		else:
			# if the vessel is on the right side, the velocity normal to the port should be between 0.3 and 0.5 m/s toward the port.
			# the RCS is fired accordingly to make that so.
			if vessel.flight(target_ref).velocity[1] > -0.3:
				vessel.control.forward = 1
			elif vessel.flight(target_ref).velocity[1] < -0.5:
				vessel.control.forward = -1
			else:
				vessel.control.forward = 0

		# if the velocity and target vectors are not very closely aligned, the vessel will use RCS according to the last response in the response list.
		if dot_prod < minimum_dot:
			if responses[-1] == 'up':
				vessel.control.up = 0.4
				vessel.control.right = 0
			elif responses[-1] == 'down':
				vessel.control.up = -0.4
				vessel.control.right = 0
			elif responses[-1] == 'left':
				vessel.control.right = -0.4
				vessel.control.up = 0
			elif responses[-1] == 'right':
				vessel.control.right = 0.4
				vessel.control.up = 0
		else:
			# if the vectors are closely aligned, the up/down and left/right RCS is turned off.
			vessel.control.up = 0
			vessel.control.right = 0

		# if the vessel is on the right side of the target, the internal delay of the function is set to 0.15 seconds. Otherwise, the internal delay is set to 0.5 seconds.
		# this delay exists to make sure there is enough time for the RCS to have a meaningful effect on the velocity vector.
		if dot_prod > 0:
			time.sleep(0.15)
		else:
			time.sleep(0.5)


			
