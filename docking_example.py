# docking_example.py

# imports necessary modules.
import krpc
import time
import docking

# connects to the krpc server and retrieves the active vessel.
conn = krpc.connect(name="docking_example")
vessel = conn.space_center.active_vessel


# asks the user for the names of the target, controlling port, and target port.
target_vessel = input("What is the name of the target vessel? ")
control_port = input("What is the name of the port the vessel should control from? ")
target_port = input("What is the name of the port the vessel should target? ")


# sets the target.
docking.acquire_target(conn,vessel,target_vessel)

# zeroes the inclination of the two orbits.
docking.zeroing_inclination(conn,vessel)

# returns a node that will result in an orbit that will have a close intercept with the target.
intercept_burn = docking.orbit_finder(conn,vessel)

# executes the maneuver node.
docking.execute_transfer_burn(vessel,intercept_burn)

# executes the first slowdown phase.
docking.first_slowdown(conn,vessel)

# executes the second slowdown phase.
docking.second_slowdown(conn,vessel)

# executes the third slowdown phase.
docking.third_slowdown(conn,vessel)

# executes the final approach.
docking.final_approach(conn,vessel,control_port,target_port)