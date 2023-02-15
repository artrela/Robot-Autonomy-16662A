import mujoco as mj
from mujoco import viewer
import numpy as np
import math
import quaternion
from scipy.spatial.transform import Rotation as R


# Set the XML filepath
xml_filepath = "../franka_emika_panda/panda_nohand_torque.xml"

################################# Control Callback Definitions #############################

# Control callback for gravity compensation
def gravity_comp(model, data):

    # data.ctrl exposes the member that sets the actuator control inpuconda install -c conda-forge quaternionts that participate in the
    # physics, data.qfrc_bias exposes the gravity forces expressed in generalized coordinates, i.e.
    # as torques about the joints

    data.ctrl[:7] = data.qfrc_bias[:7]

# Force control callback
def force_control(model, data): #TODO:

    """ Generate Force Control 
    1. Instantite a handle to the desired body on the robot
    2. Get the Jacobian for the desired location on the robot (The end-effector)
    3. Specify the desired force in global coordinates
    4. Perform open loop control by turning the wrench from the task space to joint space
    5. Get the control inputs
    """
    
    # 1 
    body = data.body("hand")

    # 2
    jacp = np.zeros(shape=(3, 7)) # position 
    jacr = np.zeros(shape=(3, 7)) # rotation

    mj.mj_jacBody(model, data, jacp, jacr, body.id)

    J = np.vstack((jacp, jacr))[:, :7]

    # 3 
    wrench_des = np.transpose(np.array([15, 0, 0, 0, 0, 0]))

    # 4 
    torque_des = np.transpose(J) @ wrench_des

    #5 
    data.ctrl[:7] = torque_des+data.qfrc_bias[:7]

    # DO NOT CHANGE ANY THING BELOW THIS IN THIS FUNCTION

    # Force readings updated here
    force[:] = np.roll(force, -1)[:]
    force[-1] = data.sensordata[2]

    #! print to verify!
    print(force[-1])
    
    # DO NOT CHANGE ANY THING BELOW THIS IN THIS FUNCTION

    # Force readings updated here
    force[:] = np.roll(force, -1)[:]
    force[-1] = data.sensordata[2]

    
# Control callback for an impedance controller
def impedance_control(model, data): #TODO:
    """Run impedance control

    Args:
        model : mucojo model values
        data : link & body data for mucojo model

    1. Set the gains for the controller
        - the gain should be desired_force/distance behind the wall
    2. create an object to access the hand data
    3. obtain the curr hand orientation -> translate to axis-angle 
    3. rot pos is not relevant to this problem
    5. set the desired position as behind the wall
    6. we dont want the ee to be moving
    7. generate the velocity of the ee
    8. calc the error
    9. Get the Jacobian at the desired location on the robot
        - This function works by taking in return parameters!!! Make sure you supply it with placeholder
        - variables
    10. Set the control inputs
        - print(np.shape(des_vel), np.shape(curr_vel))
    """
    
    # 1
    Kp = 30
    Kd = 4

    # 2
    body = data.body("hand")
    
    # 3 
    curr_rot = np.array([0,0,0])

    # 4 
    curr_pos = np.hstack((data.body("hand").xpos, curr_rot))

    # 5
    des_pos = np.hstack([0.59526372 + 0.5, 0.00142708, 0.59519669, 0,0,0])

    # 6
    des_vel = np.array([0,0,0,0,0,0]).T

    # 7
    curr_vel = np.zeros(shape=(6))
    mj.mj_objectVelocity(model, data, mj.mjtObj.mjOBJ_BODY, 7, curr_vel, True)

    # 8
    vel_err = des_vel - curr_vel
    pos_err = des_pos - curr_pos

    # 9
    jacp = np.zeros(shape=(3, 7)) # position 
    jacr = np.zeros(shape=(3, 7)) # rotation

    mj.mj_jacBody(model, data, jacp, jacr, body.id)

    J = np.vstack((jacp, jacr))[:, :7]

    # 10
    data.ctrl[:7] = data.qfrc_bias[:7] + J.T @ (Kp * (pos_err) + Kd*(vel_err))

    # DO NOT CHANGE ANY THING BELOW THIS IN THIS FUNCTION

    # Update force sensor readings
    force[:] = np.roll(force, -1)[:]
    force[-1] = data.sensordata[2]

    #! print to verify!
    print(force[-1])
    

def position_control(model, data):

    # Instantite a handle to the desired body on the robot
    body = data.body("hand")

    # Set the desired joint angle positions
    #desired_joint_positions = np.array([0,0,0,-1.57079,0,1.57079,-0.7853])

    #? Inverse Kinematics Solution
    desired_joint_positions = [0.5959219360587886, 0.31733877628027085, -0.7070529960556343, -2.078321363968197, -0.12071721426271223, 3.914944417807852, -2.8501142686556826]

    # Set the desired joint velocities
    desired_joint_velocities = np.array([0,0,0,0,0,0,0])

    # Desired gain on position error (K_p)
    Kp = 1000

    # Desired gain on velocity error (K_d)
    Kd = 1000

    # Set the actuator control torques
    data.ctrl[:7] = data.qfrc_bias[:7] + (Kp*(desired_joint_positions-data.qpos[:7]) + Kd*(np.array([0,0,0,0,0,0,0])-data.qvel[:7]))



####################################### MAIN #####################################

if __name__ == "__main__":
    
    # Load the xml file here
    model = mj.MjModel.from_xml_path(xml_filepath)
    data = mj.MjData(model)

    # Set the simulation scene to the home configuration
    mj.mj_resetDataKeyframe(model, data, 0)

    ################################# Swap Callback Below This Line #################################
    # This is where you can set the control callback. Take a look at the Mujoco documentation for more
    # details. Very briefly, at every timestep, a user-defined callback function can be provided to
    # mujoco that sets the control inputs to the actuator elements in the model. The gravity
    # compensation callback has been implemented for you. Run the file and play with the model as
    # explained in the PDF

    mj.set_mjcb_control(position_control) #TODO:

    ################################# Swap Callback Above This Line #################################

    # Initialize variables to store force and time data points
    force_sensor_max_time = 10
    force = np.zeros(int(force_sensor_max_time/model.opt.timestep))
    time = np.linspace(0, force_sensor_max_time, int(force_sensor_max_time/model.opt.timestep))

    # Launch the simulate viewer
    viewer.launch(model, data)   

    # Save recorded force and time points as a csv file
    force = np.reshape(force, (5000, 1))
    time = np.reshape(time, (5000, 1))
    plot = np.concatenate((time, force), axis=1)
    np.savetxt('force_vs_time.csv', plot, delimiter=',')