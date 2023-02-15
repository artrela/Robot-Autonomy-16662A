# Homework 1 Notes

## Force Control (Open-Loop)
*Resource: https://www.youtube.com/watch?v=M1U629sREiY*
<br>

### Purpose:
- Here we will institute an open-loop force controller that can apply 15N to a wall
- This works by using the Jacobian to transform force readings at the end-effector into joint torques (which you can then control)

### Steps:
1. data.body() specifies which part of the body data to look at 
2. Here we are getting the Jacobian with the mj.mj_jacBody() 
    - Because of Cpp memory allocation, all parameters must be passed (this library was adapted from Cpp)
    - This method splits the Jacobian into positional and rotational components 
    - This method will then update the Jacobian at each step
    - Concatenate positional on top of rotational
    - body.id is an integer identifier 
3. Set our desired "wrench" (x,y,z Forces & x,y,z Moments)
4. Transform the wrench force to joint forces
5. Add gravity compensatino and the open loop control


```python 
# Force control callback
def force_control(model, data): #TODO:

    # Implement a force control callback here that generates a force of 15 N along the global x-axis,
    # i.e. the x-axis of the robot arm base. You can use the comments as prompts or use your own flow
    # of code. The comments are simply meant to be a reference.

    # 1. Instantite a handle to the desired body on the robot
    body = data.body("hand")

    # 2. Get the Jacobian for the desired location on the robot (The end-effector)
    # This function works by taking in return parameters!!! Make sure you supply it with placeholder
    # variables
    jacp = np.zeros(shape=(3, 7)) # position 
    jacr = np.zeros(shape=(3, 7)) # rotation

    mj.mj_jacBody(model, data, jacp, jacr, body.id)

    J = np.vstack((jacp, jacr))

    # 3. Specify the desired force in global coordinates
    wrench_des = np.array([15, 0, 0, 0, 0, 0])

    # 4. Compute the required control input using desied force values
    torque_des = np.transpose(J) @ wrench_des

    # 5. Set the control inputs
    data.ctrl[:7] = torque_des + data.qfrc_bias[:7]

    # DO NOT CHANGE ANY THING BELOW THIS IN THIS FUNCTION

    # Force readings updated here
    force[:] = np.roll(force, -1)[:]
    force[-1] = data.sensordata[2]

    #! print to verify!
    print(force[-1]) 
```

## Force 

