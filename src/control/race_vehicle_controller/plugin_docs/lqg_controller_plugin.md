# LQG Controller Plugin

## Purpose

This plugin implements the LQG algorithm for vehicle control. It was developed by Dominic Nightangle and made compliant with C++, editted some part of the algorithm to make consider turns and bank by Dvij Kalaria. For the theorotic details of the algoritm, readers are referred to [Vehicle Dynamics and Control book chapter 3](https://link.springer.com/book/10.1007/978-1-4614-1433-9) and also some part of this [reserch paper](https://ieeexplore.ieee.org/abstract/document/8792042)   

## Additional Publishers and Subscribers

None

## Additional Parameters

lqr_config: Parameters relevant to LQR which is part of the algorithm
    are_max_iter: Maximum no of iterations for solving DARE (Keep higher for accuracy)
    are_error_thresh: Assume DARE solution has converged if difference between consecutive steps less than this (Keep low for accuracy)
    state_performance: Parameters which determine Q and R matrices of LQR (exactly as per [here](https://ieeexplore.ieee.org/abstract/document/8792042))
        l_factor: Look ahead factor for vehicle speed. Lookahead distance is l_factor*speed + l_const. Keep higher to reduce oscillations at high speeds
        l_const: Constant to add to lookahead distance. Lookahead distance is l_factor*speed + l_const. Keep higher to reduce oscillations at low speeds
        rc_1_val: Magnitude of penalty for aggressive steering. Keep high for smoother control
        
    lqg_config: Parameters relevant to LQG main algorithm
      ts: Discrete time step
      min_speed_mps: Minimum speed passed to the algorithm. Speed = clip(speed,min_speed_mps,max_speed_mps)
      max_speed_mps: Maximum speed passed to the algorithm. Speed = clip(speed,min_speed_mps,max_speed_mps)
      max_steer_rad: Maximum steering outputted from algorithm. Final control = clip(Control,-max_steer_rad,+max_steer_rad)
      line_error_threshold: Not used
      initial_state: Intial LQG state of the vehicle. Keep zero if initial pose is close to the reference, algorithm will use EKF to lean towards the new observed state.
        x1: 
        x2: 
        x3: 
        x4: 
      initial_error_cov: Intital error covariance used for the kalman filter
        p0_1: 
        p0_2: 
        p0_3: 
        p0_4: 
    
    lkf_config: Parameters relevant to the linear kalman filter used in the algorithm
      model_error: Model error at each step used by the kalman filter 
        qf_1: 
        qf_2: 
        qf_3: 
        qf_4: 
      measurement_error: Measurement error at each step used by the kalman filter
        rf_1: 
        rf_2: 
        rf_3: 
        rf_4: 
    
    lqg_model:
      ov_ud_steer_lookup: Not used
      ts: Controller sample time for discretization
      m: total mass (kg)
      mf: Front mass : m*lr/(lf+lr)
      mr: Rear mass : m*lf/(lf+lr)
      L: wheel base (meters)
      lf: Length of the front wheel axle from COM
      lr: Length of the rear wheel axle from COM
      cf: front tire cornering stiffness
      cr: rear tire cornering stiffness
      iz: Rotational Moment of Inertia

## Inner-Working

The plugin to use the lqg controller. Most of the lqg core code resides in src/control/lqg_controller. The lqg_controller_plugin is to interface with that code. For the theorotic details of the algoritm, readers are referred to [Vehicle Dynamics and Control book chapter 3](https://link.springer.com/book/10.1007/978-1-4614-1433-9) and also some part of this [reserch paper](https://ieeexplore.ieee.org/abstract/document/8792042). It mostly relies on a dunamic bicycle model based on linear tire slip to force curve. The plugin takes as input the follows :-

- Vehicle Kinematic State
- Target Trajectory Command

It also checks if the trajectory is empty or if the model is not initialized yet or if any of the inputs are not received yet.

If either of the conditions above in true, it returns false in `compute_control_command`, **which causes the RVC to skip querying the rest of the plugins, and not publish any control command message.** Therefore, this plugin may not be the solution to everything, because it prevents alternative safety behaviors in the RVC (such as emergencing stopping when too many NaNs have occured). Instead, it expects the low level interface to have a timeout on the RVC output, and trigger safety related logics on its own.

## User Manual

Place this plugin as a controller plugin. 

Special note : This plugin doesn't take into account steering bias. It expects that steering bias corrections will be followed by a plugin later. Also, it doesn't explicitly consider obstacle avoidance but assumes th the target trajectory it got from the path planner is obstacle free and that the lateral error is sufficiently low to not hit the obstacle.
