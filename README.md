# Vehicle Dynamics Model

This model represents the lateral dynamics of a vehicle, including its response to steering and external disturbances. It is based on the bicycle model, which is a simplified representation of a vehicle that captures the essential dynamics of lane changes and cornering.

## System Description

The system consists of four interconnected components:

- **Body:** Represents the vehicle's center of mass.
- **Front tires:** Model the behavior of the front tires, including slip angles and side forces.
- **Rear tires:** Model the behavior of the rear tires, including slip angles, side forces, and Yaw Moment.
- **Steering:** Represents the driver's steering input, which affects the yaw rate of the vehicle.

## Model Equations

The model equations describe the relationship between the steering input and the vehicle's lateral response. They are represented by a state-space model, which consists of a set of differential equations and algebraic equations.


x  = [x, y, psi, v]' - State vector <p></p>
u  = delta - Steering input <p></p>
y  = [beta, psi_dot]' - Measurement vector


<p align="center">
  <img width="500" height="500" alt="image" src="https://github.com/itsVinM/Vehicle_dynamics/assets/85823292/1883f2cb-f13a-4aa3-bb6d-7ad0436170d8">
</p>

where:

- \(x\) is the state vector, representing the vehicle's position (\(x, y\)), yaw angle (\(\psi\)), and forward velocity (\(v\)).
- \(u\) is the steering input, representing the steering angle (\(\delta\)).
- \(y\) is the measurement vector, representing the sideslip angle (\(\beta\)) and yaw rate (\(\dot{\psi}\)).
- \(A\) is the state-space matrix, which describes the dynamics of the vehicle.
- \(B\) is the input matrix, which relates the steering input to the state derivatives.
- \(C\) is the output matrix, which relates the state variables to the measurements.

## Model Parameters

The model parameters include:

- \(m:\) Vehicle mass
- \(I_x:\) Moment of inertia about the x-axis
- \(I_z:\) Moment of inertia about the z-axis
- \(L:\) Wheelbase
- \(T:\) Tire slip angle stiffness
- \(J_z:\) Roll moment of inertia

These parameters can be estimated from vehicle specifications or measured using specialized test equipment.

## Model Validation

The model can be validated by comparing its simulation results to experimental data obtained from real-world vehicle tests. This involves driving the vehicle through a series of maneuvers, measuring its lateral responses, and comparing these measurements to the model's predictions.

## Applications

The vehicle dynamics model can be used for a variety of applications, including:

- **Vehicle control design:** Design controllers to enhance the vehicle's handling characteristics, such as stability control and lane-keeping assist systems.
- **Vehicle simulation:** Simulate the vehicle's performance in different driving conditions, such as cornering, braking, and acceleration, to assess safety and drivability.
- **Vehicle design optimization:** Optimize the vehicle's design parameters, such as suspension geometry and tire characteristics, to improve handling and performance.

## Conclusion

The vehicle dynamics model provides a simplified yet powerful representation of a vehicle's lateral dynamics. It can be used for a variety of applications, including vehicle control, simulation, and design optimization.

<p align="center">
  <img width="500" height="500" alt="image" src="https://github.com/itsVinM/Basic_vehicle_dynamics/assets/85823292/44d2a12d-4342-4331-b55f-5db80d760fb1">
</p>
