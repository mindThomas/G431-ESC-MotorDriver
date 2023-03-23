Two ways of doing it:

1. Linear exogenous oscillator system 
2. Constant-velocity system with non-linear output equation

In both cases it requires an Extended Kalman Filter to do the tracking/estimation.

An oscillator can be defined from the differential equation and sine/cosine relationships:
d cos(theta(t)) / dt = dcos/dtheta * dtheta/dt
d cos(theta(t)) / dt = -sin(theta) * omega

d -sin(theta(t)) / dt = - dsin(theta)/dtheta * dtheta/dt
d -sin(theta(t)) / dt = -cos(theta) * omega

So we can describe this with two states:
X1 = cos(theta)
X2 = -sin(theta)

And thereby the differential equations governing the oscillator are:
dX1/dt = omega * X2
dX2/dt = -omega * X1

# Linear exogenous oscillator system 
In a linear exogenous oscillator system the oscillator frequency is captured within the A matrix.
The output equation is thus identity since the frequency is captured within the A matrix and the amplitude is captured as the magnitude of the state vector.

# Constant-velocity system with non-linear output equation
The states of the constant-velocity system is the current singwave angle and the angular velocity, omega. 
The system model is thus linear and can be encoded in the A matrix.
However to capture and track the sine-wave the output matrix is used to convert the angle into the sine-wave measurement.
This method is also described in the book: Fundamentals of Kalman filtering - A practical approach