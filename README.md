# Air Brakes
The Air Brakes control system primarily manages the vehicle’s energy state throughout its unpowered ascent phase of 
flight by modulating the rocket’s aerodynamic drag profile in a bid to regulate its apogee to the specified target of 
10,000 ft AGL with a tolerance of ± 500 ft. At the 2025 IREC, Clarkson University Rocketry (CUR) launched Veðrfölnir, 
last year’s rocket and Air Brakes iteration, to an apogee of 10,352 ft, successfully achieving our apogee target goal. 
This year, CUR returns with the improvements detailed below with the intention of again improving our apogee accuracy through further control system development.

The complete Air Brakes system includes a mechanical assembly engineered for fail-safe, symmetric brake flap deployment, a refined electronics hardware design, 
and an improved control algorithm responsible for state estimation of the rocket, apogee prediction, and continuously variable PI-controller generating actuation commands. 
Throughout the development cycle, the team focused on building off of prior successes and expanding novel concepts, while ensuring Air Brakes safety and working 
within manufacturability and integration constraints.

## Mechanical Design
The Air Brakes mechanical design source files are not included in this repository.

The design features four symmetrically arranged brake flaps whcih are deployed radially into the airstream surrounding the rocket body. 
The current revision has a total wetted area of 10.18 in², resulting in a near doubling of the total drag force on the rocket to a maximum of 222.6 lbf if deployed after burnout.
The brake flaps ride along linear rails to bear the aerodynamic load and are deployed by a high-speed, high-torque, and dust-rated servo motor, which rotates a central hub connected by linkages to the flaps. 
The chosen servo was tested under load to fully deploy in only 0.3 seconds, and has been upgraded since last year with a feedback signal allowing the microcontroller to measure true deployment as well as detect and mitigate brake flap binding. 
The system continues to integrate compactly within the rocket airframe, adding only 1.75 in to the rocket’s overall length while featuring an increased upper coupler bonded length of 2 in.

Prior revisions of the Air Brakes module have been flown at the Spaceport America Cup and Midland, TX Interneational Rocket Engineering Competition (IREC) events.

## Control System PCB
KiCad 9 source files for the control system PCB are included within the `pcb/` directory. A hierarchical structure is used for schematic diagrams, and schematic symbols as well as PCB footprints
created by the team or downloaded from the manufacturer are included within the `pcb/kicad-library` submodule, which may need to be synced as part of the setup process.

The air brakes electronics feature a four layer, single sided control system PCB with the following considerations taken to achieve our safety critical design constraints while implementing an STM32H730 MCU to perform state estimation and control system tasks, handle user interface, and command the brake flap actuation servo.
The PCB form factor has changed since the revision flown at IREC 2025: the 18650 battery holders have been removed in favor of an externally connected battery, allowing the board size to be halved and battery located beside the PCB.

Power for the system is supplied by a 7.4V (2S) 3000 mAh lithium-ion battery connected to the board by an XT30 connector.
The system has a power-on battery life of at least 12 hr based on battery current draw measurements for an onboard shunt resistor-based current monitor IC.
System power status is determined using an on-chip ADC peripheral of the MCU, reading both the current sense IC output and level of a 2.75:1 voltage divider to convert the battery voltage to fit within the ADC's 3.3V range.

This control system PCB includes power distribution to the servo motor, which may be switched on or off by an on-board transistor under software control.
The servo motor is connected with a four pin latching connector.
As an iterative improvement to our prior design with experience from IREC 2025, the servo has been substituted for an otherwise similar model with position feedback on an additional wire carrying an analog voltage signal corresponding to the servo shaft angle, which may be read by the MCU with an ADC peripheral.

The system includes multiple physical controls including an on-off slide switch, an eight-position rotary mode selector switch, and four tactile switches with alternative functionality based on current mode.
Available modes include startup, testing of user interface, servo, flash, sensors, and control system, a launch detect state to be enabled at the pad, and a data offloading state.
The power switch controls the buck converter enable signal, and includes an RC (resistor-capacitor) filter (time constant tau=0.135 s) to prevent switch bouncing from temporarily disabling the buck converter and resetting the system.
In addition, two RGB LEDs and a passive buzzer (allowing for frequency control) are implemented on the PCB to give internally and externally accessible go/no-go feedback.

To act as inputs to the control system, MEMS sensors from STMicroelectronics were chosen for their pre-existing driver compatibility with STM32 MCUs.
Each sensor also has commonality with the SRAD flight computer developed in parallel, allowing electrical schematics, PCB layout, and code to be shared between the systems towards integrating them in the future.
The LPS22DF barometer was chosen for its 24 bit at 200 Hz pressure output capability, and 22,000G shock survivability. A new inertial measurement unit (IMU), the LSM6DSV80X, was selected for its dual 16 bit accelerometer channels: a low-G channel measuring up to 16G and high-G channel measuring up to 80G in addition to including a 4000 dps, 16 bit gyroscope. 
Together with the final sensor, an IIS2MDC 16 bit magnetometer compass IC, a full complement 9-axis IMU can be implemented in software.

The control system PCB retains a USB Type-C connector for power and data transfer, as well as an on-board 128 MBit flash memory, which emulates a USB flash drive when plugged into a computer in data offloading mode.

## State Estimation and Control System

The air brakes control system was prototyped as a subsystem within the rocket plant Simulink model, which was
later ported to C code to run on the air brakes system’s MCU, taking advantage of the device’s double precision FPU
(Floating Point Unit) as well as using ARM-optimized CMSIS-DSP functions for matrix processing. For clarity and
brevity, the control system functionality is described in this report referencing the Simulink model.
Factoring in the orientation of the implemented sensors on the PCB relative to the body frame was crucial for the
interpretation of IMU and magnetometer data. Unit conversions were also implemented in this sensor data pre-processing step using conversion
factors to common SI units.
The state estimation subsystem implements Madgwick filter sensor fusion on IMU and magnetometer data to
estimate a body to inertial quaternion rotation. The magnetometer assists in estimating the orientation of the rocket by
recording magnetic flux density to the end of determining the magnetic north direction, though it contributes to the
Madgwick filter at a slower update rate through a separate function due to its limited ODR (Output Data Rate). This
quaternion is normalized to a unit vector, and a vector rotation is then performed, where the rotated vector represents 
estimated acceleration in the inertial frame, a necessary input to the next stage of the
control system: the Kalman filter. A two-state Kalman filter was implemented to estimate the rocket’s current altitude
and vertical velocity, and in a similar way to the Madgwick filter, the Kalman filter is implemented as two separate
functions to allow the prediction step—based on high-ODR accelerometer data—to be decoupled from the correction
step which relies on the slower barometer data updates.
The final component of the state estimation subsystem, a launch detect function, was incorporated to identify the
beginning of flight, and differentiate between unintentional movement of the rocket and actual launch. If the rocket
experiences a significant acceleration in the body +X direction and increase in barometric altitude, launch detect is
triggered. When launch detect is triggered, the time since system power on is stored as the launch time, and the mission
elapsed time afterwards is calculated by subtracting the launch time from the time since power on.
The air brakes control system relies on the data from the Kalman filter to predict the rocket’s eventual apogee
using a simplified lightweight model for a projectile with drag, which is used (when the target apogee is subtracted) as the
error input to the controller.
The air brakes deployment controller follows a typical PID (Proportional-Integral-Derivative) structure, but with the
derivative component removed (i.e. gain set to zero) to avoid over correction or oscillation that may make achieving
the target apogee impossible. Instead, small Kp and Ki gains are used to drive the servo motor to decrease the error
function, which is calculated as the difference between the predicted apogee and 3048 m target apogee. The integral
term includes both anti-windup protection, limiting its contribution to deployment to a maximum of 50% and also
resetting the integral term any time the predicted apogee is below the target apogee.
An air brakes lockout subsystem places checks on the controller’s output to the brake actuation servo, preventing
deployment or retracting the brakes if the rocket is still in the powered ascent phase of flight, if the vertical
velocity is slow and the air brakes are ineffective, or if the rocket’s attitude exceeds 30◦ from launch elevation.
