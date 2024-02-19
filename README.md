# 6672's 2024 Robot Code

## Robot Mechanisms
* **Drivetrain**: A standard swerve drivetrain using 4 SDS MK4 gearboxes with L4 gear ratios - TalonFX (Kraken motors for drive, Falcon 500 for steer), CANCoders, and a Pigeon 2 gyroscope. Our code directly uses the CTRE swerve API. Uses a pose estimator for odometry and PathPlanner for auto routines.
* **Intake**: One NEO Vortex, operates under the bumper. Code has commands for intaking and outtaking.
* **Index**: One NEO Vortex. Contains a beam break sensor for detecting if a note is already contained.
* **Pivot**: Two TalonFX, changes the shooter angle. 
* **Shooter**: Two NEO Vortex motors for launching notes.
* **Cobra** (Climb): One TalonFX. 
* **Cameras**: two Limelight 2+ cameras, one mounted on the shooter side, strictly used for detecting AprilTags and creating 3D pose estimates (used in the drivetrain's pose estimator), and a second mounted on the intake side for detecting notes and aiding driver vision.

## PathPlanner Naming Conventions:
Mobility / # of pieces | Location | Step # | End Loaded (T/F) |
--- | --- | --- | --- |
2S | TOP | 2 | T |
1S | MID | 1 | F |
MOB | BOT | --- | T

* The first example path is the second step of a two piece auto starting at the top of the robot starting zone that ends with a note loaded
* The second example path is the first step of a one piece auto starting at the middle of the robot starting zone that does not end with a note loaded
* The third example path is a simple path that is part of a mobility auto starting at the bottom of the starting zone that only moves out of the starting zone but also loads a note at the end

## NamedCommands:
* RunIntake
* RunIndex (maybe)
* AimAtTarget
* ShootSpeaker
* ShootAmp
