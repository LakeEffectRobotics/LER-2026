# Feature Implementation Walkthrough: Snake Mode 

## Goal
Implement a features similar to the Wildstang "watch where you are going" / "snake" mode, where the swerve robot automatically rotates to face its direction of travel. This is useful for efficiently harvesting balls in both Teleop and Auto modes.

## Changes Made

1. **Created `SnakeDriveCommand`**
   - Added a new command file at `src/main/java/frc/robot/commands/SnakeDriveCommand.java`.
   - This command takes the raw x and y translation stick inputs (or Auto inputs) and translates them into a velocity vector.
   - It computes the mathematical direction of travel using `Math.atan2(leftVel, forwardVel)` to determine the proper heading.
   - Using a continuous `PIDController` enabled from `-PI` to `PI`, it computes the necessary rotational output (`omega`) to automatically rotate the robot's front side (0 degrees) to the desired travel heading angle.
   - If the robot isn't translating heavily (to avoid jittering while stationary), it does not forcefully correct the heading.

2. **Added Teleop Binding**
   - Updated `src/main/java/frc/robot/RobotContainer.java` to map the `SnakeDriveCommand` to the Xbox **Y** button on the driver's controller (`OI.driveControllerY`).
   - By holding the **Y** button, the driver can enter Snake Mode ad-hoc, translating in any direction while the robot handles facing that direction automatically.

## Integration in Auto
Since `SnakeDriveCommand` accepts generic `DoubleSupplier`s for x and y translation, it can be seamlessly used in any autonomous routine by supplying the path following values instead of joystick axes. It will act exactly the same, turning the robot automatically.

## Verification & Deployment
- Ran a local `./gradlew build` to ensure that all imports, WPILib classes, and Java syntax are correct. 
- The project successfully compiled without any errors.
- **Action Required**: Please deploy this code to the actual robot (or simulator) and try driving around while holding **Y** on the driver's controller to test out the PID tuning for the heading!
