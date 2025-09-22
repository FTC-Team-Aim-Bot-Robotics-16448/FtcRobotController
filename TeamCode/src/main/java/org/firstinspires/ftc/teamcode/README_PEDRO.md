# Pedro Pathing Integration Guide

This setup provides a complete Pedro Pathing integration for your FTC robot with odometry support.

## What's Included

### Core Components
- **DriveSubsystem**: Main drive control with Pedro Pathing integration
- **AutonomousBase**: Base class for creating autonomous OpModes
- **EnhancedTeleOp**: Advanced teleop with field-centric driving and pose tracking

### Example OpModes
- **ExampleAutonomous**: Simple square pattern autonomous
- **SimpleAutonomous**: Basic autonomous template
- **EnhancedTeleOp**: Feature-rich teleop mode

### Tuning Tools
- **LocalizationTuner**: Test and verify odometry accuracy
- **PIDTuner**: Tune PIDF coefficients for path following
- **OdometryTuner**: Calibrate encoder multipliers

## Hardware Requirements

### Required Hardware
1. **Pinpoint Odometry Computer** - configured as "pinpoint" in robot config
2. **4 Drive Motors** - configured as:
   - "frontLeftMotor"
   - "frontRightMotor" 
   - "backLeftMotor"
   - "backRightMotor"
3. **Two Odometry Pods** - connected to the Pinpoint device

### Hardware Configuration
1. Connect your Pinpoint device to an I2C port
2. Connect odometry pods to the Pinpoint device
3. Configure motor directions in `FConstants.java`
4. Set odometry pod offsets in `LConstants.java`

## Setup Instructions

### 1. Configure Constants
Edit the constants in `FConstants.java` and `LConstants.java`:

**FConstants.java:**
- Motor names and directions
- PIDF coefficients
- Robot physical properties

**LConstants.java:**
- Pinpoint device configuration
- Odometry pod offsets
- Encoder directions

### 2. Tune Your Robot

**Step 1: Test Motor Directions**
Run the existing "Motor Directions" tuner to verify motor directions.

**Step 2: Calibrate Odometry**
1. Run "Odometry Tuner"
2. Follow the on-screen instructions to calibrate:
   - Forward movement multiplier
   - Lateral movement multiplier  
   - Turn multiplier
3. Update the multipliers in `LConstants.java`

**Step 3: Test Localization**
1. Run "Localization Tuner"
2. Drive the robot around manually
3. Verify position tracking is accurate

**Step 4: Tune PIDF**
1. Run "PID Tuner"
2. Adjust PIDF coefficients in FTC Dashboard
3. Update values in `FConstants.java`

### 3. Use in Your OpModes

**For Autonomous:**
```java
public class MyAutonomous extends AutonomousBase {
    @Override
    protected void buildPaths() {
        // Build your paths here
    }
    
    @Override
    protected void updateAutonomous() {
        // Implement your state machine here
    }
}
```

**For TeleOp:**
```java
DriveSubsystem drive = new DriveSubsystem();
drive.init(hardwareMap);
drive.startTeleop();

// In your loop:
drive.update(gamepad1, gamepad2);
```

## Key Features

### Drive Subsystem Features
- **Field-centric driving**: Robot moves relative to field, not robot orientation
- **Robot-centric driving**: Traditional robot-relative movement
- **Slow mode**: Reduced power for precise movements
- **Pose tracking**: Always know where your robot is
- **Path following**: Autonomous path execution

### TeleOp Controls
- **Left stick**: Forward/backward and strafe
- **Right stick**: Turn left/right
- **Y button**: Toggle field-centric mode
- **X button**: Reset heading to 0
- **Back button**: Reset position to origin
- **Left bumper**: Slow mode

### Autonomous Features
- **State machine**: Easy to follow autonomous logic
- **Path building**: Simple methods to create paths
- **Pose utilities**: Check if robot is near target positions
- **Timer utilities**: Wait for specific time periods

## Troubleshooting

### Common Issues

**Robot doesn't move correctly:**
- Check motor directions in `FConstants.java`
- Verify motor names match your robot configuration

**Odometry tracking is inaccurate:**
- Run the odometry tuner and update multipliers
- Check odometry pod wiring and mounting
- Verify encoder directions in `LConstants.java`

**Path following is poor:**
- Tune PIDF coefficients using the PID tuner
- Check that your robot's physical properties are correct
- Verify zero power acceleration values

**Robot drifts during autonomous:**
- Calibrate your IMU properly
- Check odometry pod mounting for slippage
- Tune heading PIDF coefficients

### Getting Help
- Use FTC Dashboard (192.168.43.1:8080/dash) for real-time tuning
- Check the Pedro Pathing documentation
- Run the included tuning OpModes to diagnose issues

## Next Steps
1. Run through all tuning OpModes
2. Create your own autonomous using `AutonomousBase`
3. Customize the `DriveSubsystem` for your robot's specific needs
4. Add additional subsystems as needed

This setup provides a solid foundation for competitive FTC programming with accurate odometry and smooth path following!