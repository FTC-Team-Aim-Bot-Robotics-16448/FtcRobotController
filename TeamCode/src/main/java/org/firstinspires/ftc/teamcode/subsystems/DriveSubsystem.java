package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * Drive subsystem that integrates Pedro Pathing with manual control
 * Handles both autonomous path following and teleop driving
 */
public class DriveSubsystem {
    private Follower follower;
    private boolean isAutonomousMode = false;
    private boolean fieldCentric = true;
    
    // Drive power scaling
    private double drivePowerScale = 0.8;
    private double turnPowerScale = 0.6;
    private double slowModeScale = 0.4;
    
    public DriveSubsystem() {
        // Constructor
    }
    
    /**
     * Initialize the drive subsystem
     * @param hardwareMap Robot hardware map
     */
    public void init(HardwareMap hardwareMap) {
        // Initialize Pedro Pathing follower
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        
        // Set starting pose (adjust as needed for your field position)
        follower.setStartingPose(new Pose(0, 0, 0));
    }
    
    /**
     * Start teleop driving mode
     */
    public void startTeleop() {
        isAutonomousMode = false;
        follower.startTeleopDrive();
    }
    
    /**
     * Start autonomous mode
     */
    public void startAutonomous() {
        isAutonomousMode = true;
    }
    
    /**
     * Update the drive subsystem - call this in your main loop
     * @param gamepad1 Primary gamepad for driving
     * @param gamepad2 Secondary gamepad for additional controls
     */
    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        // Always update the follower for localization
        follower.update();
        
        if (!isAutonomousMode) {
            handleTeleopDriving(gamepad1, gamepad2);
        }
    }
    
    /**
     * Handle teleop driving controls
     */
    private void handleTeleopDriving(Gamepad gamepad1, Gamepad gamepad2) {
        // Toggle field-centric mode with Y button
        if (gamepad1.y) {
            fieldCentric = !fieldCentric;
        }
        
        // Reset heading with X button
        if (gamepad1.x) {
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
        }
        
        // Determine drive power scaling
        double currentDriveScale = drivePowerScale;
        double currentTurnScale = turnPowerScale;
        
        // Slow mode with left bumper
        if (gamepad1.left_bumper) {
            currentDriveScale *= slowModeScale;
            currentTurnScale *= slowModeScale;
        }
        
        // Get joystick inputs
        double forward = -gamepad1.left_stick_y * currentDriveScale;
        double strafe = gamepad1.left_stick_x * currentDriveScale;
        double turn = gamepad1.right_stick_x * currentTurnScale;
        
        // Apply deadband
        forward = applyDeadband(forward, 0.1);
        strafe = applyDeadband(strafe, 0.1);
        turn = applyDeadband(turn, 0.1);
        
        // Set movement vectors
        follower.setTeleOpMovementVectors(forward, strafe, turn, !fieldCentric);
    }
    
    /**
     * Apply deadband to joystick inputs
     */
    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0;
        }
        return value;
    }
    
    /**
     * Follow a path in autonomous mode
     * @param path The path to follow
     * @param holdEnd Whether to hold position at the end
     */
    public void followPath(com.pedropathing.pathgen.Path path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
    }
    
    /**
     * Follow a path chain in autonomous mode
     * @param pathChain The path chain to follow
     * @param holdEnd Whether to hold position at the end
     */
    public void followPath(com.pedropathing.pathgen.PathChain pathChain, boolean holdEnd) {
        follower.followPath(pathChain, holdEnd);
    }
    
    /**
     * Check if the follower is busy following a path
     * @return true if busy, false if finished
     */
    public boolean isBusy() {
        return follower.isBusy();
    }
    
    /**
     * Get the current robot pose
     * @return Current pose (x, y, heading)
     */
    public Pose getPose() {
        return follower.getPose();
    }
    
    /**
     * Set the robot pose (useful for resetting position)
     * @param pose New pose to set
     */
    public void setPose(Pose pose) {
        follower.setPose(pose);
    }
    
    /**
     * Get the follower instance for advanced usage
     * @return The Pedro Pathing follower
     */
    public Follower getFollower() {
        return follower;
    }
    
    /**
     * Set drive power scaling
     * @param scale Power scale (0.0 to 1.0)
     */
    public void setDrivePowerScale(double scale) {
        this.drivePowerScale = Range.clip(scale, 0.0, 1.0);
    }
    
    /**
     * Set turn power scaling
     * @param scale Turn scale (0.0 to 1.0)
     */
    public void setTurnPowerScale(double scale) {
        this.turnPowerScale = Range.clip(scale, 0.0, 1.0);
    }
    
    /**
     * Toggle field-centric driving mode
     */
    public void toggleFieldCentric() {
        fieldCentric = !fieldCentric;
    }
    
    /**
     * Check if in field-centric mode
     * @return true if field-centric, false if robot-centric
     */
    public boolean isFieldCentric() {
        return fieldCentric;
    }
    
    /**
     * Emergency stop - stops all movement
     */
    public void emergencyStop() {
        follower.breakFollowing();
        // Set all motor powers to 0
        follower.setTeleOpMovementVectors(0, 0, 0, false);
    }
}