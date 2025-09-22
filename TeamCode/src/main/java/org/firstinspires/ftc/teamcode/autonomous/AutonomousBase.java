package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Base class for autonomous OpModes
 * Provides common functionality for path following and robot control
 */
public abstract class AutonomousBase extends LinearOpMode {
    protected DriveSubsystem driveSubsystem;
    protected Follower follower;
    protected Timer pathTimer;
    protected Timer actionTimer;
    
    // State management
    protected int pathState = 0;
    protected boolean isFinished = false;
    
    // Common poses - adjust these for your field setup
    protected Pose startPose = new Pose(9, 111, Math.toRadians(270));
    protected Pose scorePose = new Pose(14, 129, Math.toRadians(315));
    protected Pose parkPose = new Pose(60, 98, Math.toRadians(90));
    
    @Override
    public void runOpMode() {
        // Initialize subsystems
        initializeRobot();
        
        // Build paths specific to this autonomous
        buildPaths();
        
        // Wait for start
        telemetry.addLine("Robot initialized. Ready to start!");
        telemetry.addData("Starting pose", "X: %.1f, Y: %.1f, Heading: %.1f°", 
            startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.update();
        
        waitForStart();
        
        if (isStopRequested()) return;
        
        // Start autonomous
        driveSubsystem.startAutonomous();
        pathTimer.resetTimer();
        actionTimer.resetTimer();
        
        // Run autonomous loop
        while (opModeIsActive() && !isFinished) {
            // Update subsystems
            driveSubsystem.update(gamepad1, gamepad2);
            
            // Update autonomous state machine
            updateAutonomous();
            
            // Update telemetry
            updateTelemetry();
            
            // Small delay to prevent overwhelming the system
            sleep(10);
        }
        
        // Cleanup
        cleanup();
    }
    
    /**
     * Initialize the robot and all subsystems
     */
    protected void initializeRobot() {
        // Initialize timers
        pathTimer = new Timer();
        actionTimer = new Timer();
        
        // Initialize drive subsystem
        driveSubsystem = new DriveSubsystem();
        driveSubsystem.init(hardwareMap);
        
        // Get follower reference
        follower = driveSubsystem.getFollower();
        follower.setStartingPose(startPose);
        
        // Initialize other subsystems (override in child classes)
        initializeSubsystems();
    }
    
    /**
     * Override this method to initialize additional subsystems
     */
    protected abstract void initializeSubsystems();
    
    /**
     * Override this method to build paths specific to your autonomous
     */
    protected abstract void buildPaths();
    
    /**
     * Override this method to implement your autonomous state machine
     */
    protected abstract void updateAutonomous();
    
    /**
     * Update telemetry with current robot state
     */
    protected void updateTelemetry() {
        Pose currentPose = follower.getPose();
        
        telemetry.addData("Path State", pathState);
        telemetry.addData("Robot Pose", "X: %.1f, Y: %.1f, H: %.1f°", 
            currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Path Timer", "%.2f seconds", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Is Busy", follower.isBusy());
        telemetry.addData("Is Finished", isFinished);
        
        // Add custom telemetry (override in child classes)
        addCustomTelemetry();
        
        telemetry.update();
    }
    
    /**
     * Override this method to add custom telemetry
     */
    protected void addCustomTelemetry() {
        // Default implementation - override in child classes
    }
    
    /**
     * Utility method to advance to the next path state
     * @param nextState The next state to transition to
     */
    protected void setPathState(int nextState) {
        pathState = nextState;
        pathTimer.resetTimer();
    }
    
    /**
     * Check if the robot is close to a target pose
     * @param targetPose The target pose to check against
     * @param tolerance Position tolerance in inches
     * @param headingTolerance Heading tolerance in degrees
     * @return true if within tolerance
     */
    protected boolean isNearPose(Pose targetPose, double tolerance, double headingTolerance) {
        Pose currentPose = follower.getPose();
        
        double distanceError = Math.sqrt(
            Math.pow(currentPose.getX() - targetPose.getX(), 2) + 
            Math.pow(currentPose.getY() - targetPose.getY(), 2)
        );
        
        double headingError = Math.abs(Math.toDegrees(currentPose.getHeading() - targetPose.getHeading()));
        while (headingError > 180) headingError -= 360;
        headingError = Math.abs(headingError);
        
        return distanceError < tolerance && headingError < headingTolerance;
    }
    
    /**
     * Wait for a specified amount of time
     * @param seconds Time to wait in seconds
     * @return true if time has elapsed
     */
    protected boolean waitForTime(double seconds) {
        return pathTimer.getElapsedTimeSeconds() >= seconds;
    }
    
    /**
     * Create a simple straight line path
     * @param start Start point
     * @param end End point
     * @return Path object
     */
    protected Path createStraightPath(Point start, Point end) {
        return new Path(new BezierLine(start, end));
    }
    
    /**
     * Create a curved path with one control point
     * @param start Start point
     * @param control Control point for curve
     * @param end End point
     * @return Path object
     */
    protected Path createCurvedPath(Point start, Point control, Point end) {
        return new Path(new BezierCurve(start, control, end));
    }
    
    /**
     * Cleanup method called at the end of autonomous
     */
    protected void cleanup() {
        // Stop all movement
        driveSubsystem.emergencyStop();
        
        // Additional cleanup (override in child classes)
        performCleanup();
    }
    
    /**
     * Override this method for additional cleanup
     */
    protected void performCleanup() {
        // Default implementation - override in child classes
    }
}