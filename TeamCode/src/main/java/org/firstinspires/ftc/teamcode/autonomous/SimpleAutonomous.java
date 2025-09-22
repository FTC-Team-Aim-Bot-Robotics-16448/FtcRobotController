package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Simple autonomous that demonstrates basic Pedro Pathing functionality
 * Moves forward, turns, and parks
 */
@Autonomous(name = "Simple Pedro Autonomous", group = "Pedro")
public class SimpleAutonomous extends AutonomousBase {
    
    // Define poses for this autonomous
    private final Pose forwardPose = new Pose(24, 0, Math.toRadians(0));
    private final Pose turnPose = new Pose(24, 0, Math.toRadians(90));
    private final Pose finalPose = new Pose(24, 24, Math.toRadians(90));
    
    // Paths
    private Path moveForward, turn, moveToFinal;
    
    @Override
    protected void initializeSubsystems() {
        // No additional subsystems needed for this simple example
    }
    
    @Override
    protected void buildPaths() {
        // Build simple paths
        moveForward = new Path(new BezierLine(new Point(startPose), new Point(forwardPose)));
        moveForward.setLinearHeadingInterpolation(startPose.getHeading(), forwardPose.getHeading());
        
        turn = new Path(new BezierLine(new Point(forwardPose), new Point(turnPose)));
        turn.setLinearHeadingInterpolation(forwardPose.getHeading(), turnPose.getHeading());
        
        moveToFinal = new Path(new BezierLine(new Point(turnPose), new Point(finalPose)));
        moveToFinal.setLinearHeadingInterpolation(turnPose.getHeading(), finalPose.getHeading());
    }
    
    @Override
    protected void updateAutonomous() {
        switch (pathState) {
            case 0:
                // Start moving forward
                follower.followPath(moveForward, true);
                setPathState(1);
                break;
                
            case 1:
                // Wait for forward movement to complete
                if (!follower.isBusy()) {
                    // Wait a moment, then start turn
                    if (waitForTime(0.5)) {
                        follower.followPath(turn, true);
                        setPathState(2);
                    }
                }
                break;
                
            case 2:
                // Wait for turn to complete
                if (!follower.isBusy()) {
                    // Wait a moment, then move to final position
                    if (waitForTime(0.5)) {
                        follower.followPath(moveToFinal, true);
                        setPathState(3);
                    }
                }
                break;
                
            case 3:
                // Wait for final movement to complete
                if (!follower.isBusy()) {
                    if (waitForTime(1.0)) {
                        isFinished = true;
                    }
                }
                break;
        }
    }
    
    @Override
    protected void addCustomTelemetry() {
        telemetry.addLine("Running Simple Autonomous");
        
        String currentAction = "Unknown";
        switch (pathState) {
            case 0:
            case 1:
                currentAction = "Moving Forward";
                break;
            case 2:
                currentAction = "Turning";
                break;
            case 3:
                currentAction = "Moving to Final Position";
                break;
            default:
                currentAction = "Finished";
                break;
        }
        
        telemetry.addData("Current Action", currentAction);
    }
}