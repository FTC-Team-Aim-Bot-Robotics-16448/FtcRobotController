package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Example autonomous OpMode demonstrating Pedro Pathing usage
 * This creates a simple path that moves the robot in a square pattern
 */
@Autonomous(name = "Example Pedro Autonomous", group = "Pedro")
public class ExampleAutonomous extends AutonomousBase {
    
    // Define specific poses for this autonomous
    private final Pose squarePoint1 = new Pose(24, 0, Math.toRadians(0));
    private final Pose squarePoint2 = new Pose(24, 24, Math.toRadians(90));
    private final Pose squarePoint3 = new Pose(0, 24, Math.toRadians(180));
    private final Pose squarePoint4 = new Pose(0, 0, Math.toRadians(270));
    
    // Paths for the square pattern
    private Path path1, path2, path3, path4;
    
    @Override
    protected void initializeSubsystems() {
        // Initialize any additional subsystems here
        // For this example, we only need the drive subsystem
    }
    
    @Override
    protected void buildPaths() {
        // Build paths for a square pattern
        path1 = new Path(new BezierLine(new Point(startPose), new Point(squarePoint1)));
        path1.setLinearHeadingInterpolation(startPose.getHeading(), squarePoint1.getHeading());
        
        path2 = new Path(new BezierLine(new Point(squarePoint1), new Point(squarePoint2)));
        path2.setLinearHeadingInterpolation(squarePoint1.getHeading(), squarePoint2.getHeading());
        
        path3 = new Path(new BezierLine(new Point(squarePoint2), new Point(squarePoint3)));
        path3.setLinearHeadingInterpolation(squarePoint2.getHeading(), squarePoint3.getHeading());
        
        path4 = new Path(new BezierLine(new Point(squarePoint3), new Point(squarePoint4)));
        path4.setLinearHeadingInterpolation(squarePoint3.getHeading(), squarePoint4.getHeading());
    }
    
    @Override
    protected void updateAutonomous() {
        switch (pathState) {
            case 0:
                // Start first path
                follower.followPath(path1, true);
                setPathState(1);
                break;
                
            case 1:
                // Wait for first path to complete
                if (!follower.isBusy()) {
                    telemetry.addLine("Completed path 1, starting path 2");
                    follower.followPath(path2, true);
                    setPathState(2);
                }
                break;
                
            case 2:
                // Wait for second path to complete
                if (!follower.isBusy()) {
                    telemetry.addLine("Completed path 2, starting path 3");
                    follower.followPath(path3, true);
                    setPathState(3);
                }
                break;
                
            case 3:
                // Wait for third path to complete
                if (!follower.isBusy()) {
                    telemetry.addLine("Completed path 3, starting path 4");
                    follower.followPath(path4, true);
                    setPathState(4);
                }
                break;
                
            case 4:
                // Wait for fourth path to complete
                if (!follower.isBusy()) {
                    telemetry.addLine("Completed square pattern!");
                    isFinished = true;
                }
                break;
        }
    }
    
    @Override
    protected void addCustomTelemetry() {
        telemetry.addLine("Running Example Square Autonomous");
        telemetry.addData("Current Target", getCurrentTarget());
    }
    
    private String getCurrentTarget() {
        switch (pathState) {
            case 0:
            case 1:
                return "Point 1 (24, 0)";
            case 2:
                return "Point 2 (24, 24)";
            case 3:
                return "Point 3 (0, 24)";
            case 4:
                return "Point 4 (0, 0)";
            default:
                return "Finished";
        }
    }
}