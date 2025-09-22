package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * PID tuning OpMode for Pedro Pathing
 * Use this to tune your PIDF coefficients for optimal path following
 */
@Config
@Autonomous(name = "PID Tuner", group = "Tuning")
public class PIDTuner extends LinearOpMode {
    
    // Tunable parameters via FTC Dashboard
    public static double DISTANCE = 40;
    public static boolean USE_CURVED_PATH = false;
    public static double CURVE_CONTROL_OFFSET = 20;
    
    private Follower follower;
    private Path forwardPath, backwardPath;
    private Telemetry telemetryA;
    
    private boolean goingForward = true;
    private boolean pathCompleted = false;
    
    @Override
    public void runOpMode() {
        // Initialize telemetry
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Initialize follower
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        
        // Build paths
        buildPaths();
        
        telemetryA.addLine("PID Tuner Ready");
        telemetryA.addLine("Adjust PIDF values in FTC Dashboard");
        telemetryA.addData("Distance", DISTANCE);
        telemetryA.addData("Curved Path", USE_CURVED_PATH);
        telemetryA.update();
        
        waitForStart();
        
        if (isStopRequested()) return;
        
        // Start first path
        follower.followPath(forwardPath, true);
        
        while (opModeIsActive()) {
            // Update follower
            follower.update();
            
            // Check if path is complete and switch direction
            if (!follower.isBusy() && !pathCompleted) {
                pathCompleted = true;
                
                // Wait a moment then start return path
                sleep(500);
                
                if (goingForward) {
                    follower.followPath(backwardPath, true);
                    goingForward = false;
                } else {
                    follower.followPath(forwardPath, true);
                    goingForward = true;
                }
                
                pathCompleted = false;
            }
            
            // Update telemetry
            updateTelemetry();
            
            sleep(20);
        }
    }
    
    private void buildPaths() {
        if (USE_CURVED_PATH) {
            // Create curved paths for more complex tuning
            Point start = new Point(0, 0, Point.CARTESIAN);
            Point control = new Point(DISTANCE / 2, CURVE_CONTROL_OFFSET, Point.CARTESIAN);
            Point end = new Point(DISTANCE, 0, Point.CARTESIAN);
            
            forwardPath = new Path(new com.pedropathing.pathgen.BezierCurve(start, control, end));
            backwardPath = new Path(new com.pedropathing.pathgen.BezierCurve(end, control, start));
        } else {
            // Create straight line paths
            forwardPath = new Path(new BezierLine(
                new Point(0, 0, Point.CARTESIAN), 
                new Point(DISTANCE, 0, Point.CARTESIAN)
            ));
            
            backwardPath = new Path(new BezierLine(
                new Point(DISTANCE, 0, Point.CARTESIAN), 
                new Point(0, 0, Point.CARTESIAN)
            ));
        }
        
        // Set heading interpolation
        forwardPath.setConstantHeadingInterpolation(0);
        backwardPath.setConstantHeadingInterpolation(0);
    }
    
    private void updateTelemetry() {
        telemetryA.addData("Going Forward", goingForward);
        telemetryA.addData("Path Type", USE_CURVED_PATH ? "Curved" : "Straight");
        telemetryA.addData("Distance", DISTANCE);
        telemetryA.addLine();
        
        // Show current pose
        telemetryA.addData("X", "%.2f", follower.getPose().getX());
        telemetryA.addData("Y", "%.2f", follower.getPose().getY());
        telemetryA.addData("Heading", "%.2f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetryA.addLine();
        
        telemetryA.addData("Is Busy", follower.isBusy());
        telemetryA.addLine();
        
        telemetryA.addLine("Tune PIDF values in FTC Dashboard:");
        telemetryA.addLine("  - translationalPIDFCoefficients");
        telemetryA.addLine("  - headingPIDFCoefficients");
        telemetryA.addLine("  - drivePIDFCoefficients");
        
        telemetryA.update();
    }
}