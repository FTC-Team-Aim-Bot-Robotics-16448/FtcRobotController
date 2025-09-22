package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * Odometry tuning tool for calibrating encoder multipliers
 * Use this to ensure accurate distance and angle measurements
 */
@Config
@TeleOp(name = "Odometry Tuner", group = "Tuning")
public class OdometryTuner extends OpMode {
    
    // Tunable parameters
    public static double TARGET_DISTANCE = 48.0; // inches
    public static double TARGET_ANGLE = 360.0; // degrees
    
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    private Telemetry telemetryA;
    
    private double startX, startY, startHeading;
    private boolean calibrationStarted = false;
    
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        poseUpdater = new PoseUpdater(hardwareMap, FConstants.class, LConstants.class);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        
        telemetryA.addLine("Odometry Tuner");
        telemetryA.addLine();
        telemetryA.addLine("Instructions:");
        telemetryA.addLine("1. Press A to start calibration");
        telemetryA.addLine("2. Push robot exactly " + TARGET_DISTANCE + " inches forward");
        telemetryA.addLine("3. Check the forward multiplier");
        telemetryA.addLine("4. Press B to reset for lateral test");
        telemetryA.addLine("5. Push robot exactly " + TARGET_DISTANCE + " inches right");
        telemetryA.addLine("6. Check the lateral multiplier");
        telemetryA.addLine("7. Press Y to reset for turn test");
        telemetryA.addLine("8. Turn robot exactly " + TARGET_ANGLE + " degrees");
        telemetryA.addLine("9. Check the turn multiplier");
        telemetryA.update();
        
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }
    
    @Override
    public void loop() {
        poseUpdater.update();
        dashboardPoseTracker.update();
        
        // Handle button presses
        if (gamepad1.a && !calibrationStarted) {
            startCalibration();
        }
        
        if (gamepad1.b) {
            resetForLateralTest();
        }
        
        if (gamepad1.y) {
            resetForTurnTest();
        }
        
        if (gamepad1.x) {
            resetAll();
        }
        
        // Update telemetry
        updateTelemetry();
        
        // Update dashboard
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }
    
    private void startCalibration() {
        startX = poseUpdater.getPose().getX();
        startY = poseUpdater.getPose().getY();
        startHeading = poseUpdater.getTotalHeading();
        calibrationStarted = true;
    }
    
    private void resetForLateralTest() {
        poseUpdater.resetPose();
        startX = 0;
        startY = 0;
        startHeading = 0;
        calibrationStarted = true;
    }
    
    private void resetForTurnTest() {
        poseUpdater.resetPose();
        startX = 0;
        startY = 0;
        startHeading = 0;
        calibrationStarted = true;
    }
    
    private void resetAll() {
        poseUpdater.resetPose();
        calibrationStarted = false;
    }
    
    private void updateTelemetry() {
        telemetryA.addData("Calibration Started", calibrationStarted);
        telemetryA.addLine();
        
        telemetryA.addData("Current X", "%.2f inches", poseUpdater.getPose().getX());
        telemetryA.addData("Current Y", "%.2f inches", poseUpdater.getPose().getY());
        telemetryA.addData("Current Heading", "%.2f degrees", Math.toDegrees(poseUpdater.getPose().getHeading()));
        telemetryA.addData("Total Heading", "%.2f degrees", Math.toDegrees(poseUpdater.getTotalHeading()));
        telemetryA.addLine();
        
        if (calibrationStarted) {
            double deltaX = poseUpdater.getPose().getX() - startX;
            double deltaY = poseUpdater.getPose().getY() - startY;
            double deltaHeading = Math.toDegrees(poseUpdater.getTotalHeading() - startHeading);
            
            telemetryA.addData("Distance Moved X", "%.2f inches", deltaX);
            telemetryA.addData("Distance Moved Y", "%.2f inches", deltaY);
            telemetryA.addData("Angle Turned", "%.2f degrees", deltaHeading);
            telemetryA.addLine();
            
            // Calculate multipliers
            if (Math.abs(deltaX) > 0.1) {
                double forwardMultiplier = TARGET_DISTANCE / (deltaX / poseUpdater.getLocalizer().getForwardMultiplier());
                telemetryA.addData("Forward Multiplier", "%.6f", forwardMultiplier);
            }
            
            if (Math.abs(deltaY) > 0.1) {
                double lateralMultiplier = TARGET_DISTANCE / (deltaY / poseUpdater.getLocalizer().getLateralMultiplier());
                telemetryA.addData("Lateral Multiplier", "%.6f", lateralMultiplier);
            }
            
            if (Math.abs(deltaHeading) > 1.0) {
                double turnMultiplier = Math.toRadians(TARGET_ANGLE) / (poseUpdater.getTotalHeading() / poseUpdater.getLocalizer().getTurningMultiplier());
                telemetryA.addData("Turn Multiplier", "%.6f", turnMultiplier);
            }
        }
        
        telemetryA.addLine();
        telemetryA.addLine("Controls:");
        telemetryA.addLine("  A: Start calibration");
        telemetryA.addLine("  B: Reset for lateral test");
        telemetryA.addLine("  Y: Reset for turn test");
        telemetryA.addLine("  X: Reset all");
        
        telemetryA.update();
    }
}