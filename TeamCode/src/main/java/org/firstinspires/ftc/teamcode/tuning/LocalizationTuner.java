package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.Arrays;
import java.util.List;

/**
 * Comprehensive localization tuner for Pedro Pathing
 * Use this to verify your odometry is working correctly
 */
@TeleOp(name = "Localization Tuner", group = "Tuning")
public class LocalizationTuner extends OpMode {
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    private Telemetry telemetryA;
    
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private List<DcMotorEx> motors;
    
    @Override
    public void init() {
        // Set constants
        Constants.setConstants(FConstants.class, LConstants.class);
        
        // Initialize pose updater
        poseUpdater = new PoseUpdater(hardwareMap, FConstants.class, LConstants.class);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        
        // Initialize motors
        initializeMotors();
        
        // Setup telemetry
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("Localization Tuner");
        telemetryA.addLine("Use gamepad to drive and verify odometry tracking");
        telemetryA.addLine("Check FTC Dashboard for visual feedback");
        telemetryA.addLine();
        telemetryA.addLine("Controls:");
        telemetryA.addLine("  Left stick: Forward/Strafe");
        telemetryA.addLine("  Right stick: Turn");
        telemetryA.addLine("  A: Reset pose to origin");
        telemetryA.update();
        
        // Draw initial robot position
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }
    
    private void initializeMotors() {
        // Get motors from hardware map
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        
        // Set directions to match your robot
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);
        
        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);
        
        // Set zero power behavior
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    
    @Override
    public void loop() {
        // Update pose
        poseUpdater.update();
        dashboardPoseTracker.update();
        
        // Reset pose if A button is pressed
        if (gamepad1.a) {
            poseUpdater.resetPose();
        }
        
        // Handle driving
        handleDriving();
        
        // Update telemetry
        updateTelemetry();
        
        // Update dashboard
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }
    
    private void handleDriving() {
        double y = -gamepad1.left_stick_y; // Forward/backward
        double x = gamepad1.left_stick_x;  // Strafe left/right
        double rx = gamepad1.right_stick_x; // Turn
        
        // Apply deadband
        y = applyDeadband(y, 0.1);
        x = applyDeadband(x, 0.1);
        rx = applyDeadband(rx, 0.1);
        
        // Calculate motor powers for mecanum drive
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftRearPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightRearPower = (y + x - rx) / denominator;
        
        // Apply power scaling
        double powerScale = gamepad1.left_bumper ? 0.3 : 0.7; // Slow mode with left bumper
        
        leftFront.setPower(leftFrontPower * powerScale);
        leftRear.setPower(leftRearPower * powerScale);
        rightFront.setPower(rightFrontPower * powerScale);
        rightRear.setPower(rightRearPower * powerScale);
    }
    
    private double applyDeadband(double value, double deadband) {
        return Math.abs(value) < deadband ? 0 : value;
    }
    
    private void updateTelemetry() {
        telemetryA.addData("X Position", "%.2f inches", poseUpdater.getPose().getX());
        telemetryA.addData("Y Position", "%.2f inches", poseUpdater.getPose().getY());
        telemetryA.addData("Heading", "%.2f degrees", Math.toDegrees(poseUpdater.getPose().getHeading()));
        telemetryA.addData("Total Heading", "%.2f degrees", Math.toDegrees(poseUpdater.getTotalHeading()));
        telemetryA.addLine();
        
        telemetryA.addData("Velocity X", "%.2f in/s", poseUpdater.getVelocity().getX());
        telemetryA.addData("Velocity Y", "%.2f in/s", poseUpdater.getVelocity().getY());
        telemetryA.addLine();
        
        telemetryA.addLine("Drive the robot around to test odometry");
        telemetryA.addLine("Press A to reset pose to origin");
        telemetryA.addLine("Use left bumper for slow mode");
        
        telemetryA.update();
    }
}