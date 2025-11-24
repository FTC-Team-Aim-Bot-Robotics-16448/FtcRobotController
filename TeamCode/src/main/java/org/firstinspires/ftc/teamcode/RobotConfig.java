package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
public class RobotConfig {
    // Wheels config
    public static String frontLeftWheelName = "lf";
    public static DcMotor.Direction frontLeftWheelDirection = DcMotor.Direction.REVERSE;
    public static String frontRightWheelName = "rf";
    public static DcMotor.Direction frontRightWheelDirection = DcMotor.Direction.FORWARD;
    public static String backLeftWheelName = "lr";
    public static DcMotor.Direction backLeftWheelDirection = DcMotor.Direction.REVERSE;
    public static String backRightWheelName = "rr";
    public static DcMotor.Direction backRightWheelDirection = DcMotor.Direction.FORWARD;
    public static double countPerMotorRev = 537.7;
    public static double driveGearReduction = 1.0;
    public static double wheelDiameterInches = 4.0;
    public static boolean usePetroPathingManualDrive = true;

    // Shooter
    public static double shooterMotorVelocity = 1100;
    public static double shooterMotorVelocityPer = 0.98;
    public static double shooterMotorDecompressionPer = 0.9;

    // IMU config
    public static String imuName = "imu";
    public static RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    // camera
    public static boolean cameraEnabled = true;
    public static String cameraName = "limelight";
    public static double cameraHeight = 323.85;
    public static double cameraAngel = 10;
    public static double targetHeight = 63.5;
    public static int limeLightDetectBallPipeLine = 8;
    public static double cameraToCentorStrafe = -80; // on left of center
    public static double cameraToCentorForward = 120; // on left of center
    public static double intakeToCenterForward = 240; //  forward of center

    public static int limeLightGoalAirTaglPipeLine = 0;
    public static double goalAirTagHeight =  749.3; // mm
    public static double goalAirTagX = 1387; // mm
    public static double goalAirTagY = 2112; // mm

    // Limelight 3A lens parameters
    public static double cameraHorizontalFOV = 54.5; // degrees
    public static double cameraVerticalFOV = 42.0; // degrees

    // Ball detection filters
    public static double ballDiameter = 127; // mm, actual ball diameter
    public static double ballTaTolerance = 0.3; // tolerance ratio for ta validation (±30%)
    public static double maxBallDetectionX = 1500; // mm, max absolute strafe offset
    public static double maxBallDetectionY = 2000; // mm, max absolute forward offset
}
