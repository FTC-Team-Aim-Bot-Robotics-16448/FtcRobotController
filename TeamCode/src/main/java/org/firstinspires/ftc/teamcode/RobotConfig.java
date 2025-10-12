package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
public class RobotConfig {
    // Wheels config
    public static String frontLeftWheelName = "frontLeftMotor";
    public static DcMotor.Direction frontLeftWheelDirection = DcMotor.Direction.FORWARD;
    public static String frontRightWheelName = "frontRightMotor";
    public static DcMotor.Direction frontRightWheelDirection = DcMotor.Direction.REVERSE;
    public static String backLeftWheelName = "backLeftMotor";
    public static DcMotor.Direction backLeftWheelDirection = DcMotor.Direction.FORWARD;
    public static String backRightWheelName = "backRightMotor";
    public static DcMotor.Direction backRightWheelDirection = DcMotor.Direction.FORWARD;
    public static double countPerMotorRev = 537.7;
    public static double driveGearReduction = 1.0;
    public static double wheelDiameterInches = 4.0;

    // IMU config
    public static String imuName = "imu";
    public static RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
    public static RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

    // camera
    public static boolean cameraEnabled = true;
    public static String cameraName = "limelight";
    public static double cameraHeight = 282;
    public static double cameraAngel = 0;
    public static double targetHeight = 63.5;
    public static int limeLightDetectBallPipeLine = 8;
    public static double cameraToCentorStrafe = -80; // on left of center
    public static double cameraToCentorForward = 120; // on left of center
    public static double intakeToCenterForward = 240; //  forward of center

    public static int limeLightGoalAirTaglPipeLine = 0;
    public static double goalAirTagHeight = 63.5;
    public static double goalAirTagX = 10; // inch
    public static double goaAirTagY = 140; // inch

    // Limelight 3A lens parameters
    public static double cameraHorizontalFOV = 54.5; // degrees
    public static double cameraVerticalFOV = 42.0; // degrees

    // Ball detection filters
    public static double ballDiameter = 127; // mm, actual ball diameter
    public static double ballTaTolerance = 0.3; // tolerance ratio for ta validation (±30%)
    public static double maxBallDetectionX = 1500; // mm, max absolute strafe offset
    public static double maxBallDetectionY = 2000; // mm, max absolute forward offset
}
