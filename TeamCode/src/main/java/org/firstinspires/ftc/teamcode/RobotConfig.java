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
}
