package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.*;
import com.pedropathing.control.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.20)
            .forwardZeroPowerAcceleration(-40.00)
            .lateralZeroPowerAcceleration(-56.93)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.0005)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.12, 0, 0.01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.05, 0))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.01, 0, 0.0001, 0.6, 0.05)
            );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("lf")
            .leftRearMotorName("lr")
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftFrontMotorDirection(DcMotor.Direction.REVERSE)
            .leftRearMotorDirection(DcMotor.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotor.Direction.FORWARD)
            .rightRearMotorDirection(DcMotor.Direction.FORWARD)
            .xVelocity(63.58)
            .yVelocity(57.78);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-1.9685)
            .strafePodX(-5.9055)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
           // .yawScalar(1.0)
            .encoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            )
            //.customEncoderResolution(13.26291192)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.985,
            100,
            1,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}