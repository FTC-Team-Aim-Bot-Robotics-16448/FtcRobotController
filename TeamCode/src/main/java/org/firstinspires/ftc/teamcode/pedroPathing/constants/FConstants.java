package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        // Motor configuration - adjust these to match your robot's hardware configuration
        FollowerConstants.leftFrontMotorName = "frontLeftMotor";
        FollowerConstants.leftRearMotorName = "backLeftMotor";
        FollowerConstants.rightFrontMotorName = "frontRightMotor";
        FollowerConstants.rightRearMotorName = "backRightMotor";

        // Motor directions - adjust these based on your robot's wiring
        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        // Robot physical properties
        FollowerConstants.mass = 13.35; // Robot mass in kg

        // Movement characteristics (tune these with velocity tuners)
        FollowerConstants.xMovement = 52.1043; // Forward movement capability
        FollowerConstants.yMovement = 40.9187; // Lateral movement capability

        // Zero power acceleration (tune these with acceleration tuners)
        FollowerConstants.forwardZeroPowerAcceleration = -44.703; // Forward deceleration when power is cut
        FollowerConstants.lateralZeroPowerAcceleration = -53.3467; // Lateral deceleration when power is cut

        // Translational PIDF coefficients (for following paths)
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1, 0, 0.01, 0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1, 0, 0.01, 0);

        // Heading PIDF coefficients (for maintaining orientation)
        FollowerConstants.headingPIDFCoefficients.setCoefficients(2, 0, 0.1, 0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2, 0, 0.1, 0);

        // Drive PIDF coefficients (for motor control)
        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01, 0, 0.0001, 0.6, 0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1, 0, 0, 0.6, 0);

        // Advanced tuning parameters
        FollowerConstants.zeroPowerAccelerationMultiplier = 4; // Multiplier for zero power acceleration
        FollowerConstants.centripetalScaling = 0.0005; // Scaling for centripetal force correction

        // Path end constraints
        FollowerConstants.pathEndTimeoutConstraint = 50;
        FollowerConstants.pathEndTValueConstraint = 0.995; // How close to path end before considering complete
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
