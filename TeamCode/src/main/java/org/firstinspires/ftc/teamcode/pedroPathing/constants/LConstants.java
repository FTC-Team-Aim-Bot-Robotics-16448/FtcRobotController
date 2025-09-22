package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {
        // Pinpoint odometry configuration
        // These values represent the offset of the odometry pods from the robot's center
        // Positive forwardY means the forward pod is in front of the center
        // Positive strafeX means the strafe pod is to the right of the center
        PinpointConstants.forwardY = 2.3622; // Forward pod Y offset in inches
        PinpointConstants.strafeX = 6.5945;  // Strafe pod X offset in inches
        
        // Distance unit for measurements
        PinpointConstants.distanceUnit = DistanceUnit.INCH;
        
        // Hardware map name for the Pinpoint device
        PinpointConstants.hardwareMapName = "pinpoint";
        
        // Yaw scaling configuration
        PinpointConstants.useYawScalar = false;
        PinpointConstants.yawScalar = 1.0; // Adjust if IMU heading needs scaling
        
        // Encoder resolution configuration
        PinpointConstants.useCustomEncoderResolution = false;
        // Use standard goBILDA 4-bar pod resolution
        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        PinpointConstants.customEncoderResolution = 13.26291192; // Custom resolution if needed
        
        // Encoder directions - adjust these based on your wiring
        // FORWARD means positive robot movement gives positive encoder readings
        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    }
}