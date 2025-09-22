package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class RobotConfig {
    // Camera configuration
    public static boolean cameraEnabled = false;
    public static String cameraName = "limelight";
    public static double cameraHeight = 420;
    public static double cameraAngel = -60;
    public static double targetHeight = 35;
    
    // Pedro Pathing and odometry configuration is now handled in FConstants and LConstants
    // Motor names and directions are configured in FConstants
    // Pinpoint odometry settings are configured in LConstants
}
