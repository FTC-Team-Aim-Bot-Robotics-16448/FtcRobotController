package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.aim.Vision;
import org.firstinspires.ftc.teamcode.aim.components.Button;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class Robot {
    private DriveSubsystem driveSubsystem;
    private LinearOpMode opMode;
    private Button botRotateButton = new Button();

    public Vision vision = new Vision();

    private void initCamera() {
        vision.init(this.opMode.hardwareMap, this.opMode.telemetry, 9,
                RobotConfig.cameraName, RobotConfig.cameraHeight, RobotConfig.cameraAngel,
                RobotConfig.targetHeight);
    }

    public void init(LinearOpMode opMode) {
        this.opMode = opMode;
        
        this.driveSubsystem = new DriveSubsystem();
        this.driveSubsystem.init(opMode.hardwareMap);
        if (RobotConfig.cameraEnabled) {
            this.initCamera();
        }
    }

    public void start() {
        if (RobotConfig.cameraEnabled) {
            this.vision.start();
        }
        this.driveSubsystem.startTeleop();
    }

    public void handleRobotMove() {
        // Robot movement is now handled by the DriveSubsystem
        // This method can be removed or used for additional robot-specific logic
    }

    public void update() {
        // Update drive subsystem
        this.driveSubsystem.update(this.opMode.gamepad1, this.opMode.gamepad2);
        
        if (RobotConfig.cameraEnabled) {
            vision.update();
        }
    }
    
    /**
     * Get the drive subsystem for advanced control
     * @return DriveSubsystem instance
     */
    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }
}
