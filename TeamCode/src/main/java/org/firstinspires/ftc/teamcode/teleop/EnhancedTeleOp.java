package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Enhanced TeleOp with Pedro Pathing integration
 * Features field-centric driving, pose tracking, and advanced controls
 */
@TeleOp(name = "Enhanced Pedro TeleOp", group = "Pedro")
public class EnhancedTeleOp extends LinearOpMode {
    
    private DriveSubsystem driveSubsystem;
    private ElapsedTime runtime;
    private ElapsedTime buttonCooldown;
    
    // Control flags
    private boolean lastYButton = false;
    private boolean lastXButton = false;
    private boolean lastBackButton = false;
    
    @Override
    public void runOpMode() {
        // Initialize runtime timer
        runtime = new ElapsedTime();
        buttonCooldown = new ElapsedTime();
        
        // Initialize drive subsystem
        driveSubsystem = new DriveSubsystem();
        driveSubsystem.init(hardwareMap);
        
        // Set initial pose if needed (e.g., from autonomous)
        // driveSubsystem.setPose(new Pose(0, 0, 0));
        
        telemetry.addLine("Enhanced TeleOp Initialized");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Left stick: Forward/Backward and Strafe");
        telemetry.addLine("  Right stick: Turn");
        telemetry.addLine("  Left bumper: Slow mode");
        telemetry.addLine("  Y: Toggle field-centric mode");
        telemetry.addLine("  X: Reset heading");
        telemetry.addLine("  Back: Reset position");
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        
        if (isStopRequested()) return;
        
        // Start teleop mode
        driveSubsystem.startTeleop();
        
        while (opModeIsActive()) {
            // Handle button presses with cooldown
            handleButtonInputs();
            
            // Update drive subsystem
            driveSubsystem.update(gamepad1, gamepad2);
            
            // Update telemetry
            updateTelemetry();
            
            // Small delay to prevent overwhelming the system
            sleep(20);
        }
    }
    
    /**
     * Handle button inputs with proper debouncing
     */
    private void handleButtonInputs() {
        // Only process button presses if cooldown has elapsed
        if (buttonCooldown.milliseconds() < 200) {
            return;
        }
        
        // Toggle field-centric mode
        if (gamepad1.y && !lastYButton) {
            driveSubsystem.toggleFieldCentric();
            buttonCooldown.reset();
        }
        
        // Reset heading
        if (gamepad1.x && !lastXButton) {
            Pose currentPose = driveSubsystem.getPose();
            driveSubsystem.setPose(new Pose(currentPose.getX(), currentPose.getY(), 0));
            buttonCooldown.reset();
        }
        
        // Reset position
        if (gamepad1.back && !lastBackButton) {
            driveSubsystem.setPose(new Pose(0, 0, 0));
            buttonCooldown.reset();
        }
        
        // Update button states
        lastYButton = gamepad1.y;
        lastXButton = gamepad1.x;
        lastBackButton = gamepad1.back;
    }
    
    /**
     * Update telemetry with current robot state
     */
    private void updateTelemetry() {
        Pose currentPose = driveSubsystem.getPose();
        
        telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
        telemetry.addLine();
        
        telemetry.addData("Drive Mode", driveSubsystem.isFieldCentric() ? "Field-Centric" : "Robot-Centric");
        telemetry.addData("Slow Mode", gamepad1.left_bumper ? "ON" : "OFF");
        telemetry.addLine();
        
        telemetry.addData("Robot Position", "X: %.1f, Y: %.1f", currentPose.getX(), currentPose.getY());
        telemetry.addData("Robot Heading", "%.1f degrees", Math.toDegrees(currentPose.getHeading()));
        telemetry.addLine();
        
        telemetry.addData("Left Stick", "X: %.2f, Y: %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.addData("Right Stick", "X: %.2f", gamepad1.right_stick_x);
        telemetry.addLine();
        
        telemetry.addLine("Controls:");
        telemetry.addLine("  Y: Toggle field-centric (" + (driveSubsystem.isFieldCentric() ? "ON" : "OFF") + ")");
        telemetry.addLine("  X: Reset heading");
        telemetry.addLine("  Back: Reset position");
        telemetry.addLine("  Left bumper: Slow mode");
        
        telemetry.update();
    }
}