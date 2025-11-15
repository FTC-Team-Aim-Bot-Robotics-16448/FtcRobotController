package org.firstinspires.ftc.teamcode.actions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.aim.Vision;
import org.firstinspires.ftc.teamcode.aim.action.*;
import org.firstinspires.ftc.teamcode.aim.drive.PIDControl;
import org.firstinspires.ftc.teamcode.aim.drive.PIDControlParams;
import org.firstinspires.ftc.teamcode.Robot;

public class AprilTagTrackingAction extends Action {
    private final Robot robot;
    private int status = 0;
    private int pipeline = 0;
    private double airTagHeight;
    private PIDControl pidController;
    private boolean shouldStop = false;
    private double lastTx = 0;
    private double lastTy = 0;
    private double lastPower = 0;
    private double turretPos = 0;
    private boolean aimed = false;

    public AprilTagTrackingAction(Robot robot, int limelightPipeLine, double airTagHeight) {
        super("AprilTagTracking");
        this.robot = robot;
        this.pipeline = limelightPipeLine;
        this.airTagHeight = airTagHeight;

        // Initialize PID controller for turn control
        // The tx value from limelight is the error - we want it to be 0
        PIDControlParams pidParams = new PIDControlParams();
        pidParams.gain = 0.02;              // Proportional gain - adjust based on robot response
        pidParams.ki = 0.00;               // Integral gain - helps eliminate steady-state error
        pidParams.accelLimit = 2.0;         // Acceleration limit
        pidParams.outputLimit = 0.2;        // Max turn power
        pidParams.tolerance = 1;          // Within 1 degree is considered on target
        pidParams.deadband = 0.25;           // Don't move if error is less than xxx
        pidParams.circular = false;         // Not circular control (tx is linear)
        pidParams.minNonZeroOutput = 0;  // Minimum power to overcome friction

        this.pidController = new PIDControl(pidParams);
        this.pidController.reset(0.0); // Target is 0 (centered on AprilTag)
    }

    public String toString() {
        String statusName;
        switch (this.status) {
            case 0: statusName = "initializing"; break;
            case 1: statusName = "tracking"; break;
            case 2: statusName = "stopped"; break;
            default: statusName = "unknown"; break;
        }

        String result = super.toString() +
                String.format(" [state=%s, tx=%.2f, power=%.2f, pos=%f, aimed=%b, shouldStop=%b]",
                        statusName, this.lastTx, this.lastPower,
                        this.turretPos, this.aimed, this.shouldStop);

        return result;
    }

    public double getTy() {
        return this.lastTy;
    }

    @Override
    public boolean run() {
        switch (this.status) {
            case 0: // Initialize vision system
                //robot.enableManualDrive();
                this.robot.vision.startObjectDetection(
                        this.pipeline,
                        RobotConfig.cameraHeight,
                        RobotConfig.cameraAngel,
                        this.airTagHeight
                );
                this.status = 1;
                this.markStarted();
                break;

            case 1: // Continuously track the AprilTag
                // Get the horizontal offset (tx) from the limelight
                // tx > 0 means target is to the right
                // tx < 0 means target is to the left
                Vision.ObjectDetectionResult detectionRet = this.robot.vision.getObjectDetectionResult();
                if (detectionRet == null) {
                    this.robot.turnTurret(0);
                    return false;
                }
                double tx =  detectionRet.tx;
                this.lastTx = tx;
                this.lastTy = detectionRet.ty;

                // Use PID controller to calculate turn power
                double turnPower = this.pidController.getOutput(this.lastTx);
                this.lastPower = turnPower;
                this.turretPos = this.robot.turretMotor.getCurrentPosition();

                // Aimed at the AprilTag
                if (this.pidController.inPosition()) {
                    this.robot.turretMotor.setPower(0);
                    this.aimed = true;
                    return false;
                }

                this.robot.turnTurret(turnPower);
/*
                // Limit the turret turning angle
                if ((this.turretPos < -200 && turnPower < 0) ||
                        (this.turretPos > 200 && turnPower > 0)) {
                    this.robot.turretMotor.setPower(0);
                    return false;
                }

                this.robot.turretMotor.setPower(turnPower);

 */
                // We can also turn the robot to aim the AprilTag,
                //this.robot.follower.setTeleOpDrive(0, 0, turnPower, true);

                break;

            case 2: // Stopped
                return true;
        }

        return false;
    }

    @Override
    protected void cleanup() {
        // Stop the robot from turning
        //this.robot.follower.setTeleOpDrive(0, 0, 0, true);
        // Stop vision processing
        this.robot.vision.stop();
        this.robot.turretMotor.setPower(0);

        /*this.robot.turretMotor.setPower(0);
        this.robot.turretMotor.setTargetPosition(0);
        this.robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (this.robot.turretMotor.getCurrentPosition() > 0) {
            this.robot.turretMotor.setPower(-0.5);
        } else {
            this.robot.turretMotor.setPower(0.5);
        }*/

    }

    public boolean aprilTagAimed() {
        return this.aimed;
    }
}
