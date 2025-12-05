package org.firstinspires.ftc.teamcode.actions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.aim.action.*;
import org.firstinspires.ftc.teamcode.aim.drive.PIDControl;
import org.firstinspires.ftc.teamcode.aim.drive.PIDControlParams;
import org.firstinspires.ftc.teamcode.Robot;

public class TurretMotorRestAction extends Action {
    private final Robot robot;
    private int status = 0;
    private PIDControl pidController;
    private double lastPower = 0;
    private double currentPosition = 0;
    private double targetPosition = 0;
    public boolean resetEnabled = true;
    private static final double POSITION_TOLERANCE = 50; // Same as PID tolerance

    public TurretMotorRestAction(Robot robot) {
        super("TurretMotorRest");
        this.robot = robot;

        // Initialize PID controller to hold turret motor at position 0
        // Uses the same PID parameters as AprilTagTrackingAction
        PIDControlParams pidParams = new PIDControlParams();
        pidParams.gain = RobotConfig.turretPidGain;              // Proportional gain
        pidParams.ki = 0.00;               // Integral gain
        pidParams.accelLimit = 2.0;         // Acceleration limit
        pidParams.outputLimit = RobotConfig.turretPidMaxPower;        // Max power
        pidParams.tolerance = TurretMotorRestAction.POSITION_TOLERANCE;          // Within 5 encoder ticks is considered at position
        pidParams.deadband = 2;           // Don't move if error is less than 2 ticks
        pidParams.circular = false;         // Not circular control
        pidParams.minNonZeroOutput = RobotConfig.turretMotorMinPower;  // Minimum power to overcome friction

        this.pidController = new PIDControl(pidParams);
        this.pidController.reset(this.targetPosition); // Target is 0
    }

    @Override
    public String toString() {
        String statusName;
        switch (this.status) {
            case 0: statusName = "initializing"; break;
            case 1: statusName = "holding"; break;
            case 2: statusName = "stopped"; break;
            default: statusName = "unknown"; break;
        }

        String result = super.toString() +
                String.format(" [state=%s, pos=%.0f, target=%.0f, power=%.2f, atPos=%b]",
                        statusName, this.currentPosition, this.targetPosition,
                        this.lastPower, this.pidController.inPosition());

        return result;
    }

    @Override
    public boolean run() {
        switch (this.status) {
            case 0: // Initialize
                // Ensure motor is in the correct mode
                if (this.robot.turretMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    this.robot.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                this.status = 1;
                this.markStarted();
                break;

            case 1: // Hold position using PID
                // Get current position
                this.currentPosition = this.robot.turretMotor.getCurrentPosition();

                if (!this.resetEnabled) {
                    // PID is disabled, stop the motor
                    this.robot.turretMotor.setPower(0);
                    break;
                }

                // Calculate power needed to hold position at 0
                double power = this.pidController.getOutput(this.currentPosition);
                this.lastPower = power;

                // Apply power to motor
                this.robot.turretMotor.setPower(power);

                break;

            case 2: // Stopped
                return true;
        }

        return false;
    }

    @Override
    protected void cleanup() {
        // Stop the motor
        this.robot.turretMotor.setPower(0);
    }

    public boolean isAtPosition() {
        // Dynamically check if current position is within tolerance of target (0)
        double error = Math.abs(this.currentPosition - this.targetPosition);
        return error <= POSITION_TOLERANCE;
    }

    public double getCurrentPosition() {
        return this.currentPosition;
    }

    public double getLastPower() {
        return this.lastPower;
    }

    public void enableReset(boolean val) {
        this.resetEnabled = val;
        if (val) {
            // Reset PID when enabling
            this.pidController.reset(this.targetPosition);
        } else {
            // Stop motor when disabling
            this.robot.turretMotor.setPower(0);
        }
    }
}
