package org.firstinspires.ftc.teamcode.actions;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.aim.action.*;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.Supplier;

public class IntakeAction extends Action {
    private final Robot robot;
    // initialized with a big number to prevent intake running at start.
    private double ballDistance = 100;
    private double triggeedDistance = -1;
    private boolean distSensorReady = false;
    private boolean reverse = false;
    private boolean stopping = false;
    private Timer stopTimer;

    public IntakeAction(Robot robot, boolean reverse) {
        super("Intake");
        this.robot = robot;
        this.reverse = reverse;
        this.stopTimer = new Timer();
    }

    public String toString() {
        String result = super.toString() +
                " [ball_distance=" + this.ballDistance +
                "trigger=" + this.triggeedDistance +
                "]";
        return result;
    }

    private boolean reverseRun() {
        this.robot.intakeMotor.setPower(1.0);
        this.robot.optakeMotor.setPower(-1);
        return false;
    }

    @Override
    public boolean run() {
        if (this.reverse) {
            return reverseRun();
        }

        if (this.stopping) {
            //if (stopTimer.getElapsedTimeSeconds() > 0.3) {
                this.robot.intakeMotor.setPower(0);
                this.stopping = false;
            //}
        }
        if (!this.distSensorReady) {
            double dist = this.robot.intakeDistSensor.getDistance(DistanceUnit.CM);
            if (dist > 10 && dist < 50) {
                this.distSensorReady = true;
            } else {
                return false;
            }
        }
        this.ballDistance = this.robot.intakeDistSensor.getDistance(DistanceUnit.CM);
        this.robot.optakeMotor.setPower(1);
        if (this.ballDistance > 0 && this.ballDistance < 18) {
            this.triggeedDistance = this.ballDistance;
            if (this.stopping) {
                this.stopping = false;
            }
            this.robot.intakeMotor.setPower(-1.0);
        } else {
            //this.robot.intakeMotor.setPower(0);
            if (!this.stopping) {
                this.stopping = true;
                this.stopTimer.resetTimer();
            }
        }
        return false;
    }

    @Override
    protected void cleanup() {
        this.robot.optakeMotor.setPower(0);
        this.robot.intakeMotor.setPower(0);
    }
}
