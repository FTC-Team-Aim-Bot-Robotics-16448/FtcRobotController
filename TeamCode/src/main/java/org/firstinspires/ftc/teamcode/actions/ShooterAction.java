package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.aim.action.Action;
import org.firstinspires.ftc.teamcode.aim.action.ActionWithDelay;
import org.firstinspires.ftc.teamcode.aim.action.CommonAction;
import org.firstinspires.ftc.teamcode.aim.action.SeqAction;

import java.util.function.Supplier;

public class ShooterAction extends Action {
    private final Robot robot;
    private SeqAction seqAct;
    public AprilTagTrackingAction aprilTagTrackAct;
    private int llPipeLineForAiming;
    private boolean prevBallInHood = false;

    public ShooterAction(Robot robot, int llPipeLineForAiming) {
        super("Shoot");
        this.robot = robot;
        this.seqAct = this.shootAllSteps();
        this.llPipeLineForAiming = llPipeLineForAiming;
        this.aprilTagTrackAct = this.robot.createAprilTagTrackingAction(this.llPipeLineForAiming);
    }

    @Override
    public boolean run() {
        this.aprilTagTrackAct.run();
        return this.seqAct.run();
    }

    @Override
    protected void cleanup() {
        this.aprilTagTrackAct.stop();
        this.seqAct = null;
        this.robot.intakeMotor.setPower(0);
        this.robot.launchMotor.setPower(0);
        this.robot.optakeMotor.setPower(0);
    }

    private SeqAction shootAllSteps() {
        SeqAction seqAction = new SeqAction("shootAll");
        seqAction.addAction(this.shootStartAction());
        seqAction.addAction(this.waitingForAiming());

        seqAction.addAction(this.setIntakePower(-1, 0));
        seqAction.addAction(this.waitingForBallInHood(true));
        seqAction.addAction(this.setIntakePower(0, 0));

        seqAction.addAction(this.waitingForBallInHood(false));
        seqAction.addAction(this.waitingForLaunchMotorSpeed());
        seqAction.addAction(this.setIntakePower(-1, 0));
        seqAction.addAction(this.waitingForBallInHood(true));
        seqAction.addAction(this.waitingForLaunchMotorSpeed());
        seqAction.addAction(this.setIntakePower(0, 0));

        seqAction.addAction(this.waitingForBallInHood(false));
        seqAction.addAction(this.waitingForLaunchMotorSpeed());
        seqAction.addAction(this.setIntakePower(-1, 0));
        seqAction.addAction(this.waitingForBallInHood(true));
        seqAction.addAction(this.waitingForLaunchMotorSpeed());
        seqAction.addAction(this.setIntakePower(0, 0));

        /*seqAction.addAction(this.setLaunchPower(-0.55, 0));
        seqAction.addAction(this.setIntakePower(0, 200));
        seqAction.addAction(this.setIntakePower(-1, 80));
        seqAction.addAction(this.setLaunchPower(-0.55, 0));*/
        /*seqAction.addAction(this.setLaunchPower(-0.9, 0));
        seqAction.addAction(this.setIntakePower(0, 60));
        seqAction.addAction(this.setIntakePower(-1, 50));
        seqAction.addAction(this.setLaunchPower(-0.9, 0));
        seqAction.addAction(this.setIntakePower(0, 80));
        seqAction.addAction(this.setIntakePower(-1, 3000));*/
        seqAction.addAction(this.shootEndAction());
        return seqAction;
    }

    private Action waitingForAiming() {
        Supplier<Boolean> step1Func = () -> {
            return this.aprilTagTrackAct.aprilTagAimed();
        };
        return new CommonAction("aprilTagAiming", step1Func);
    }

    private Action waitingForBallInHood(boolean inHood) {
        Supplier<Boolean> step1Func = () -> {
            double ballDistance = this.robot.shootDistSensor.getDistance(DistanceUnit.CM);
            if (inHood) {
                if (ballDistance < 2) {
                    return true;
                }
            } else {
                if (ballDistance > 5) {
                    return true;
                }
            }
            return false;
        };
        return new CommonAction("waitingBallInHood", step1Func);
    }

    private Action waitingForLaunchMotorSpeed() {
        Supplier<Boolean> step1Func = () -> {
            return this.robot.launchMotor.getVelocity() > 5000;
        };
        return new CommonAction("waitingLaunchMotor", step1Func);
    }

    private Action waitingForBallShoot() {
        Supplier<Boolean> step1Func = () -> {
            double ballDistance = this.robot.shootDistSensor.getDistance(DistanceUnit.CM);
            boolean ballInHood = false;
            if (ballDistance < 2) {
                ballInHood = true;
            }
            if (this.prevBallInHood && !ballInHood) {
                this.prevBallInHood = ballInHood;
                return true;
            }
            this.prevBallInHood = ballInHood;
            return false;
        };
        return new CommonAction("waitingBallShoot", step1Func);
    }

    private double getLaunchPower() {
        double ty = this.aprilTagTrackAct.getTy();
        return -0.5;
    }

    private Action shootStartAction() {
        Supplier<Boolean> step1Func = () -> {
            this.robot.leftLaunchAngle.setPosition(0);
            this.robot.rightLaunchAngle.setPosition(1);
           // this.robot.launchMotor.setPower(-0.75);
            this.robot.launchMotor.setPower(-0.45);
            this.robot.optakeMotor.setPower(1);
            return true;
        };
        Action step1Act = new CommonAction("ShootStart", step1Func);
        return new ActionWithDelay("ShootStep1", step1Act, 3000);
    }

    private Action shootEndAction() {
        Supplier<Boolean> step1Func = () -> {
            this.robot.intakeMotor.setPower(0);
            this.robot.launchMotor.setPower(0);
            this.robot.optakeMotor.setPower(0);
            return true;
        };
        return new CommonAction("ShootEnd", step1Func);
    }

    private Action setIntakePower(double power, long wait) {
        Supplier<Boolean> step1Func = () -> {
            this.robot.intakeMotor.setPower(power);
            return true;
        };
        Action step1Act = new CommonAction("setIntakePower", step1Func);
        return new ActionWithDelay("withDelay", step1Act,  wait);
    }

    private Action setLaunchPower(double power, long wait) {
        Supplier<Boolean> step1Func = () -> {
            this.robot.launchMotor.setPower(power);
            return true;
        };
        Action step1Act = new CommonAction("setShootPower", step1Func);
        return new ActionWithDelay("withDelay", step1Act,  wait);
    }
}
