package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.aim.action.Action;
import org.firstinspires.ftc.teamcode.aim.action.ActionWithDelay;
import org.firstinspires.ftc.teamcode.aim.action.CommonAction;
import org.firstinspires.ftc.teamcode.aim.action.SeqAction;

import java.util.function.Supplier;

public class ShooterAction extends Action {
    private final Robot robot;
    private SeqAction seqAct;

    public ShooterAction(Robot robot) {
        super("Shoot");
        this.robot = robot;
        this.seqAct = this.shootAllSteps();
    }

    @Override
    public boolean run() {
        if (seqAct != null) {
            return this.seqAct.run();
        }
        return true;
    }

    @Override
    protected void cleanup() {
        this.seqAct = null;
        this.robot.intakeMotor.setPower(0);
        this.robot.launchMotor.setPower(0);
        this.robot.optakeMotor.setPower(0);
    }

    private SeqAction shootAllSteps() {
        SeqAction seqAction = new SeqAction("shootAll");
        seqAction.addAction(this.robot.createAprilTagTrackingAction());
        seqAction.addAction(this.shootStartAction());
        seqAction.addAction(this.setIntakePower(-1, 200));
        seqAction.addAction(this.setIntakePower(0, 300));
        seqAction.addAction(this.setIntakePower(-1, 200));
        seqAction.addAction(this.setIntakePower(0, 300));
        seqAction.addAction(this.setIntakePower(0, 300));
        seqAction.addAction(this.setIntakePower(-1, 3000));
        seqAction.addAction(this.shootEndAction());
        return seqAction;
    }

    private Action shootStartAction() {
        Supplier<Boolean> step1Func = () -> {
            this.robot.leftLaunchAngle.setPosition(0);
            this.robot.rightLaunchAngle.setPosition(1);
            this.robot.launchMotor.setPower(-0.75);
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

}
