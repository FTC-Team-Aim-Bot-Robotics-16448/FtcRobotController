package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.aim.action.Action;
import org.firstinspires.ftc.teamcode.aim.action.ActionWithDelay;
import org.firstinspires.ftc.teamcode.aim.action.CommonAction;
import org.firstinspires.ftc.teamcode.aim.action.EitherOneAction;
import org.firstinspires.ftc.teamcode.aim.action.ParallelAction;
import org.firstinspires.ftc.teamcode.aim.action.SeqAction;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.aim.action.SleepAction;
import org.firstinspires.ftc.teamcode.aim.utils.MathUtils;
import org.firstinspires.ftc.teamcode.aim.utils.Logging;

import java.io.IOException;
import java.util.function.Supplier;

public class ShooterAction extends Action {
    public enum DisCalMode {
        CAMERA,
        FIXED
    }
    private final Robot robot;
    public SeqAction seqAct;
    public AprilTagTrackingAction aprilTagTrackAct;

    private int llPipeLineForAiming;
    private boolean prevBallInHood = false;
    private DisCalMode disCalMode = DisCalMode.CAMERA;
    private Logging log = null;
    private boolean shouldStopLaunchMotor = true;

    private double curLlTy = 0;
    public double curLlDist = 0;
    public int shootCount = 0;
    public int shootCountBySensor = 0;

    public double setShooterVel = 0;
    private double preShooterVel = 0;
    private double curShooterVel = 0;

    // Distance sensor caching
    private double cachedDisSensorValue = 0;
    private long lastDisSensorReadTime = 0;
    private static final long DIS_SENSOR_READ_TIMEOUT_MS = 50; // 50ms timeout


    public ShooterAction(Robot robot, int llPipeLineForAiming, boolean shouldStopLaunchMotor) {
        super("Shoot");
        this.robot = robot;
        this.seqAct = this.shootAllSteps();
        this.llPipeLineForAiming = llPipeLineForAiming;
        this.aprilTagTrackAct = this.robot.aprilTagTrackAct;
        this.shouldStopLaunchMotor = shouldStopLaunchMotor;

        if (RobotConfig.shooterPanelsEnabled) {
            try {
                this.log = new Logging("shooter");
                this.log.setFieldsLine("velocity,pre-vel,distance");
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public String toString() {
        String s = super.toString();
        if (this.seqAct != null) {
            s += ":" + this.seqAct.toString();
        }
        return s;
    }

    public void enableFixedDisCalMode(double vel) {
        this.disCalMode = DisCalMode.FIXED;
        this.setShooterVel = vel;
    }

    private double readDisSensor() {
        /*long currentTime = System.currentTimeMillis();
        if (currentTime - lastDisSensorReadTime >= DIS_SENSOR_READ_TIMEOUT_MS) {
            cachedDisSensorValue = this.robot.shootDistSensor.getDistance(DistanceUnit.CM);
            lastDisSensorReadTime = currentTime;
        }*/
        cachedDisSensorValue = this.robot.shootDistSensor.getDistance(DistanceUnit.CM);
        return cachedDisSensorValue;
    }

    @Override
    public boolean run() {
        this.aprilTagTrackAct.enableTurretTurning(true);
        this.curShooterVel = this.robot.launchMotor.getVelocity();
        this.readDisSensor();

        boolean ret = this.seqAct.run();

        if (RobotConfig.shooterPanelsEnabled) {
            try {
                log.write("%f,%f,%f", curShooterVel, this.preShooterVel, this.cachedDisSensorValue);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        this.preShooterVel = this.curShooterVel;
        return ret;
    }

    @Override
    protected void cleanup() {
        if (RobotConfig.shooterPanelsEnabled) {
            try {
                this.log.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        this.aprilTagTrackAct.enableTurretTurning(false);
        this.seqAct = null;
        this.robot.intakeMotor.setPower(0);
        this.robot.optakeMotor.setPower(0);
        if (this.shouldStopLaunchMotor) {
            this.robot.launchMotor.setPower(0);
        }
    }

    private SeqAction shootAllSteps() {
        SeqAction seqAction = new SeqAction("shootAll");

        if (this.disCalMode == DisCalMode.CAMERA) {
            seqAction.addAction(this.waitingForAiming());
            seqAction.addAction(this.shootStartAction());
        } else if (this.disCalMode == DisCalMode.FIXED) {
            //seqAction.addAction(this.shootStartAndAimingAction());
            seqAction.addAction(this.waitingForAiming());
            //seqAction.addAction(this.shootStartAction());
        }

        // 1st shoot
        seqAction.addAction(this.waitingForLaunchMotorSpeed());
        seqAction.addAction(this.setIntakePower(-0.8, 0));
        seqAction.addAction(this.waitingForBallInHood(true));
        seqAction.addAction(this.setIntakePower(0, 0));
        //seqAction.addAction(new SleepAction("stabilize", 1000));
        seqAction.addAction(this.waitingForBallInHood(false));

        // 2nd shoot
        seqAction.addAction(this.waitingForLaunchMotorSpeed());
        seqAction.addAction(this.setIntakePower(-0.8, 0));
        seqAction.addAction(this.waitingForBallInHood(true));
        seqAction.addAction(this.setIntakePower(0, 0));
        //seqAction.addAction(new SleepAction("stabilize", 1000));
        seqAction.addAction(this.waitingForBallInHood(false));

        // 3rd shoot
        seqAction.addAction(this.waitingForLaunchMotorSpeed());
        seqAction.addAction(this.setIntakePower(-0.8, 0));
        seqAction.addAction(this.waitingForBallInHood(true));
        seqAction.addAction(this.setIntakePower(0, 0));
        seqAction.addAction(this.waitingForBallInHood(false));

        seqAction.addAction(this.shootEndAction());
        return seqAction;
    }

    private Action waitingForAiming() {
        Supplier<Boolean> step1Func = () -> {
            boolean aimed = this.aprilTagTrackAct.aprilTagAimed();
            return aimed;
        };
        return new CommonAction("aprilTagAiming", step1Func);
    }

    private Action waitingForBallInHood(boolean inHood) {
        Supplier<Boolean> step1Func = () -> {
            if (inHood) {
                boolean ret = false;
                for (int i = 0; i < 1; i++) {
                    double ballDistance = this.cachedDisSensorValue;
                    if (ballDistance <= 13) {
                        this.shootCount++;
                        this.shootCountBySensor++;
                        ret = true;
                        break;
                    }
                    if (this.curShooterVel < this.preShooterVel &&
                            (this.preShooterVel - this.curShooterVel) / this.preShooterVel >= RobotConfig.shooterMotorCompressionPer) {
                        ret = true;
                        this.shootCount++;
                        break;
                    }
                }
                return ret;
            } else {
                double ballDistance = this.cachedDisSensorValue;
                if (ballDistance >= 17) {
                    return true;
                }
            }
            return false;
        };
        Action act = new CommonAction("waitingBallInHood", step1Func);
        return new EitherOneAction("waitingBallInHood", act, new SleepAction("timeout", 2000));
    }

    private Action waitingForLaunchMotorSpeed() {
        Supplier<Boolean> runFunc = () -> {
            return this.robot.launchMotor.getVelocity() >= setShooterVel - 25 &&
                   this.robot.launchMotor.getVelocity() <= setShooterVel + 25;
        };
        Action act = new CommonAction("waitingLaunchMotor", runFunc);
        return new EitherOneAction("waitingLaunchMotor", act, new SleepAction("timeout", 2000));
    }

    private double getLaunchVelocity() {
        this.curLlTy = this.aprilTagTrackAct.getTy();
        this.curLlDist = Math.abs(this.aprilTagTrackAct.getDistance());

        if (this.disCalMode == DisCalMode.FIXED) {
            return this.setShooterVel;
        }
        this.setShooterVel = RobotConfig.shooterMotorVelocity;

        if (!RobotConfig.shooterTraining) {
            double yInt = 832.5;
            double slope = 0.2163;
            double bx = slope * this.curLlDist;
            this.setShooterVel = bx + yInt;
        }

        return this.setShooterVel;
    }

    private Action shootStartAction() {
        Supplier<Boolean> step1Func = () -> {
            this.robot.leftLaunchAngle.setPosition(0.8);
            double velocity = this.getLaunchVelocity();
            this.robot.launchMotor.setVelocity(velocity);
            this.robot.optakeMotor.setPower(1);
            return true;
        };
        Action step1Act = new CommonAction("ShootStart", step1Func);
        return  new ActionWithDelay("ShootStep1", step1Act, 1000);
    }

    private Action shootStartAndAimingAction() {
        ParallelAction paraAct = new ParallelAction("ShootAndAiming");
        paraAct.addAction(this.shootStartAction());
        paraAct.addAction(this.waitingForAiming());
        return paraAct;
    }

    private Action startTransferAndWaitForBallInHood() {
        ParallelAction paraAct = new ParallelAction("TransferAndBallin");
        paraAct.addAction(this.waitingForBallInHood(true));
        paraAct.addAction(this.setIntakePower(-0.8, 0));
        return paraAct;

    }

    private Action shootEndAction() {
        Supplier<Boolean> step1Func = () -> {
            this.robot.intakeMotor.setPower(0);
            if (this.shouldStopLaunchMotor) {
                this.robot.launchMotor.setPower(0);
            }
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
