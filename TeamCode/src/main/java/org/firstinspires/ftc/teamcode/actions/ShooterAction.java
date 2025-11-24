package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.aim.action.Action;
import org.firstinspires.ftc.teamcode.aim.action.ActionWithDelay;
import org.firstinspires.ftc.teamcode.aim.action.CommonAction;
import org.firstinspires.ftc.teamcode.aim.action.EitherOneAction;
import org.firstinspires.ftc.teamcode.aim.action.SeqAction;
import  org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.aim.action.SleepAction;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.function.Supplier;

public class ShooterAction extends Action {
    private final Robot robot;
    private SeqAction seqAct;
    public AprilTagTrackingAction aprilTagTrackAct;
    private int llPipeLineForAiming;
    private boolean prevBallInHood = false;

    private double curLlTy = 0;
    private double curShooterVel = 0;
    private double curLlDist = 0;

    // CSV logging
    private static final boolean ENABLE_CSV_LOGGING = true; // Set to false to disable logging
    private FileWriter csvWriter = null;
    private long shootStartTime = 0;

    public ShooterAction(Robot robot, int llPipeLineForAiming) {
        super("Shoot");
        this.robot = robot;
        this.seqAct = this.shootAllSteps();
        this.llPipeLineForAiming = llPipeLineForAiming;
        this.aprilTagTrackAct = this.robot.createAprilTagTrackingAction(this.llPipeLineForAiming);

        if (ENABLE_CSV_LOGGING) {
            this.initCSVLogging();
        }
    }

    private void initCSVLogging() {
        try {
            // Create timestamp for unique filename
            String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            String filename = "shooter_velocity_" + timestamp + ".csv";

            // Save to FIRST directory for easy access via REV Hub web UI
            File logDir = new File("/sdcard/FIRST/logs");
            if (!logDir.exists()) {
                logDir.mkdirs();
            }

            File logFile = new File(logDir, filename);
            csvWriter = new FileWriter(logFile);

            // Write CSV header
            csvWriter.write("timestamp_ms,elapsed_ms,target_velocity,actual_velocity,decompression_threshold,current_action\n");
            csvWriter.flush();

            shootStartTime = System.currentTimeMillis();

            robot.opMode.telemetry.addData("Shooter Log", "Created: " + filename);
        } catch (IOException e) {
            robot.opMode.telemetry.addData("Shooter Log Error", e.getMessage());
        }
    }

    @Override
    public boolean run() {
        this.aprilTagTrackAct.run();
        this.robot.opMode.telemetry.addData("Shooter Dis:Velocity:Decom","%f:%f:%f",
                this.curLlDist, this.curShooterVel, this.curShooterVel * RobotConfig.shooterMotorDecompressionPer);

        // Log velocity data to CSV
        this.logVelocityData();

        return this.seqAct.run();
    }

    private void logVelocityData() {
        if (csvWriter == null) return;

        try {
            long currentTime = System.currentTimeMillis();
            long elapsed = currentTime - shootStartTime;
            double actualVelocity = this.robot.launchMotor.getVelocity();
            double decompressionThreshold = this.curShooterVel * RobotConfig.shooterMotorDecompressionPer;
            String currentAction = this.seqAct != null ? this.seqAct.getName() : "unknown";

            // Write data row: timestamp, elapsed, target, actual, threshold, action
            csvWriter.write(String.format(Locale.US, "%d,%d,%.2f,%.2f,%.2f,%s\n",
                    currentTime, elapsed, this.curShooterVel, actualVelocity,
                    decompressionThreshold, currentAction));
            csvWriter.flush(); // Flush immediately to ensure data is written even if crash occurs
        } catch (IOException e) {
            // Silently fail to avoid disrupting robot operation
        }
    }

    @Override
    protected void cleanup() {
        this.aprilTagTrackAct.stop();
        this.seqAct = null;
        this.robot.intakeMotor.setPower(0);
        this.robot.launchMotor.setPower(0);
        this.robot.optakeMotor.setPower(0);

        // Close CSV file
        if (csvWriter != null) {
            try {
                csvWriter.close();
                robot.opMode.telemetry.addData("Shooter Log", "Closed successfully");
            } catch (IOException e) {
                robot.opMode.telemetry.addData("Shooter Log Error", "Failed to close");
            }
        }
    }

    private SeqAction shootAllSteps() {
        SeqAction seqAction = new SeqAction("shootAll");

        seqAction.addAction(this.waitingForAiming());
        seqAction.addAction(this.shootStartAction());

        // 1st shoot
        seqAction.addAction(this.waitingForLaunchMotorSpeed());
        seqAction.addAction(new SleepAction("stabilize", 200)); // Wait for flywheel to stabilize
        seqAction.addAction(this.setIntakePower(-0.8, 0));
        seqAction.addAction(this.waitingForLaunchMotorDecompression());
        seqAction.addAction(this.setIntakePower(0, 500));

        // 2nd shoot
        seqAction.addAction(this.waitingForLaunchMotorSpeed());
        seqAction.addAction(new SleepAction("stabilize", 200)); // Wait for flywheel to stabilize
        seqAction.addAction(this.setIntakePower(-0.8, 0));
        seqAction.addAction(this.waitingForLaunchMotorDecompression());
        seqAction.addAction(this.setIntakePower(0, 500));

        // 3rd shoot
        seqAction.addAction(this.waitingForLaunchMotorSpeed());
        seqAction.addAction(new SleepAction("stabilize", 200)); // Wait for flywheel to stabilize
        seqAction.addAction(this.setIntakePower(-0.8, 0));
        seqAction.addAction(this.waitingForLaunchMotorDecompression());
        seqAction.addAction(this.setIntakePower(0, 0));
       /* seqAction.addAction(this.setIntakePower(-1, 0));
        seqAction.addAction(this.waitingForBallInHood(true));
        seqAction.addAction(this.waitingForLaunchMotorSpeed());*/

        /*seqAction.addAction(this.setLaunchPower(-0.55, 0));
        seqAction.addAction(this.setIntakePower(0, 200));
        seqAction.addAction(this.setIntakePower(-1, 80));
        seqAction.addAction(this.setLaunchPower(-0.55, 0));*/
        /*seqAction.addAction(this.setIntakePower(-1, 50));
        seqAction.addAction(this.setLaunchPower(0.9, 0));
        seqAction.addAction(this.setIntakePower(0, 60));
        seqAction.addAction(this.setIntakePower(-1, 50));
        seqAction.addAction(this.setLaunchPower(0.9, 0));
        seqAction.addAction(this.setIntakePower(0, 80));
        seqAction.addAction(this.setIntakePower(-1, 3000));*/
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
            double ballDistance = this.robot.shootDistSensor.getDistance(DistanceUnit.CM);
            if (inHood) {
                if (ballDistance < 3) {
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
        Supplier<Boolean> runFunc = () -> {
            return this.robot.launchMotor.getVelocity() >= curShooterVel - 25 &&
                   this.robot.launchMotor.getVelocity() <= curShooterVel + 25;
        };
        Action act = new CommonAction("waitingLaunchMotor", runFunc);
        return new EitherOneAction("waitingLaunchMotor", act, new SleepAction("timeout", 2000));
    }

    private Action waitingForLaunchMotorDecompression() {
        Supplier<Boolean> runFunc = () -> {
            //return this.robot.launchMotor.getVelocity() < 1100;
            return this.robot.launchMotor.getVelocity() <
                    (curShooterVel * RobotConfig.shooterMotorDecompressionPer) ;
        };
        Action act =  new CommonAction("waitingLaunchMotor",runFunc);
        return new EitherOneAction("waitingDepression", act, new SleepAction("timeout", 2000));
    }

    /*private Action waitingForLaunchMotorDecompression1() {
        Supplier<Boolean> step1Func = () -> {
            //return this.robot.launchMotor.getVelocity() < 1200;
            return this.robot.launchMotor.getVelocity() <
                    (RobotConfig.shooterMotorVelocity * RobotConfig.shooterMotorDecompressionPer) ;
        };
        return new CommonAction("waitingLaunchMotor", step1Func);
    }*/

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

    private double getLaunchVelocity() {
        this.curLlTy = this.aprilTagTrackAct.getTy();
        this.curLlDist = Math.abs(this.aprilTagTrackAct.getDistance());
        this.curShooterVel = RobotConfig.shooterMotorVelocity;

        /*double yInt = 1263; //a value in LSRL equation
        double slope = -16.15; //b value in LSRL equation
        double bx = slope * this.curLlTy;
        this.curShooterVel = bx + yInt;*/

        /*double yInt = 1332; //1301; //a value in LSRL equation
        double slope = -19.70; //b value in LSRL equation
        double bx = slope * this.curLlTy;
        this.curShooterVel = bx + yInt;*/

        double yInt = 821.1; //1301; //a value in LSRL equation
        double slope = 0.2727; //b value in LSRL equation
        double bx = slope * this.curLlDist;
        this.curShooterVel = bx + yInt;

        return this.curShooterVel;
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
