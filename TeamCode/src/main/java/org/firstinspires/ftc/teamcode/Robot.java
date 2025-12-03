package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.aim.Vision;
import org.firstinspires.ftc.teamcode.aim.components.Button;
import org.firstinspires.ftc.teamcode.aim.drive.MecanumIMUDrive;
import org.firstinspires.ftc.teamcode.actions.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.bylazar.utils.LoopTimer;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

public class Robot {
    private IMU imu = null;
    private DcMotor frontRightMotor, backRightMotor, frontLeftMotor, backLeftMotor;
    private MecanumIMUDrive driveCtrl;
    public LinearOpMode opMode;
    private Button botRotateButton = new Button();
    private boolean botRotated = false;
    private boolean manualDriveEnabled = false;
    public Vision vision = new Vision();
    public Follower follower;
    public TelemetryManager panelsTelemetry;

    // hardwares for intake and shooter systems
    public DcMotor intakeMotor, turretMotor, optakeMotor;
    public DcMotorEx launchMotor;
    public Servo leftLaunchAngle, rightLaunchAngle;
    public DistanceSensor intakeDistSensor;
    public DistanceSensor shootDistSensor;
    public AprilTagTrackingAction aprilTagTrackAct = null;

    private void initImu() {
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RobotConfig.logoDirection, RobotConfig.usbDirection);
        imu = this.opMode.hardwareMap.get(IMU.class, RobotConfig.imuName);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    private void initWheels() {
        this.frontLeftMotor = this.opMode.hardwareMap.get(DcMotor.class, RobotConfig.frontLeftWheelName);
        this.frontRightMotor = this.opMode.hardwareMap.get(DcMotor.class, RobotConfig.frontRightWheelName);
        this.backLeftMotor = this.opMode.hardwareMap.get(DcMotor.class,  RobotConfig.backLeftWheelName);
        this.backRightMotor = this.opMode.hardwareMap.get(DcMotor.class, RobotConfig.backRightWheelName);

        this.frontLeftMotor.setDirection(RobotConfig.frontLeftWheelDirection);
        this.backLeftMotor.setDirection(RobotConfig.backLeftWheelDirection);
        this.frontRightMotor.setDirection(RobotConfig.frontRightWheelDirection);
        this.backLeftMotor.setDirection(RobotConfig.backLeftWheelDirection);
    }

    private void initMecanumIMUDrive() {
        MecanumIMUDrive gryo = new MecanumIMUDrive();
        MecanumIMUDrive.InitParams params = gryo.defaultParams();
        params.opMode = this.opMode;
        params.imuName = RobotConfig.imuName;
        params.frontLeftWheelName = RobotConfig.frontLeftWheelName;
        params.frontRightWheelName = RobotConfig.frontRightWheelName;
        params.backLeftWheelName =  RobotConfig.backLeftWheelName;
        params.backRightWheelName = RobotConfig.backRightWheelName;
        params.countPerMotorRev = RobotConfig.countPerMotorRev; //537.7; //12;
        params.driveGearReduction = RobotConfig.driveGearReduction; // 1.0; // 19.2;
        params.wheelDiameterInches = RobotConfig.wheelDiameterInches; //4.0; //3.78;
        gryo.init(params);

        this.driveCtrl = gryo;
    }

    private void initVision() {
        vision.init(this.opMode.hardwareMap, this.opMode.telemetry, RobotConfig.cameraName);
    }

    private void initOdometry(Pose startPose) {
        follower = Constants.createFollower(this.opMode.hardwareMap);
        if (startPose != null) {
            follower.setStartingPose(startPose);
            follower.update();
        }
    }

    public void enableManualDrive() {
        if (this.manualDriveEnabled) {
            return;
        }
        if (RobotConfig.usePetroPathingManualDrive) {
            this.follower.startTeleopDrive(true);
        } else {
            this.driveCtrl.setupWheels();
        }
        this.manualDriveEnabled = true;
    }

    public void disableManualDrive() {
        if (!this.manualDriveEnabled) {
            return;
        }
        if (!RobotConfig.usePetroPathingManualDrive) {
            this.driveCtrl.restoreWheels();
        }
        this.manualDriveEnabled = false;
    }

    public void initShooterSystem() {
        launchMotor = this.opMode.hardwareMap.get(DcMotorEx.class, "launchMotor");
        intakeMotor = this.opMode.hardwareMap.get(DcMotor.class, "intakeMotor");
        turretMotor = this.opMode.hardwareMap.get(DcMotor.class, "turretMotor");
        optakeMotor = this.opMode.hardwareMap.get(DcMotor.class, "optakeMotor");
        leftLaunchAngle = this.opMode.hardwareMap.get(Servo.class, "leftLaunchAngle");
        rightLaunchAngle = this.opMode.hardwareMap.get(Servo.class, "rightLaunchAngle");
        intakeDistSensor = this.opMode.hardwareMap.get(DistanceSensor.class, "dist");
        shootDistSensor = this.opMode.hardwareMap.get(DistanceSensor.class, "shootDist");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*MotorConfigurationType motorConfigurationType = launchMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        launchMotor.setMotorType(motorConfigurationType);*/

        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLaunchAngle.setPosition(1);


    }
    public void initPanels() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    public void init(LinearOpMode opMode, Pose startPos) {
        this.opMode = opMode;
        this.initPanels();
        if (!RobotConfig.usePetroPathingManualDrive) {
            this.initImu();
            this.initWheels();
            this.initMecanumIMUDrive();
        }
        this.initOdometry(startPos);

        if (RobotConfig.cameraEnabled) {
            this.initVision();
        }

        if (RobotConfig.shooterEnabled) {
            initShooterSystem();
        }
    }

    public void start(int llPipeline) {
        if (RobotConfig.shooterEnabled) {
            leftLaunchAngle.setPosition(0.6);
        }
        if (RobotConfig.cameraEnabled && llPipeline >= 0) {
            this.aprilTagTrackAct = this.createAprilTagTrackingAction(llPipeline);
            this.aprilTagTrackAct.start();
        }

    public void handleRobotMove() {
        this.botRotateButton.update(this.opMode.gamepad1.x);
        if (this.botRotateButton.isToggleOn()) {
            this.botRotated = true;
        }  else if (this.botRotateButton.isToggleOff()) {
            this.botRotated = false;
        }
        int botDirection = 1;
        if (this.botRotated) {
            botDirection = -1;
        }
        double power = 0, x = 0, y = 0, turn = 0;
        double smallTurn = 0.3, bigTurn = 0.6;

        if (this.opMode.gamepad1.left_stick_x != 0 || this.opMode.gamepad1.left_stick_y != 0) {
            power = 0.5;
            x = (botDirection)*this.opMode.gamepad1.left_stick_x;
            y = -(botDirection)*this.opMode.gamepad1.left_stick_y;
        } else if (this.opMode.gamepad1.right_stick_x != 0 || this.opMode.gamepad1.right_stick_y != 0) {
            power = 1;
            x = (botDirection)*this.opMode.gamepad1.right_stick_x;
            y = -(botDirection)*this.opMode.gamepad1.right_stick_y;
        }

        if (this.opMode.gamepad1.left_bumper){
            turn = bigTurn ;
        } else if (this.opMode.gamepad1.right_bumper){
            turn = -bigTurn;
        } else if (this.opMode.gamepad1.left_trigger > 0.5){
            turn = smallTurn;
        } else if (this.opMode.gamepad1.right_trigger > 0.5){
            turn = -smallTurn;
        }

        if (RobotConfig.usePetroPathingManualDrive) {
            this.follower.setTeleOpDrive(y * power, -x * power, turn, true);
           // this.follower.setTeleOpDrive( power, -power, turn, true);
        } else {
            this.driveCtrl.moveByPower(power, x, y, turn);
        }
    }

    public BallSearchingAndIntakeAction createBallTrackingAction() {
        return new BallSearchingAndIntakeAction(this);
    }

    public AprilTagReadAction createAirTagReadAction() {
        return new AprilTagReadAction(this, 0,
                RobotConfig.goalAirTagX,
                RobotConfig.goalAirTagY,
                RobotConfig.goalAirTagHeight,
                false);
    }

    public AprilTagTrackingAction createAprilTagTrackingAction(int llPipeline) {
        return new AprilTagTrackingAction(this, llPipeline, RobotConfig.goalAirTagHeight);
    }

    public IntakeAction createIntakeAction(boolean reverse) {
        return new IntakeAction(this, reverse);
    }

    public ShooterAction createShooterAction(int llPipelineForAiming, boolean shouldStopLaunchMotor) {
        return new ShooterAction(this,  llPipelineForAiming, shouldStopLaunchMotor);
    }

    public void turnTurret(double turnPower) {
        if (this.turretMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            this.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.turretMotor.setPower(0);
        }
        int pos = this.turretMotor.getCurrentPosition();
        /*if ((pos < -200 && turnPower < 0) ||
                (pos > 200 && turnPower > 0)) {
            this.turretMotor.setPower(0);
            return;
        }*/
        this.turretMotor.setPower(turnPower);
    }

    public void turretSafeCheckAndStop() {
        int pos = this.turretMotor.getCurrentPosition();
        if (pos < -200 || pos > 200) {
            this.turretMotor.setPower(0);
        }
    }

    public void turretPosReset() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean isInFarZone() {
        if (this.aprilTagTrackAct == null) {
            return false;
        }
        return this.aprilTagTrackAct.getDistance() > 2500;
    }

    public void update() {
        follower.update();
        if (manualDriveEnabled) {
            handleRobotMove();
        }
        if (RobotConfig.cameraEnabled) {
            vision.update();
            if (this.aprilTagTrackAct != null) {
                this.aprilTagTrackAct.update();
            }
        }
   }
}
