package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.actions.ShooterAction;
import org.firstinspires.ftc.teamcode.aim.action.Action;
import org.firstinspires.ftc.teamcode.aim.components.Button;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class BlueTeleOp extends LinearOpMode {
    private Robot robot = new Robot();

    // intake buttons
    private Button intakeButton1 = new Button();
    private Button intakeButton2 = new Button();
    private Button reverseIntakeButton1 = new Button();
    private Button reverseIntakeButton2 = new Button();

    private Button shootButton1 = new Button();
    private Button shootButton2 = new Button();
    private Button stopShootButton1 = new Button();
    private Button stopShootButton2 = new Button();

    private Button turretLeftButton1 = new Button();
    private Button turretRightButton1 = new Button();
    private Button turretLeftButton2 = new Button();
    private Button turretRightButton2 = new Button();

    private final Pose startPose = new Pose(DistanceUnit.INCH.fromMm(804),
            DistanceUnit.INCH.fromMm(363), Math.toRadians(90)); // Start Pose of our robot.

    private Action airTagTrackingAction = null;
    private Action intakeAction = null;
    private ShooterAction shootAction  = null;

    private final int LL_AIMING_PIPELINE = 0;

    protected int getApriltagAimingPipeline() {
        return 0;
    }

    @Override
    public void runOpMode() {
        robot.init(this, this.startPose);

        waitForStart();

        robot.enableManualDrive();
        robot.start(getApriltagAimingPipeline());
        //robot.aprilTagTrackAct.enableTurretTurning(true);

        int i = 0;
        while (opModeIsActive()) {
            robot.update();

            if (this.shootAction != null) {
                this.shootAction.update();
            }
            if (this.intakeAction != null) {
                this.intakeAction.update();
            }

            // intake buttons
            this.intakeButton1.update(this.gamepad1.b);
            this.intakeButton2.update(this.gamepad2.b);
            this.reverseIntakeButton1.update(this.gamepad1.x);
            this.reverseIntakeButton2.update(this.gamepad2.x);

            // shoot buttons
            this.shootButton1.update(this.gamepad1.y);
            this.shootButton2.update(this.gamepad2.y);
            this.stopShootButton1.update(this.gamepad1.a);
            this.stopShootButton2.update(this.gamepad2.a);

            // turret buttons
            this.turretLeftButton1.update(this.gamepad1.dpad_left);
            this.turretRightButton1.update(this.gamepad1.dpad_right);
            this.turretLeftButton2.update(this.gamepad2.dpad_left);
            this.turretRightButton2.update(this.gamepad2.dpad_right);

            // intake
            if (this.intakeButton1.isToggleOn() || this.intakeButton2.isToggleOn()) {
                if (this.intakeAction == null) {
                    this.intakeAction = this.robot.createIntakeAction(false);
                    this.intakeAction.start();
                }
            }
            if (this.intakeButton1.isToggleOff() || this.intakeButton2.isToggleOff()) {
                this.intakeAction.stop();
                this.intakeAction = null;
            }

            // reverse intake
            if (this.reverseIntakeButton1.isToggleOn() || this.reverseIntakeButton2.isToggleOn()) {
                if (this.intakeAction == null) {
                    this.intakeAction = this.robot.createIntakeAction(true);
                    this.intakeAction.start();
                }
            }
            if (this.reverseIntakeButton1.isToggleOff() || this.reverseIntakeButton2.isToggleOff()) {
                this.intakeAction.stop();
                this.intakeAction = null;
            }

            // start shooter motor
            if (this.shootButton1.isToggleOn() || this.shootButton2.isToggleOn()) {
                this.robot.launchMotor.setVelocity(RobotConfig.shooterMotorVelocity);
            }

            // start shooting
            if (this.shootButton1.isToggleOff() || this.shootButton2.isToggleOff()) {
                if (this.shootAction == null || this.shootAction.isFinished()) {
                    this.shootAction = this.robot.createShooterAction(this.getApriltagAimingPipeline(), true);
                    //this.shootAction.enableFixedDisCalMode(RobotConfig.shooterMotorVelocity);
                    this.shootAction.start();
                }
            }

            // stop shooting
            if (this.stopShootButton1.isPressed() || this.stopShootButton2.isPressed()) {
                this.robot.launchMotor.setPower(0);
                if (this.shootAction != null && !this.shootAction.isFinished()) {
                    this.shootAction.stop();
                }
                this.shootButton1.reset();
                this.shootButton2.reset();
            }

            // turn turret to left
            if (turretLeftButton1.isPressed() || turretLeftButton2.isPressed()) {
                this.robot.turnTurret(0.2);
            }

            // stop turret turning to left
            if (turretLeftButton1.isReleased() || turretLeftButton2.isReleased()) {
                this.robot.turnTurret(0);
                this.robot.turretPosReset();
            }

            // turn turret to right
            if (turretRightButton1.isPressed() || turretRightButton2.isPressed()) {
                this.robot.turnTurret(-0.2);
            }

            // stop turret turning to right
            if (turretRightButton1.isReleased() || turretRightButton2.isReleased() ) {
                this.robot.turnTurret(0);
                this.robot.turretPosReset();
            }

            if (this.intakeAction != null) {
                telemetry.addData("Ball Dist:", "%s", this.intakeAction.toString());
            }

            // Just in case, this might not be needed
            this.robot.turretSafeCheckAndStop();

            if (this.shootAction != null) {
                telemetry.addData("Shooter Ty:", "%f", this.shootAction.aprilTagTrackAct.getTy());
                telemetry.addData("Shooter count", "%d", this.shootAction.shootCount);
                telemetry.addData("Shooter Vel", "%f", this.shootAction.curShooterVel);
                telemetry.addData("Shooter Dis", "%f", this.shootAction.curLlDist);
                //telemetry.addData("Shooter:", this.shootAction.toString());
            }

            telemetry.addData("Ball Dist", "%f", this.robot.shootDistSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Turret pos:", "%d", this.robot.turretMotor.getCurrentPosition());
            telemetry.addData("Launch Motor:", "%f", this.robot.launchMotor.getVelocity());
            telemetry.addData("X:Y", "%f:%f: %f",
                    robot.follower.getPose().getX(), robot.follower.getPose().getY(),
                    Math.toDegrees(robot.follower.getHeading()));
            telemetry.update();
        }
    }
}
