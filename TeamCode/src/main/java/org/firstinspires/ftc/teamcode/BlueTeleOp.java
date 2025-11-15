package org.firstinspires.ftc.teamcode.tests;

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

    private Button intakeButton1 = new Button();
    private Button shootButton1 = new Button();
    private Button turretLeftButton1 = new Button();
    private Button turretRightButton1 = new Button();
    private Button reverseIntakeButton1 = new Button();

    private Button intakeButton2 = new Button();
    private Button shootButton2 = new Button();
    private Button turretLeftButton2 = new Button();
    private Button turretRightButton2 = new Button();
    private Button reverseIntakeButton2 = new Button();

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

        robot.start();

        int i = 0;
        while (opModeIsActive()) {
            robot.update();

            if (this.shootAction != null) {
                this.shootAction.update();
            }
            if (this.intakeAction != null) {
                this.intakeAction.update();
            }

            this.intakeButton1.update(this.gamepad1.b);
            this.shootButton1.update(this.gamepad1.y);
            this.turretLeftButton1.update(this.gamepad1.dpad_left);
            this.turretRightButton1.update(this.gamepad1.dpad_right);
            this.intakeButton2.update(this.gamepad2.b);
            this.shootButton2.update(this.gamepad2.y);
            this.turretLeftButton2.update(this.gamepad2.dpad_left);
            this.turretRightButton2.update(this.gamepad2.dpad_right);
            this.reverseIntakeButton1.update(this.gamepad1.dpad_up);
            this.reverseIntakeButton2.update(this.gamepad2.dpad_up);

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

            // shoot
            if (this.shootButton1.isPressed() || this.shootButton2.isPressed()) {
                if (this.shootAction == null || this.shootAction.isFinished()) {
                    this.shootAction = this.robot.createShooterAction(this.getApriltagAimingPipeline());
                    this.shootAction.start();
                }
            }
            if (turretLeftButton1.isPressed() || turretLeftButton2.isPressed()) {
                this.robot.turnTurret(0.2);
            }
            if (turretLeftButton1.isReleased() || turretLeftButton2.isReleased()) {
                this.robot.turnTurret(0);
                this.robot.turretPosReset();
            }
            if (turretRightButton1.isPressed() || turretRightButton2.isPressed()) {
                this.robot.turnTurret(-0.2);
            }
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
                telemetry.addData("Ty:", "%f", this.shootAction.aprilTagTrackAct.getTy());
            }
            telemetry.addData("Turret pos:", "%d", this.robot.turretMotor.getCurrentPosition());
            telemetry.addData("X:Y", "%f:%f: %f",
                    robot.follower.getPose().getX(), robot.follower.getPose().getY(),
                    Math.toDegrees(robot.follower.getHeading()));
            telemetry.update();
        }
    }
}
