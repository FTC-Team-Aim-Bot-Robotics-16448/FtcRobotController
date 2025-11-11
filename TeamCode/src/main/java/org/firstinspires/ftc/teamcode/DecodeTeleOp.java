package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.aim.action.Action;
import org.firstinspires.ftc.teamcode.aim.components.Button;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConfig;

@TeleOp
public class DecodeTeleOp extends LinearOpMode {
    private Robot robot = new Robot();
    private Button intakeButton = new Button();
    private Button shootButton = new Button();

    private final Pose startPose = new Pose(DistanceUnit.INCH.fromMm(804),
            DistanceUnit.INCH.fromMm(363), Math.toRadians(90)); // Start Pose of our robot.

    private Action airTagAction = null;
    private Action intakeAction = null;
    private Action shootAction  = null;

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

            this.intakeButton.update(this.gamepad1.b);
            this.shootButton.update(this.gamepad1.y);

            if (this.intakeButton.isToggleOn()) {
                telemetry.addData("button", "pressed %d", i++);
                this.intakeAction = this.robot.createIntakeAction();
                this.intakeAction.start();
                telemetry.update();
            }
            if (this.intakeButton.isToggleOff()) {
                this.intakeAction.stop();
                this.intakeAction = null;
            }
            if (this.shootButton.isToggleOn()) {
                this.shootAction = this.robot.createShooterAction();
                this.shootAction.start();
            }
            if (this.shootButton.isToggleOff()) {
                this.shootAction.stop();
                //this.shootAction = null;
            }

            if (this.intakeAction != null) {
                telemetry.addData("Ball Dist:", "%s", this.intakeAction.toString());
            }

            telemetry.addData("Turret pos:", "%d", this.robot.turretMotor.getCurrentPosition());
            telemetry.addData("X:Y", "%f:%f: %f",
                    robot.follower.getPose().getX(), robot.follower.getPose().getY(),
                    Math.toDegrees(robot.follower.getHeading()));
            telemetry.update();
        }
    }
}
