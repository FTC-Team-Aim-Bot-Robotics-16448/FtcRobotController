package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.aim.action.Action;
import org.firstinspires.ftc.teamcode.aim.components.Button;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConfig;

@TeleOp
public class AirTagTest extends LinearOpMode {
    private Robot robot = new Robot();
    private Button testButton = new Button();
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // Start Pose of our robot.

    private Action airTagAction = null;

    private void startAirTagTracking() {
        this.airTagAction = robot.createBallTrackingAction();
        this.airTagAction.start();
    }

    private void updateAirTagTracking() {
        if (this.airTagAction == null) {
            return;
        }
        this.airTagAction.update();
        telemetry.addData("Status", this.airTagAction.toString());

        if (this.airTagAction.isFinished()) {
            telemetry.addData("Status", "Tracking is done!!!");
        } else {
            telemetry.addData("Status", "Tracking is running");
        }

        telemetry.addData("X:Y", "%f:%f: %f",
                robot.follower.getPose().getX(), robot.follower.getPose().getY(),
                Math.toDegrees(robot.follower.getHeading()));
        telemetry.update();

    }

    @Override
    public void runOpMode() {
        robot.init(this, this.startPose);

        waitForStart();

        robot.start();

        int i = 0;
        while (opModeIsActive()) {
            robot.update();
            updateAirTagTracking();

            this.testButton.update(this.gamepad1.b);
            if (this.testButton.isPressed()) {
                telemetry.addData("button", "pressed %d", i++);
                if (this.airTagAction == null) {
                    startAirTagTracking();
                }
                telemetry.update();
            }
        }
    }
}
