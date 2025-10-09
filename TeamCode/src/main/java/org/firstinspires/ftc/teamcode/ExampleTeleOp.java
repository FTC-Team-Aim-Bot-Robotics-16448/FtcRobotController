package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.aim.action.Action;
import org.firstinspires.ftc.teamcode.aim.components.Button;


@TeleOp
public class ExampleTeleOp extends LinearOpMode {
    private Robot robot = new Robot();
    private Button testButton = new Button();
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // Start Pose of our robot.

    private Action balltrackingAction = null;

    private void startBallTracking() {
        this.balltrackingAction = robot.createBallTrackingAction();
        this.balltrackingAction.start();
    }

    private void updateBallTracking() {
        if (this.balltrackingAction == null) {
            return;
        }
        this.balltrackingAction.update();
        telemetry.addData("Status", this.balltrackingAction.toString());

        if (this.balltrackingAction.isFinished()) {
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
            updateBallTracking();

            this.testButton.update(this.gamepad1.b);
            if (this.testButton.isPressed()) {
                telemetry.addData("button", "pressed %d", i++);
                if (this.balltrackingAction == null) {
                    startBallTracking();
                }
                telemetry.update();
            }
        }
    }
}
