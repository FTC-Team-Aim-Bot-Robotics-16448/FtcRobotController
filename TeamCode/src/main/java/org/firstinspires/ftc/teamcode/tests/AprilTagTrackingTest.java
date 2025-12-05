package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.actions.AprilTagTrackingAction;
import org.firstinspires.ftc.teamcode.aim.components.Button;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
@Disabled
public class AprilTagTrackingTest extends LinearOpMode {
    private Robot robot = new Robot();
    private Button startButton = new Button();
    private Button stopButton = new Button();
    private final Pose startPose = new Pose(DistanceUnit.INCH.fromMm(804),
            DistanceUnit.INCH.fromMm(363), Math.toRadians(90)); // Start Pose of our robot.

    private AprilTagTrackingAction trackingAction = null;

    private void startAprilTagTracking(int llPipeline) {
        if (this.trackingAction != null && !this.trackingAction.isFinished()) {
            // Already tracking, don't start again
            return;
        }
        this.trackingAction = robot.createAprilTagTrackingAction(llPipeline);
        this.trackingAction.start();
        this.trackingAction.enableTurretTurning(true);
        telemetry.addData("Status", "AprilTag tracking started");
    }

    private void stopAprilTagTracking() {
        if (this.trackingAction == null || this.trackingAction.isFinished()) {
            // Not tracking, nothing to stop
            return;
        }
        this.trackingAction.stop();
        telemetry.addData("Status", "AprilTag tracking stop requested");
    }

    private void updateAprilTagTracking() {
        if (this.trackingAction == null) {
            return;
        }
        this.trackingAction.update();
        telemetry.addData("Tracking Status", this.trackingAction.toString());

        if (this.trackingAction.isFinished()) {
            telemetry.addData("Status", "Tracking stopped - manual drive enabled");
        } else {
            telemetry.addData("Status", "Tracking active");
        }
    }

    @Override
    public void runOpMode() {
        robot.init(this, this.startPose);

        telemetry.addData("Controls", "B = Start Tracking, X = Stop Tracking");
        telemetry.update();

        waitForStart();

        robot.start(-1);
        robot.turretMotorRstAct.enableReset(false);

        while (opModeIsActive()) {
            robot.update();
            updateAprilTagTracking();

            // Start tracking when B button is pressed
            this.startButton.update(this.gamepad1.b);
            if (this.startButton.isPressed()) {
                startAprilTagTracking(0);
            }

            // Stop tracking when X button is pressed
            this.stopButton.update(this.gamepad1.x);
            if (this.stopButton.isPressed()) {
                stopAprilTagTracking();
            }

            // Display robot position
            telemetry.addData("Robot Pose", "X: %.2f, Y: %.2f, Heading: %.2f°",
                    robot.follower.getPose().getX(),
                    robot.follower.getPose().getY(),
                    Math.toDegrees(robot.follower.getHeading()));
            telemetry.addData("Turret pos", this.robot.turretMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
