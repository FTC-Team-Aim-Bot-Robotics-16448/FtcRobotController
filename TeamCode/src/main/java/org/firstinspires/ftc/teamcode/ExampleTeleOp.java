package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierPoint;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.aim.components.Button;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

import org.firstinspires.ftc.teamcode.aim.action.Action;

@TeleOp
public class ExampleTeleOp extends LinearOpMode{
    private Robot robot = new Robot();
    private Button testButton = new Button();
    private Follower follower;
    private Supplier<PathChain> pathChain;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // Start Pose of our robot.
    private Pose targetPose = null; // Start Pose of our robot.

    private Action balltrackingAction = null;

    private void doInit() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, this::getTargetPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, this::getTargetHeading, 0.8))
                .build();
    }
    public Pose getTargetPose() {
        return this.targetPose;
    }
    public double getTargetHeading() {
        return this.targetPose.getHeading();
    }

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
        //doInit();

        waitForStart();

        robot.start();

        int i = 0;
        while (opModeIsActive()) {
            robot.update();
            //follower.update();
            updateBallTracking();

            this.testButton.update(this.gamepad1.b);
            if (this.testButton.isPressed()) {
                telemetry.addData("button", "pressed %d", i++);
                if (this.balltrackingAction == null) {
                     startBallTracking();
                }
                telemetry.update();

               /* if (RobotConfig.cameraEnabled) {
                    double strafeOffset = robot.vision.getTargetStrafeOffset();
                    double forwardOffset = robot.vision.getTargetForwardOffset();
                    forwardOffset -= RobotConfig.targetHeight;

                    telemetry.addData("Target Strafe offset", strafeOffset);
                    telemetry.addData("Target Forward offset", forwardOffset);

                    if (forwardOffset > 1) {
                        double poseArray[] = calculateTargetPosition(
                                follower.getPose().getX(),
                                follower.getPose().getY(),
                                Math.toDegrees(follower.getHeading()),
                                forwardOffset,
                                -strafeOffset,
                                100
                        );
                        targetPose = new Pose(DistanceUnit.INCH.fromMm(poseArray[0]),
                                DistanceUnit.INCH.fromMm(poseArray[1]),
                                Math.toRadians(poseArray[2]));
                        telemetry.addData("Target X", poseArray[0]);
                        telemetry.addData("Target Y", poseArray[1]);
                        telemetry.addData("Target Heading", poseArray[2]);
                        follower.followPath(pathChain.get());
                    }
                    telemetry.update();
                } */
            }

            /*if (targetPose != null) {
                if (!follower.isBusy()) {
                    telemetry.addData("Status", "Finished!!!");
                } else {
                    telemetry.addData("atParametricEnd", follower.atParametricEnd());
                    telemetry.addData("getHeadingError", "%f vs %f",
                            follower.getHeadingError(), follower.getCurrentPath().getPathEndHeadingConstraint());
                    telemetry.addData("getMagnitude", "%f vs %f",
                            follower.getVelocity().getMagnitude(), follower.getCurrentPath().getPathEndVelocityConstraint());
                    telemetry.addData("distance", "%f vs %f",
                            follower.getPose().distanceFrom(follower.getCurrentPath().endPose()), 2.54 );
                    telemetry.addData("Angular Velocity", "%f vs %f",
                            follower.getAngularVelocity(), 0.055);
                }
                telemetry.update();
            }*/

        }
    }
}
