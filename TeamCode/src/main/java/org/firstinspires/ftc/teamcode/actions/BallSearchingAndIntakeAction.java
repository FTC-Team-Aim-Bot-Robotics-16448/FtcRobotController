package org.firstinspires.ftc.teamcode.actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.aim.Vision;
import org.firstinspires.ftc.teamcode.aim.action.*;
import org.firstinspires.ftc.teamcode.aim.utils.MathUtils;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConfig;

public class BallSearchingAndIntakeAction extends Action{
    private final Robot robot;
    private int status = 0;
    private Pose ballPose = null;

    public BallSearchingAndIntakeAction(Robot robot) {
        super("searchAndIntake");
        this.robot = robot;
    }
    public String toString() {
        if (this.ballPose == null) {
            return super.toString() + ": ball not found";
        } else {
            return super.toString() + ": " + this.ballPose.toString();
        }
    }
    @Override
    public boolean run() {
        if (!this.isStarted()) {
           this.robot.vision.startObjectDetection(RobotConfig.limeLightDetectBallPipeLine, RobotConfig.cameraHeight,
                    RobotConfig.cameraAngel, RobotConfig.targetHeight);
            this.markStarted();
        }
        switch (this.status) {
            case 0: { // search for a ball
                Vision.ObjectDetectionResult detectionRet = this.robot.vision.getObjectDetectionResult();
                if (detectionRet != null) {
                    if (this.robot.follower.isBusy()) {
                        this.robot.follower.breakFollowing();
                    }
                    double forwardOffset = detectionRet.forwardOffset + RobotConfig.cameraToCentorForward;
                    double strafeOffset = detectionRet.strafeOffset + RobotConfig.cameraToCentorStrafe;
                    double poseArray[] = MathUtils.calculateTargetPosition(
                            DistanceUnit.MM.fromInches(this.robot.follower.getPose().getX()),
                            DistanceUnit.MM.fromInches(this.robot.follower.getPose().getY()),
                            Math.toDegrees(this.robot.follower.getHeading()),
                            forwardOffset,
                            -strafeOffset,
                            RobotConfig.intakeToCenterForward);

                    ballPose = new Pose(DistanceUnit.INCH.fromMm(poseArray[0]),
                            DistanceUnit.INCH.fromMm(poseArray[1]),
                            Math.toRadians(poseArray[2]));
                    this.status =1;
               } else if (!this.robot.follower.isBusy()) {
                    // if ball is not detected, slowly turn around to search
                    Pose targetPos = new Pose(this.robot.follower.getPose().getX(), this.robot.follower.getPose().getY(),
                            this.robot.follower.getHeading() + Math.toRadians(350));
                    PathChain turnPath = this.robot.follower.pathBuilder()
                            .addPath(new BezierPoint(targetPos))
                            .setLinearHeadingInterpolation(this.robot.follower.getHeading(), this.robot.follower.getHeading() + Math.toRadians(180))
                            .build();
                    this.robot.follower.followPath(turnPath, 0.4, true);
                }
                break;
            }
            case 1: { //drive to the ball
                PathChain driveToBallPath = this.robot.follower.pathBuilder()
                        .addPath((new BezierLine(this.robot.follower.getPose(), this.ballPose)))
                        .setLinearHeadingInterpolation(this.robot.follower.getHeading(), ballPose.getHeading())
                        .build();
                this.robot.follower.followPath(driveToBallPath, 0.5, true);
                this.status = 2;
                break;
            }
            case 2: { // waiting for robot driving to the ball
                if (!this.robot.follower.isBusy()) {
                    this.status = 3;
                }
                break;
            }
            case 3: { // intake the ball
                return true;
            }
        }
        return false;
    }
};
