package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.aim.action.Action;
import org.firstinspires.ftc.teamcode.aim.action.PedroPathingTurnAction;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class PetroTest extends LinearOpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private Action turnAction = null;

    private void doInit() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
        this.turnAction = new PedroPathingTurnAction("testTurn", this.follower, Math.toRadians(90), true);
    }

    /*private void startTurn() {
        Pose targetPos = new Pose(
               follower.getPose().getX(),
               follower.getPose().getY(),
                follower.getHeading() + Math.toRadians(90));
        PathChain turnPath =follower.pathBuilder()
                .addPath(new BezierPoint(targetPos))
                .setLinearHeadingInterpolation(
                        follower.getHeading(), targetPos.getHeading()).setTimeoutConstraint(500)
                .build();
       follower.followPath(turnPath, true);
       // follower.holdPoint(new BezierPoint(targetPos), follower.getHeading() + Math.toRadians(90));
    }

    private boolean isFinished() {
        if (follower.atParametricEnd() &&
                follower.getHeadingError() < follower.getCurrentPath().getPathEndHeadingConstraint()) {
            return true;
        } else if (follower.getVelocity().getMagnitude() <
                follower.getCurrentPath().getPathEndVelocityConstraint() &&
                follower.getPose().distanceFrom(follower.getCurrentPath().endPose()) < 2.54 &&
                follower.getAngularVelocity() < 0.055) {
            return true;
        }
        return false;
    }
   private boolean isFinished() {
       if (follower.getHeadingError() < follower.getCurrentPath().getPathEndHeadingConstraint() ||
               follower.getAngularVelocity() < 0.055) {
           return true;
       }
       return false;
   }

    private void startMove() {
        Pose ballPose = new Pose(follower.getPose().getX() + 20, follower.getPose().getY(), follower.getHeading() + Math.toRadians(90));
        PathChain driveToBallPath = follower.pathBuilder()
                .addPath((new BezierLine(follower.getPose(), ballPose)))
                .setLinearHeadingInterpolation(follower.getHeading(), ballPose.getHeading())
                .build();
        follower.followPath(driveToBallPath,  true);
    }*/

    public void runOpMode() {
        doInit();
        waitForStart();
        int status = 0;

        while (opModeIsActive()) {
            follower.update();
            turnAction.update();

            switch (status) {
                case 0:
                    //startTurn();
                   // startMove();
                    turnAction.start();
                    status = 1;
                    break;
                case 1:
                    //if (follower.isBusy() || !isFinished()) {
                    //if (follower.isBusy()) {
                    if (!turnAction.isFinished()) {
                        telemetry.addData("Status", "Turning....");
                    } else {
                        telemetry.addData("Status", "Turn completed");
                    }
                    telemetry.addData("Angular Velocity", "%f", follower.getAngularVelocity());
                    telemetry.addData("Heading", "%f", Math.toDegrees(follower.getHeading()));
                    telemetry.update();
                    break;
            }

        }
    }
}
