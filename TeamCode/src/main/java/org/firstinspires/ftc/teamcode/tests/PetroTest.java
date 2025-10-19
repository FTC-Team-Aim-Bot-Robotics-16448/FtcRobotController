package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.aim.action.Action;
import org.firstinspires.ftc.teamcode.aim.action.PedroPathingTurnAction;
import org.firstinspires.ftc.teamcode.aim.action.ActionWithDelay;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class PetroTest extends LinearOpMode {
    //private Follower follower;
    //private final Pose startPose = new Pose(0, 0, Math.toRadians(90));
    private final Pose startPose = new Pose(DistanceUnit.INCH.fromMm(804),
            DistanceUnit.INCH.fromMm(363), Math.toRadians(90)); // Start Pose of our robot.
    private Robot robot = new Robot();

    private Action turnAction = null;

    private void doInit() {
        /*follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();*/
        robot.init(this, this.startPose);
        robot.disableManualDrive();

        this.turnAction= new PedroPathingTurnAction("testTurn",
                this.robot.follower,
                DistanceUnit.INCH.fromMm( RobotConfig.goalAirTagX),
                DistanceUnit.INCH.fromMm( RobotConfig.goalAirTagY),
                true);
        //this.turnAction = new ActionWithDelay("turnWithDelay", act1, 200);
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
            //follower.update();
            this.robot.update();
            turnAction.update();

            switch (status) {
                case 0:
                    //startTurn();
                   // startMove();
                    turnAction.start();
                    status = 1;
                    break;
                case 1:
                    if (turnAction.isFinished()) {
                        this.robot.follower.breakFollowing();
                        status = 2;
                    }
                    break;
                default:
            }
            if (!turnAction.isFinished()) {
                telemetry.addData("Status", "Turning....");
            } else {
                telemetry.addData("Status", "Turn completed");
            }
            telemetry.addData("Angular Velocity", "%f", this.robot.follower.getAngularVelocity());
            telemetry.addData("Heading", "%f", Math.toDegrees(this.robot.follower.getHeading()));
            telemetry.addData("turnAction", "%s", this.turnAction.toString());
            telemetry.update();

        }
    }
}
