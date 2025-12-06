package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.actions.ShooterAction;
import org.firstinspires.ftc.teamcode.aim.action.Action;

@Autonomous(name = "RedAuto", group = "Examples")
public class RedAuto extends LinearOpMode {

    private Robot robot = new Robot();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private double brakingStart = 1.2;

    private int pathState;

    private ShooterAction shootAction = null;
    private Action intakeAction = null;

    private Action airTagTrackingAction = null;

    private final Pose startPose = new Pose(126.1, 121, Math.toRadians(37)); // Start Pose of our robot.

    private final Pose gatePose = new Pose(130, 72.5, Math.toRadians(90));
    private final Pose scorePose = new Pose(83.3, 92.3, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose start1Pose = new Pose(100, 83.41649048625793, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(128.5, 83.41649048625793, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose start2Pose = new Pose(103.50951374207189, 58.7568710359408, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(125.73361522198732, 59.36575052854123, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose start3Pose = new Pose(105.03171247357295, 35.315010570824526, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(125.4291754756871, 35.315010570824526, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private PathChain scorePreload;
    private PathChain grabStart1, grabPickup1, scorePickup1, openGate;
    private PathChain grabStart2, grabPickup2, scorePickup2;
    private PathChain grabStart3, grabPickup3, scorePickup3;

    private final double SCORING_DELAY_SECONDS = 1.0; // 5 seconds delay for scoring actions
    private final double PATH_PAUSE_DELAY_SECONDS = 1.0; // 1 second delay after path completion

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabStart1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, start1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), start1Pose.getHeading())
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(start1Pose, pickup1Pose))
                .setLinearHeadingInterpolation(start1Pose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, scorePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, gatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabStart2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, start2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), start2Pose.getHeading())
                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(start2Pose, pickup2Pose))
                .setLinearHeadingInterpolation(start2Pose.getHeading(), pickup2Pose.getHeading())
                .build();
        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath (new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabStart3 PathChain. */
        grabStart3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, start3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), start3Pose.getHeading())
                .build();
        /* This is our grabPickup3 PathChain. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(start3Pose, pickup3Pose))
                .setLinearHeadingInterpolation(start3Pose.getHeading(), pickup3Pose.getHeading())
                .build();


        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, 0.9, true);
                this.shootAction = this.robot.createShooterAction(1);
                this.shootAction.start();
                setPathState(11);
                break;
            case 11:
                this.shootAction.update();
                if (this.shootAction.isFinished() && !follower.isBusy()) {
                    follower.followPath(grabStart1, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    this.intakeAction = this.robot.createIntakeAction(false);
                    this.intakeAction.start();
                    follower.followPath(grabPickup1, 0.65, true);
                    setPathState(13);
                }
                break;
            case 13:
                this.intakeAction.update();
                if (!follower.isBusy()) {
                    setPathState(14);
                }
                break;
            case 14:
                this.intakeAction.update();
                if (pathTimer.getElapsedTimeSeconds() > SCORING_DELAY_SECONDS) {
                    this.intakeAction.stop();
                    setPathState(15);
                }
                break;
            case 15: // Initiate openGate
                follower.followPath(openGate);
                setPathState(16); // Transition to a new state to wait for openGate to finish
                break;
            case 16: // Wait for openGate to complete, then start scorePickup1 and ShooterAction concurrently
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1);
                    this.shootAction = this.robot.createShooterAction(1);
                    this.shootAction.start();
                    setPathState(17);
                }
                break;
            case 17:
                this.shootAction.update();
                if (this.shootAction.isFinished() && !follower.isBusy()) {
                    follower.followPath(grabStart2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    this.intakeAction = this.robot.createIntakeAction(false);
                    this.intakeAction.start();
                    follower.followPath(grabPickup2, 0.65, true);
                    setPathState(21);
                }
                break;
            case 21:
                this.intakeAction.update();
                if (!follower.isBusy()) {
                    setPathState(22);
                }
                break;
            case 22:
                this.intakeAction.update();
                if (pathTimer.getElapsedTimeSeconds() > SCORING_DELAY_SECONDS) {
                    this.intakeAction.stop();
                    follower.followPath(scorePickup2);
                    this.shootAction = this.robot.createShooterAction(1);
                    this.shootAction.start();
                    setPathState(23);
                }
                break;
            case 23:
                this.shootAction.update();
                if (this.shootAction.isFinished() && !follower.isBusy()) {
                    follower.followPath(grabStart3, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    this.intakeAction = this.robot.createIntakeAction(false);
                    this.intakeAction.start();
                    follower.followPath(grabPickup3, 0.65, true);
                    setPathState(31);
                }
                break;
            case 31:
                this.intakeAction.update();
                if (!follower.isBusy()) {
                    setPathState(32);
                }
                break;
            case 32:
                this.intakeAction.update();
                if (pathTimer.getElapsedTimeSeconds() > SCORING_DELAY_SECONDS) {
                    this.intakeAction.stop();
                    follower.followPath(scorePickup3);
                    this.shootAction = this.robot.createShooterAction(1);
                    this.shootAction.start();
                    setPathState(33);
                }
                break;
            case 33:
                this.shootAction.update();
                if (this.shootAction.isFinished() && !follower.isBusy()) {
                    //follower.followPath(grabStart3, true);
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void runOpMode() {
        robot.init(this, this.startPose);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = robot.follower; //Constants.createFollower(hardwareMap);
        try {
            buildPaths();
        } catch (
                RuntimeException e) { // Changed to RuntimeException because InterruptedException is removed
            throw new RuntimeException(e);
        }

        waitForStart();

        opmodeTimer.resetTimer();
        robot.start(); // Moved robot.start() here
        setPathState(0);

        while (opModeIsActive()) {
            robot.update();
            // These loop the movements of the robot, these must be called continuously in order to work
            //follower.update();
            autonomousPathUpdate();

            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}