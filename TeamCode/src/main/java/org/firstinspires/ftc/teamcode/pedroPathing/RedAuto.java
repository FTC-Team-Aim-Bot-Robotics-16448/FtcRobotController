package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Example Auto", group = "Examples")
public class RedAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private double brakingStart = 1.2;

    private int pathState;

    private final Pose startPose = new Pose(126.1, 121, Math.toRadians(37)); // Start Pose of our robot.

    private final Pose gatePose = new Pose(130, 69.41226215644821, Math.toRadians(90));
    private final Pose scorePose = new Pose(88.3, 87.3, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose start1Pose = new Pose(88.3, 87.3, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(125.73361522198732, 83.41649048625793, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose start2Pose = new Pose(103.50951374207189, 58.7568710359408, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(125.73361522198732, 59.36575052854123, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose start3Pose = new Pose(105.03171247357295, 35.315010570824526, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(125.4291754756871, 35.315010570824526, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private PathChain scorePreload;
    private PathChain grabPickup1, scorePickup1, openGate, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    private final double SCORING_DELAY_SECONDS = 5.0; // 5 seconds delay for scoring actions
    private final double PATH_PAUSE_DELAY_SECONDS = 1.0; // 1 second delay after path completion

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .setBrakingStart(brakingStart)
                .addPath(new BezierLine(startPose, start1Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), start1Pose.getHeading())
                .addPath(new BezierLine(start1Pose, pickup1Pose))
                .setLinearHeadingInterpolation(start1Pose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        openGate = follower.pathBuilder()
                .setBrakingStart(brakingStart)
                .addPath(new BezierLine(pickup1Pose, gatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .setBrakingStart(brakingStart)
                .addPath(new BezierLine(gatePose, start2Pose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), start2Pose.getHeading())
                .addPath(new BezierLine(start2Pose, pickup2Pose))
                .setLinearHeadingInterpolation(start2Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .setBrakingStart(brakingStart)
                .addPath(new BezierLine(scorePose, start3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), start3Pose.getHeading())
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
                follower.followPath(scorePreload, 0.5, true);
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */
                    // Start a non-blocking delay here for scoring
                    setPathState(10); // Transition to a new state for the first delay
                }
                break;
            case 10: // First non-blocking delay (after scorePreload)
                if (pathTimer.getElapsedTimeSeconds() > SCORING_DELAY_SECONDS) {
                    setPathState(2); // After delay, proceed to grab pickup 1
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    setPathState(20); // Transition to a new state for path pause after previous path
                }
                break;
            case 20: // Path pause after previous path, before grabPickup1
                if (pathTimer.getElapsedTimeSeconds() > PATH_PAUSE_DELAY_SECONDS) {
                    /* Grab Sample */
                    follower.followPath(grabPickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1, true);
                    setPathState(11); // Transition to a new state for the second delay
                }
                break;
            case 11: // Second non-blocking delay (after scorePickup1)
                if (pathTimer.getElapsedTimeSeconds() > SCORING_DELAY_SECONDS) {
                    setPathState(8); // After delay, proceed to openGate
                }
                break;
            case 8: // Initiate openGate
                follower.followPath(openGate);
                setPathState(40); // Transition to a new state to wait for openGate to finish
                break;
            case 40: // Wait for openGate to complete
                if (!follower.isBusy()) {
                    setPathState(41); // Transition to a new state for path pause after openGate
                }
                break;
            case 41: // Path pause after openGate
                if (pathTimer.getElapsedTimeSeconds() > PATH_PAUSE_DELAY_SECONDS) {
                    // Now proceed to grabPickup2 logic which starts in case 4
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabPickup2, true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    // Corrected: After grabPickup2, we score the second pickup
                    follower.followPath(scorePickup2, true);
                    setPathState(12); // Transition to a new state for the third delay
                }
                break;
            case 12: // Third non-blocking delay (after scorePickup2)
                if (pathTimer.getElapsedTimeSeconds() > SCORING_DELAY_SECONDS) {
                    setPathState(6); // After delay, proceed to grab pickup 3
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    setPathState(30); // Transition to a new state for path pause after previous path
                }
                break;
            case 30: // Path pause after previous path, before grabPickup3
                if (pathTimer.getElapsedTimeSeconds() > PATH_PAUSE_DELAY_SECONDS) {
                    /* Grab Sample */
                    follower.followPath(grabPickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    /* Score Sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(13); // Transition to a new state for the fourth delay
                }
                break;
            case 13: // Fourth non-blocking delay (after scorePickup3)
                if (pathTimer.getElapsedTimeSeconds() > SCORING_DELAY_SECONDS) {
                    setPathState(31); // After delay, proceed to path pause before stopping
                }
                break;
            case 31: // Path pause before stopping
                if (pathTimer.getElapsedTimeSeconds() > PATH_PAUSE_DELAY_SECONDS) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
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

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        try {
            buildPaths();
        } catch (RuntimeException e) { // Changed to RuntimeException because InterruptedException is removed
            throw new RuntimeException(e);
        }
        follower.setStartingPose(startPose);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}