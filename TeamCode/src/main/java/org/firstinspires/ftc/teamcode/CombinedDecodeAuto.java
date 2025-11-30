package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.actions.ShooterAction;
import org.firstinspires.ftc.teamcode.aim.action.Action;
import org.firstinspires.ftc.teamcode.aim.action.ActionWithDelay;
import org.firstinspires.ftc.teamcode.aim.action.EitherOneAction;
import org.firstinspires.ftc.teamcode.aim.action.PedroPathingAction;
import org.firstinspires.ftc.teamcode.aim.action.SeqAction;
import org.firstinspires.ftc.teamcode.aim.action.SleepAction;
import org.firstinspires.ftc.teamcode.aim.components.Menu;

import java.util.Locale;
import java.util.function.Supplier;

@Autonomous(name = "2-in-1 Decode Auto", group = "Examples")
public class CombinedDecodeAuto extends LinearOpMode {
    private Robot robot = new Robot();
    private Follower follower;
    private boolean shouldMirror = false;
    private int startPoseId = 1;
    private int goalAprilTagPipeLine = 1;
    private Action autoAction;
    private String selectedOption;

    private Pose startPose;
    private Pose gatePose;
    private Pose scorePose;
    private Pose start1Pose;
    private Pose pickup1Pose;
    private Pose start2Pose;
    private Pose pickup2Pose;
    private Pose start3Pose;
    private Pose pickup3Pose;

    private Pose getPose(Pose pose) {
        if (shouldMirror) {
            return pose.mirror();
        }
        return pose;
    }

    private void initPos() {
        startPose = getPose(new Pose(126.1, 121, Math.toRadians(37))); // Start Pose of our robot.
        gatePose = getPose(new Pose(130, 69.41226215644821, Math.toRadians(90)));
        scorePose = getPose(new Pose(83.3, 92.3, Math.toRadians(45))); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        start1Pose = getPose(new Pose(100, 83.41649048625793, Math.toRadians(0)));
        pickup1Pose = getPose(new Pose(125.73361522198732, 83.41649048625793, Math.toRadians(0))); // Highest (First Set) of Artifacts from the Spike Mark.
        start2Pose = getPose(new Pose(103.50951374207189, 58.7568710359408, Math.toRadians(0)));
        pickup2Pose = getPose(new Pose(125.73361522198732, 59.36575052854123, Math.toRadians(0))); // Middle (Second Set) of Artifacts from the Spike Mark.
        start3Pose = getPose(new Pose(105.03171247357295, 35.315010570824526, Math.toRadians(0)));
        pickup3Pose = getPose(new Pose(125.4291754756871, 35.315010570824526, Math.toRadians(0))); // Lowest (Third Set) of Artifacts from the Spike Mark.
    }

    private void selectMenu() {
        Menu menu = new Menu();

        menu.addMenuItem("Blue/CloseStart1/3line");
        menu.addMenuItem("Blue/CloseStart1/2line");
        menu.addMenuItem("Blue/CloseStart1/1line");
        menu.addMenuItem("Red/CloseStart1/3line");
        menu.addMenuItem("Red/CloseStart1/2line");
        menu.addMenuItem("Red/CloseStart1/1line");

        telemetry.addLine("Menu Initialized");
        telemetry.addLine("Use gamepad to select options");
        telemetry.update();

        this.selectedOption = menu.show(gamepad1, telemetry);
        //String selectedOption = "/Red/CloseStart1/1line";
        this.initPos();
        follower.setStartingPose(this.startPose);
        follower.update();

        telemetry.clear();
        telemetry.addData("Selected Option", selectedOption);
        //telemetry.addData("Action", this.autoAction.toString());
        telemetry.addLine("Press Start to begin");
        telemetry.update();
    }

    Action genFinalActionWithString(String r) {
        Action finalAction = null;
        String[] parts = r.split("/");

        for (String item : parts) {
            switch (item) {
                case "Blue":
                    shouldMirror = true;
                    goalAprilTagPipeLine = 0;
                    break;
                case "Red":
                    shouldMirror = false;
                    goalAprilTagPipeLine = 1;
                    break;
                case "CloseStart1":
                    startPoseId = 1;
                    break;
                case "CloseStart2":
                    startPoseId = 2;
                    break;
                case "FarStart":
                    startPoseId = 3;
                    break;
                case "3line":
                    finalAction = createFinalAction(3, false);
                    break;
                case "2line":
                    finalAction = createFinalAction(2, false);
                    break;
                case "1line":
                    finalAction = createFinalAction(1, false);
                    break;
            }
        }

        return finalAction;
    }

    private PathChain followLineToPose(Pose target) {
        Pose current = follower.getPose();
        PathChain chain = follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, target))
                .setLinearHeadingInterpolation(
                        current.getHeading(),
                        target.getHeading()
                ).build();
        return chain;
    }

    private static Pose midpointControl(Pose start, Pose target) {
        double midX = (start.getX() + target.getX()) / 2.0;
        double midY = (start.getY() + target.getY()) / 2.0;
        return new Pose(midX, midY, target.getHeading());
    }

    public PathChain followCurveToPose(Pose target) {
        Pose current = follower.getPose();
        Pose control = midpointControl(current, target);
        PathChain chain = follower.pathBuilder()
                .addPath(new BezierCurve(follower::getPose, control, target))
                .setLinearHeadingInterpolation(
                        current.getHeading(),
                        target.getHeading()
                ).build();

        return chain;
    }

    private Action followToScorePoseAct(double power) {
        Supplier<PathChain> pathChainGenFunc = () -> {
            return followLineToPose(scorePose);
        };
        return new PedroPathingAction("followToScorePoseAct", this.follower, pathChainGenFunc, power, true);
    }

    private Action followToOpenGateAct() {
        Supplier<PathChain> pathChainGenFunc = () -> {
            return followLineToPose(gatePose);
        };
        return new PedroPathingAction("followToOpenGate", this.follower, pathChainGenFunc, 0.3, true);
    }

    private Action followToIntakeAct(int line) {
        Pose endIntakePos;
        Pose startGrabPos;
        switch (line) {
            case 1:
                startGrabPos = start1Pose;
                endIntakePos = pickup1Pose;
                break;
            case 2:
                startGrabPos = start2Pose;
                endIntakePos = pickup2Pose;
                break;
            case 3:
                startGrabPos = start3Pose;
                endIntakePos = pickup3Pose;
                break;
            default: // just in case
                startGrabPos = start1Pose;
                endIntakePos = pickup1Pose;
                break;
        }

        // First action: drive to the start of intake
        Supplier<PathChain> pathChainGenFunc1 = () -> {
            return followLineToPose(startGrabPos);
        };
        Action act1 = new PedroPathingAction(
                String.format(Locale.US, "followToGrabStart%d", line),
                this.follower, pathChainGenFunc1, true);

        // Second action: drive and intake at the same time
        Supplier<PathChain> pathChainGenFunc2 = () -> {
            return followLineToPose(endIntakePos);
        };
        Action followAct = new PedroPathingAction(
                String.format(Locale.US, "intaking%d", line),
                this.follower, pathChainGenFunc2, true);

        Action followActWithDelay = new ActionWithDelay("intakingDelay", followAct, 200);

        Action act2, intakeAct;
        if (RobotConfig.shooterEnabled) {
            intakeAct = this.robot.createIntakeAction(false);
         } else {
            intakeAct = new SleepAction("sleep", 300000);
         }
        act2 = new EitherOneAction(
                String.format(Locale.US, "intaking%d", line),
                followActWithDelay, intakeAct);

        // Generate final action
        SeqAction seqAct = new SeqAction(String.format(Locale.US, "followAndIntake%d", line));
        seqAct.addAction(act1);
        seqAct.addAction(act2);
        return seqAct;
    }

    Action shootAct() {
        if (RobotConfig.shooterEnabled) {
            ShooterAction act = this.robot.createShooterAction(goalAprilTagPipeLine);
            act.enableFixedDisCalMode(1130);
            return act;
        } else {
            return new SleepAction("sleep", 10000);
        }
    }

    Action createFinalAction(int totalLines, boolean openGate) {
        SeqAction seqAct = new SeqAction("final");
        // preload shoot
        seqAct.addAction(followToScorePoseAct(0.5));
        seqAct.addAction(shootAct());

        // first line intake, open door and shoot
        seqAct.addAction(followToIntakeAct(1));
        if (openGate) {
            seqAct.addAction(followToOpenGateAct());
        }
        seqAct.addAction(followToScorePoseAct(0.8));
        seqAct.addAction(shootAct());

        if (totalLines >= 2) {
            // second line intake and shoot
            seqAct.addAction(followToIntakeAct(2));
            seqAct.addAction(followToScorePoseAct(0.9));
            seqAct.addAction(shootAct());
        }
        if (totalLines >= 3) {
            // third line intake and shoot
            seqAct.addAction(followToIntakeAct(3));
            seqAct.addAction(followToScorePoseAct(0.9));
            seqAct.addAction(shootAct());
        }
        return seqAct;
    }

    @Override
    public void runOpMode() {
        robot.init(this, null);
        follower = robot.follower;
        this.selectMenu();

        waitForStart();

        robot.start(goalAprilTagPipeLine);
        this.autoAction = genFinalActionWithString(selectedOption);
        autoAction.start();

        while (opModeIsActive()) {
            robot.update();
            if (autoAction != null) {
                autoAction.update();
            }

            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", autoAction.toString());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("aprilTag", this.robot.aprilTagTrackAct.toString());
            telemetry.update();
        }

        autoAction.stop();
    }
}