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
    private int totalRotationDegrees = 0; // Track total rotation during search
    private String detectionStatus = "searching"; // Track detection status for debug
    private boolean detectionChecked = false; // Track if we already checked for detection this cycle
    private Action turnAction = null;
    private long timer = System.currentTimeMillis();

    public BallSearchingAndIntakeAction(Robot robot) {
        super("searchAndIntake");
        this.robot = robot;
    }
    public String toString() {
        String statusName;
        switch (this.status) {
            case 0: statusName = "detecting"; break;
            case 1: statusName = "turning"; break;
            case 2: statusName = "driving_to_ball"; break;
            case 3: statusName = "waiting_arrival"; break;
            case 4: statusName = "intaking"; break;
            default: statusName = "unknown"; break;
        }

        String result = super.toString() +
                " [state=" + statusName +
                ", rotated=" + totalRotationDegrees + "°" +
                ", detection=" + detectionStatus;

        if (this.ballPose != null) {
            result += ", ball_at=" + String.format("(%.1f, %.1f, %.1f°)",
                    ballPose.getX(),
                    ballPose.getY(),
                    Math.toDegrees(ballPose.getHeading()));
        }

        result += "]";
        return result;
    }

    /**
     * Calculate the expected target area (ta) for a ball at a given distance
     * @param distance Distance to the ball in mm
     * @return Expected ta value as percentage (0-100)
     */
    private double calculateExpectedTa(double distance) {
        // Calculate the angular size of the ball in both dimensions
        double angularWidthDeg = 2 * Math.toDegrees(Math.atan(RobotConfig.ballDiameter / (2 * distance)));
        double angularHeightDeg = angularWidthDeg; // Ball is circular

        // Calculate percentage of FOV occupied
        double horizontalPercentage = angularWidthDeg / RobotConfig.cameraHorizontalFOV;
        double verticalPercentage = angularHeightDeg / RobotConfig.cameraVerticalFOV;

        // ta is the percentage of total image area occupied
        double expectedTa = (horizontalPercentage * verticalPercentage) * 100.0;

        return expectedTa;
    }

    /**
     * Validate if a detection result is a valid ball based on size and position
     * @param detectionResult The detection result from vision system
     * @return true if valid ball, false if likely a false detection
     */
    private boolean isValidBallDetection(Vision.ObjectDetectionResult detectionResult) {
        if (detectionResult == null) {
            detectionStatus = "no_detection";
            return false;
        }

        // Calculate absolute field position of detected ball
        double forwardOffset = detectionResult.forwardOffset + RobotConfig.cameraToCentorForward;
        double strafeOffset = detectionResult.strafeOffset + RobotConfig.cameraToCentorStrafe;

        double[] poseArray = MathUtils.calculateTargetPosition(
                DistanceUnit.MM.fromInches(this.robot.follower.getPose().getX()),
                DistanceUnit.MM.fromInches(this.robot.follower.getPose().getY()),
                Math.toDegrees(this.robot.follower.getHeading()),
                forwardOffset,
                -strafeOffset,
                RobotConfig.intakeToCenterForward);

        double ballAbsX = poseArray[0];
        double ballAbsY = poseArray[1];

        // Filter 1: Check position limits (absolute field coordinates)
        if (Math.abs(ballAbsX) > RobotConfig.maxBallDetectionX ||
            Math.abs(ballAbsY) > RobotConfig.maxBallDetectionY) {
            detectionStatus = String.format("pos_out_of_bounds(x=%.0f,y=%.0f)", ballAbsX, ballAbsY);
            return false; // Ball is outside valid field area
        }

        // Filter 2: Validate ta against expected value based on distance
        double distance = Math.sqrt(
            detectionResult.strafeOffset * detectionResult.strafeOffset +
            detectionResult.forwardOffset * detectionResult.forwardOffset
        );

        double expectedTa = calculateExpectedTa(distance);

        // Check if actual ta is within tolerance of expected ta
        double taRatio = detectionResult.ta / expectedTa;
        double minRatio = 1.0 - RobotConfig.ballTaTolerance;
        double maxRatio = 1.0 + RobotConfig.ballTaTolerance;

        if (taRatio < minRatio || taRatio > maxRatio) {
            detectionStatus = String.format("size_mismatch(ta=%.2f,expected=%.2f,ratio=%.2f)",
                    detectionResult.ta, expectedTa, taRatio);
            return false; // Size doesn't match expected ball size
        }

        detectionStatus = "valid_ball_found";
        return true; // Passed all filters
    }

    private void cleanup() {
        this.robot.vision.stop();
        this.robot.follower.breakFollowing();
    }

    private void resetTimer() {
        this.timer = System.currentTimeMillis();
    }

    private boolean timerPassed(long time) {
        if (System.currentTimeMillis() < this.timer) {
            this.resetTimer();
        }
        if (System.currentTimeMillis() - this.timer >= time) {
            return true;
        }
        return false;
    }

    @Override
    public boolean run() {
        if (!this.isStarted()) {
           this.robot.vision.startObjectDetection(RobotConfig.limeLightDetectBallPipeLine, RobotConfig.cameraHeight,
                    RobotConfig.cameraAngel, RobotConfig.targetHeight);
            this.markStarted();
            this.resetTimer();
            return false;
        }
        switch (this.status) {
            case 0: { // detect ball while stationary
                Vision.ObjectDetectionResult detectionRet = this.robot.vision.getObjectDetectionResult();
                if (isValidBallDetection(detectionRet)) {
                    // Valid ball detected - proceed to calculate target pose
                    double forwardOffset = detectionRet.forwardOffset + RobotConfig.cameraToCentorForward;
                    double strafeOffset = detectionRet.strafeOffset + RobotConfig.cameraToCentorStrafe;
                    double[] poseArray = MathUtils.calculateTargetPosition(
                            DistanceUnit.MM.fromInches(this.robot.follower.getPose().getX()),
                            DistanceUnit.MM.fromInches(this.robot.follower.getPose().getY()),
                            Math.toDegrees(this.robot.follower.getHeading()),
                            forwardOffset,
                            -strafeOffset,
                            RobotConfig.intakeToCenterForward);

                    ballPose = new Pose(DistanceUnit.INCH.fromMm(poseArray[0]),
                            DistanceUnit.INCH.fromMm(poseArray[1]),
                            Math.toRadians(poseArray[2]));
                    this.status = 2; // Go to drive to ball
                } else {
                    // No valid ball detected - check if we should turn or give up
                    if (totalRotationDegrees >= 360) {
                        // Already searched full 360 degrees - give up
                        this.cleanup();
                        return true;
                    } else if (this.timerPassed(1000)) {
                        this.turnAction = new PedroPathingTurnAction("turnToSearch",
                                this.robot.follower,
                                this.robot.follower.getHeading() + Math.toRadians(90),
                                 true);
                        this.turnAction.start();
                        this.status = 1; // Go to wait for turn to complete
                    }
                }
                break;
            }
            case 1: { // wait for turn to complete, then go back to detection
                //if (!this.robot.follower.isBusy()) {
                this.turnAction.update();
                if (this.turnAction.isFinished()) {
                    this.status = 0; // Go back to detection
                    this.resetTimer();
                    totalRotationDegrees += 90;
                }
                break;
            }
            case 2: { // turn to face the ball (safe debug mode)
                /*PathChain driveToBallPath = this.robot.follower.pathBuilder()
                        .addPath((new BezierLine(this.robot.follower.getPose(), this.ballPose)))
                        .setLinearHeadingInterpolation(this.robot.follower.getHeading(), ballPose.getHeading())
                        .build();
                this.robot.follower.followPath(driveToBallPath, 0.5, true);*/

                // Calculate heading to face the ball
                double targetHeading = Math.atan2(
                        ballPose.getY() - this.robot.follower.getPose().getY(),
                        ballPose.getX() - this.robot.follower.getPose().getX()
                );

                // Turn to face the ball without moving
               /* Pose turnPose = new Pose(
                        this.robot.follower.getPose().getX(),
                        this.robot.follower.getPose().getY(),
                        targetHeading
                );
                PathChain turnToBall = this.robot.follower.pathBuilder()
                        .addPath(new BezierPoint(turnPose))
                        .setLinearHeadingInterpolation(this.robot.follower.getHeading(), targetHeading)
                        .build();
                this.robot.follower.followPath(turnToBall, 0.5, true);*/

                this.turnAction = new PedroPathingTurnAction("turnToBall",
                        this.robot.follower, targetHeading, false);
                this.turnAction.start();

                this.status = 3;
                break;
            }
            case 3: { // waiting for robot driving to the ball
                this.turnAction.update();
                if (this.turnAction.isFinished()) {
                    this.status = 4;
                }
                break;
            }
            case 4: { // intake the ball
                this.cleanup();
                return true;
            }
        }
        return false;
    }
}
