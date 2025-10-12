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

public class AirTagTrackingAction extends Action{
    private final Robot robot;
    private int status = 0;
    private Pose ballPose = null;
    Vision.ObjectDetectionResult lastDetRet = null;
    private int totalRotationDegrees = 0; // Track total rotation during search
    private String detectionStatus = "searching"; // Track detection status for debug
    private boolean detectionChecked = false; // Track if we already checked for detection this cycle
    private Action turnAction = null;
    private long timer = System.currentTimeMillis();
    private double airTagX, airTagY;
    private int pipeline = 0;
    private double airTagHeight;

    public AirTagTrackingAction(Robot robot, int limelightPipeLine,
                                double airTagX, double airTagY, double airTagHeight) {
        super("searchAndIntake");
        this.robot = robot;
        this.airTagX = airTagX;
        this.airTagY = airTagY;
        this.pipeline = limelightPipeLine;
    }
    public String toString() {
        String statusName;
        switch (this.status) {
            case 0: statusName = "start_turn"; break;
            case 1: statusName = "turning"; break;
            case 2: statusName = "detection"; break;
            default: statusName = "unknown"; break;
        }

        String result = super.toString() +
                " [state=" + statusName +
                ", rotated=" + totalRotationDegrees + "°" +
                ", detection=" + detectionStatus;

        if (this.lastDetRet != null) {
            result += String.format(", airtag distance = (%.2f°) ", this.lastDetRet.distance);
        }
        result += "]";

        return result;
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
           this.robot.vision.startObjectDetection(this.pipeline, RobotConfig.cameraHeight,
                    RobotConfig.cameraAngel, this.airTagHeight);
            this.markStarted();
            this.resetTimer();
            return false;
        }
        switch (this.status) {
            case 0: {
                this.turnAction = new PedroPathingTurnAction("turnToAirTag",
                        this.robot.follower, this.airTagX, this.airTagY, true);
                this.turnAction.start();
                this.status = 1;
                break;
            }
            case 1: { // wait for turn to complete, then go back to detection
                this.turnAction.update();
                if (this.turnAction.isFinished()) {
                    this.status = 2;
                    this.resetTimer();
                }
                break;
            }
            case 2: { // detect ball while stationary
                Vision.ObjectDetectionResult detectionRet = this.robot.vision.getObjectDetectionResult();
                this.lastDetRet = detectionRet;
                if (detectionRet != null) {
                    this.cleanup();
                    return true;
                }
                break;
            }
        }
        return false;
    }
}
