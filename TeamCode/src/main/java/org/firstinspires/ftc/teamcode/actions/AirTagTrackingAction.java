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
    Vision.ObjectDetectionResult lastDetRet = null;
    private Action turnAction = null;
    private long timer = System.currentTimeMillis();
    private double airTagX, airTagY;
    private int pipeline = 0;
    private double airTagHeight;

    public AirTagTrackingAction(Robot robot, int limelightPipeLine,
                                double airTagX, double airTagY, double airTagHeight) {
        super("AirTagTracking");
        this.robot = robot;
        this.airTagX = airTagX;
        this.airTagY = airTagY;
        this.pipeline = limelightPipeLine;
        this.airTagHeight = airTagHeight;
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
                " [state=" + statusName;

        if (this.lastDetRet != null) {
            result += String.format(", airtag distance = (%.2f) id = %d",
                    this.lastDetRet.distance, this.lastDetRet.airTagID);
        }
        if (this.turnAction != null) {
            result += String.format(", turn_status=%s ", this.turnAction.toString());
        }
        result += "]";

        return result;
    }

    private void cleanup() {
        this.robot.vision.stop();
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
        switch (this.status) {
            case 0:
                this.turnAction = new PedroPathingTurnAction("turnToAirTag",
                        this.robot.follower,
                        DistanceUnit.INCH.fromMm(this.airTagX),
                        DistanceUnit.INCH.fromMm(this.airTagY), true);
                this.turnAction.start();
                this.status = 1;
                this.markStarted();
                break;
            case 1: // wait for turn to complete, then go back to detection
                this.turnAction.update();
                if (this.turnAction.isFinished()) {
                    this.robot.vision.startObjectDetection(this.pipeline, RobotConfig.cameraHeight,
                            RobotConfig.cameraAngel, this.airTagHeight);
                    this.robot.follower.breakFollowing();
                    this.status = 2;
                    return true;
                    //this.resetTimer();
                }
                break;
            case 2: // get AprilTag result
                Vision.ObjectDetectionResult detectionRet = this.robot.vision.getObjectDetectionResult();
                this.lastDetRet = detectionRet;
                if (detectionRet != null) {
                    this.cleanup();
                    return true;
                }
                break;
        }
        return false;
    }
}
