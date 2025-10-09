package org.firstinspires.ftc.teamcode.aim.action;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public class PedroPathingTurnAction extends Action {
    private Follower follower;
    private boolean holdEnd;
    private double turnToRad;

    public PedroPathingTurnAction(String name, Follower f, double turnToRad, boolean holdEnd) {
        super(name);
        follower = f;
        this.holdEnd = holdEnd;
        this.turnToRad = turnToRad;
    }

    private boolean isTurnFinished() {
        if (follower.isBusy()) {
            return false;
        }
        if (follower.getHeadingError() < follower.getCurrentPath().getPathEndHeadingConstraint() ||
                follower.getAngularVelocity() < 0.055) {
            return true;
        }
        return false;
    }

    private void startTurn() {
        Pose targetPos = new Pose(
                follower.getPose().getX(),
                follower.getPose().getY(),
                turnToRad);
        PathChain turnPath =follower.pathBuilder()
                .addPath(new BezierPoint(targetPos))
                .setLinearHeadingInterpolation(
                        follower.getHeading(), targetPos.getHeading()).setTimeoutConstraint(500)
                .build();
        follower.followPath(turnPath, this.holdEnd);
     }

    @Override
    public boolean run() {
        if (!isStarted()) {
           this.startTurn();
           markStarted();
           return false;
        }
        return this.isTurnFinished();
    }
}
