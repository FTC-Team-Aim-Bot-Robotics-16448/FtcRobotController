package org.firstinspires.ftc.teamcode.aim.action;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public class PedroPathingTurnAction extends Action {
    private Follower follower;
    private boolean holdEnd;
    private double turnToRad = 0;

    @Override
    public String toString() {
        return super.toString() + " angle:" + Math.toDegrees(this.turnToRad);
    }

    public PedroPathingTurnAction(String name, Follower f, double turnToRad, boolean holdEnd) {
        super(name);
        follower = f;
        this.holdEnd = holdEnd;
        this.turnToRad = turnToRad;
    }

    public PedroPathingTurnAction(String name, Follower f,
                                  double targetX, double targetY, boolean holdEnd) {
        super(name);
        follower = f;
        this.holdEnd = holdEnd;

        double deltaY = targetY - this.follower.getPose().getY();
        double deltaX = targetX - this.follower.getPose().getX();
        this.turnToRad = Math.atan2(deltaY, deltaX);
    }

    private boolean isTurnFinished() {
         if (follower.isBusy()) {
            return false;
        }
        return true;

        /* TODO: do custom completion checking if needed
         if (follower.getHeadingError() < follower.getCurrentPath().getPathEndHeadingConstraint() ||
                follower.getAngularVelocity() < 0.055) {
            return true;
        }
        return false;
        */
    }

    private void startTurn() {
        Pose targetPos = new Pose(
                follower.getPose().getX(),
                follower.getPose().getY(),
                turnToRad);
        PathChain turnPath =follower.pathBuilder()
                .addPath(new BezierPoint(targetPos))
                .setLinearHeadingInterpolation(
                        follower.getHeading(), targetPos.getHeading()).setTimeoutConstraint(300)
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
