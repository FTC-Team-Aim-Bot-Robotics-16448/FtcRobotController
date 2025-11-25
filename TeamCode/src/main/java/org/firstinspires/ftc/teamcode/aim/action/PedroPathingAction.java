package org.firstinspires.ftc.teamcode.aim.action;

import java.util.function.Supplier;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.*;

public class PedroPathingAction extends Action {
    private Follower follower;
    private PathChain pathChain = null;
    private Supplier<PathChain> pathChainGen = null;
    private boolean holdEnd;
    private double power;

    public PedroPathingAction(String name, Follower f, PathChain p, boolean holdEnd) {
        this(name, f, p, 1, holdEnd);
    }

    public PedroPathingAction(String name, Follower f, PathChain p, double power, boolean holdEnd) {
        super(name);
        follower = f;
        pathChain = p;
        this.holdEnd = holdEnd;
        this.power = power;
    }

    public PedroPathingAction(String name, Follower f, Supplier<PathChain> pathChainGen, boolean holdEnd) {
        this(name, f, pathChainGen, 1, holdEnd);
    }

    public PedroPathingAction(String name, Follower f, Supplier<PathChain> pathChainGen, double power, boolean holdEnd) {
        super(name);
        follower = f;
        pathChain = null;
        this.pathChainGen = pathChainGen;
        this.holdEnd = holdEnd;
        this.power = power;
    }


    public PedroPathingAction(String name, Follower follower, Path drivePath, boolean holdEnd) {
        this(name, follower, new PathChain(drivePath), holdEnd);
    }

    // TODO: an improved way to check path completion
    /*private boolean isFinished() {
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
    }*/

    @Override
    public boolean run() {
        if (!isStarted()) {
            if (this.pathChain != null) {
                this.follower.followPath(this.pathChain, this.power, this.holdEnd);
            } else {
                PathChain chain = this.pathChainGen.get();
                this.follower.followPath(chain, this.power, this.holdEnd);
            }
            markStarted();
        }
        return !this.follower.isBusy();
    }
}
