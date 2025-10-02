package org.firstinspires.ftc.teamcode.aim.action;

import java.util.ArrayList;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.*;

public class PetroPathingAction  extends Action {
    private Follower follower;
    private PathChain pathChain;
    private boolean holdEnd;

    public PetroPathingAction(String name, Follower f, PathChain p, boolean holdEnd) {
        super(name);
        follower = f;
        pathChain = p;
        this.holdEnd = holdEnd;
    }

    public PetroPathingAction(String name, Follower follower, Path drivePath, boolean holdEnd) {
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
            this.follower.followPath(this.pathChain,this.holdEnd);
            markStarted();
        }
        return !this.follower.isBusy();
    }
}
