package org.firstinspires.ftc.teamcode.aim.action;

public class EitherOneAction extends Action {
    private Action action1;
    private Action action2;
    private boolean finished1;
    private boolean finished2;

    public EitherOneAction(String name, Action action1, Action action2) {
        super(name);
        this.action1 = action1;
        this.action2 = action2;
        this.finished1 = false;
        this.finished2 = false;
    }

    @Override
    public boolean run() {
        if (action1 == null || action2 == null) {
            return true;
        }
        if (!finished1) {
            finished1 = action1.run();
        }
        if (!finished2) {
            finished2 = action2.run();
        }
        // Return true if either action is finished
        return finished1 || finished2;
    }

    @Override
    protected void cleanup() {
        // Clean up both actions when this action finishes
        if (action1 != null && !action1.isFinished()) {
            action1.stop();
        }
        if (action2 != null && !action2.isFinished()) {
            action2.stop();
        }
    }

    @Override
    public String toString() {
        String s = super.getName();
        s += "\n  Either: " + (action1 != null ? action1.toString() : "null");
        s += "\n  Or: " + (action2 != null ? action2.toString() : "null");
        return s;
    }
}
