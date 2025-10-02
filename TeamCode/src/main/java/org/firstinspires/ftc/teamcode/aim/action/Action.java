package org.firstinspires.ftc.teamcode.aim.action;

public abstract class Action {
    private String name;
    private boolean started = false;
    private boolean finished = false;

    public Action(String name) {
        this.name = name;

    }
    public abstract boolean run();

    public String toString() {
        return name;
    }

    public String getName() {
        return name;
    }

    protected void markStarted() {
        started = true;
    }

    protected boolean isStarted() {
        return started;
    }

    public void update() {
        if (isStarted()) {
            this.finished = run();
        }
    }

    public boolean isFinished() {
        return this.finished;
    }
}
