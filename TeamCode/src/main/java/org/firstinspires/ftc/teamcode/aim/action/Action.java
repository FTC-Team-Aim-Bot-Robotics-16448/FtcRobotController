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

    protected void cleanup() {

    }

    public void start() {
        this.finished = run();
        this.markStarted();
    }

    public void stop() {
        if (this.isFinished()) {
            return;
        }
        this.cleanup();
        this.finished = true;
    }


    public void update() {
        if (isFinished()) {
            return;
        }
        this.finished = run();
        if (isFinished()) {
            this.cleanup();
            return;
        }
    }

    public boolean isFinished() {
        return this.finished;
    }
}
