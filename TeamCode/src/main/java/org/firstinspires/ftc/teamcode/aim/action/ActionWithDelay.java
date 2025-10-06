package org.firstinspires.ftc.teamcode.aim.action;

public class ActionWithDelay extends Action{
    private SeqAction seqAct = new SeqAction("delaySeqAct");

    public ActionWithDelay(String name, Action actualAction, long millSecs) {
        super(name);
        this.seqAct.addAction(actualAction);
        this.seqAct.addAction(new SleepAction("delay", millSecs));
    }

    @Override
    public boolean run() {
        if (!this.isStarted()) {
            this.seqAct.start();
            this.markStarted();
            return false;
        }
        return this.seqAct.run();
    }
}
