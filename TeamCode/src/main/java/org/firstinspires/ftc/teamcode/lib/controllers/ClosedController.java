package org.firstinspires.ftc.teamcode.lib.controllers;

/**
 * Generalized closed-loop state feedback controller.
 */
public abstract class ClosedController<STATE_TYPE, OUTPUT_TYPE> {

    public abstract void update(STATE_TYPE state);

    public void update(STATE_TYPE state, STATE_TYPE targetState) {
        setTargetState(targetState);
        update(state);
    }

    public abstract void setTargetState(STATE_TYPE state);

    public abstract OUTPUT_TYPE getOutput();

}
