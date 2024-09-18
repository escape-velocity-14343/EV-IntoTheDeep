package org.firstinspires.ftc.teamcode.lib.controllers;

/**
 * Generalized closed-loop state feedback controller.
 */
public interface ClosedController<StateT, OutputT> {

    void update(StateT state);

    default void update(StateT state, StateT targetState) {
        setTargetState(targetState);
        update(state);
    }

    void setTargetState(StateT state);

    OutputT getOutput();

}
