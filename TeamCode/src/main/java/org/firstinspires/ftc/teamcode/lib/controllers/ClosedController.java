package org.firstinspires.ftc.teamcode.lib.controllers;

/**
 * Generalized closed-loop state feedback controller.
 */
public interface ClosedController<StateType, OutputType> {

    void update(StateType state);

    default void update(StateType state, StateType targetState) {
        setTargetState(targetState);
        update(state);
    }

    void setTargetState(StateType state);

    OutputType getOutput();

}
