package org.firstinspires.ftc.teamcode.lib.controllers;

/**
 * Generalized closed-loop state feedback controller.
 * @param <StateType> The type of the input to the system (what your sensors read).
 * @param <OutputType> The type of the output to the system (what your actuators take as input to move).
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
