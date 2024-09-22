package org.firstinspires.ftc.teamcode.lib.controllers;

/**
 * Generalized open-loop state feedback controller.
 * @param <StateType> The type of the input to the system (what your sensors read).
 * @param <OutputType> The type of the output to the system (what your actuators take as input to move).
 */
public interface OpenController<StateType, OutputType> {

    default void update(StateType state) {}

    OutputType getOutput();

}
