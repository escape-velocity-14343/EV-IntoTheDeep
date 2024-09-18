package org.firstinspires.ftc.teamcode.lib.controllers;

/**
 * Generalized open-loop state feedback controller.
 * @param <OutputType> The type of the controller output (plant input).
 */
public interface OpenController<OutputType> {

    OutputType getOutput();

}
