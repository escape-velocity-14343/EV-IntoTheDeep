package org.firstinspires.ftc.teamcode.lib.controllers;

/**
 * Generalized open-loop state feedback controller.
 */
public interface OpenController<OutputT> {

    OutputT getOutput();

}
