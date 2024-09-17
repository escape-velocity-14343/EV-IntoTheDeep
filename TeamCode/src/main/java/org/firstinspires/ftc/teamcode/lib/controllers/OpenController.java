package org.firstinspires.ftc.teamcode.lib.controllers;

/**
 * Generalized open-loop state feedback controller.
 */
public abstract class OpenController<OUTPUT_TYPE> {

    public abstract OUTPUT_TYPE getOutput();

}
