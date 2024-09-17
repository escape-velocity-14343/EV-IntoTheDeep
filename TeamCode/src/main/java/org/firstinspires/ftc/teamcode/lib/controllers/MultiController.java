package org.firstinspires.ftc.teamcode.lib.controllers;

import com.arcrobotics.ftclib.controller.PIDController;

import java.util.Arrays;
import java.util.List;
import java.util.function.BiFunction;

/**
 * Wraps any number of closed loop controllers with any number of open loop controllers for easy feedforward.
 */
public class MultiController<STATE_TYPE, OUTPUT_TYPE> {

    private List<ClosedController<STATE_TYPE, OUTPUT_TYPE>> closedControllers;

    private List<OpenController<OUTPUT_TYPE>> openControllers;

    private BiFunction<OUTPUT_TYPE, OUTPUT_TYPE, OUTPUT_TYPE> add;

    private OUTPUT_TYPE defaultOutput;

    /**
     * @param closedControllers The set of closed controllers
     * @param openControllers The set of open controllers.
     * @param add A lambda adding two members of the output type together. For example, for a double this would look like (a, b) -> a + b
     * @param defaultOutput The output that is given when no controllers exist.
     */
    public MultiController(List<ClosedController<STATE_TYPE, OUTPUT_TYPE>> closedControllers,
                           List<OpenController<OUTPUT_TYPE>> openControllers,
                           BiFunction<OUTPUT_TYPE, OUTPUT_TYPE, OUTPUT_TYPE> add,
                           OUTPUT_TYPE defaultOutput) {

        this.closedControllers = closedControllers;
        this.openControllers = openControllers;
        this.add = add;
        this.defaultOutput = defaultOutput;
    }

    public void update(STATE_TYPE state) {
        for (ClosedController controller : closedControllers) {
            controller.update(state);
        }
    }

    public void update(STATE_TYPE state, STATE_TYPE targetState) {
        setTargetState(targetState);
        update(state);
    }

    public void setTargetState(STATE_TYPE state) {
        for (ClosedController controller : closedControllers) {
            controller.setTargetState(state);
        }
    }

    public OUTPUT_TYPE getOutput() {
        OUTPUT_TYPE output;
        if (closedControllers.size() > 0) {
            output = closedControllers.get(0).getOutput();
            for (int i = 1; i < closedControllers.size(); i++) {
                output = add.apply(output, closedControllers.get(i).getOutput());
            }
            output = add.apply(output, openControllers.get(0).getOutput());
        } else if (openControllers.size() > 0) {
            output = openControllers.get(0).getOutput();
        } else {
            return defaultOutput;
        }

        for (int i = 1; i < openControllers.size(); i++) {
            output = add.apply(output, openControllers.get(i).getOutput());
        }

        return output;
    }

}
