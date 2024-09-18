package org.firstinspires.ftc.teamcode.lib.controllers;

import java.util.List;
import java.util.function.BiFunction;

/**
 * Wraps any number of closed loop controllers with any number of open loop controllers for easy feedforward.
 */
public class MultiController<StateT, OutputT> {

    private List<ClosedController<StateT, OutputT>> closedControllers;

    private List<OpenController<OutputT>> openControllers;

    private BiFunction<OutputT, OutputT, OutputT> add;

    private OutputT defaultOutput;

    /**
     * @param closedControllers The set of closed controllers
     * @param openControllers The set of open controllers.
     * @param add A lambda adding two members of the output type together. For example, for a double this would look like (a, b) -> a + b
     * @param defaultOutput The output that is given when no controllers exist.
     */
    public MultiController(List<ClosedController<StateT, OutputT>> closedControllers,
                           List<OpenController<OutputT>> openControllers,
                           BiFunction<OutputT, OutputT, OutputT> add,
                           OutputT defaultOutput) {

        this.closedControllers = closedControllers;
        this.openControllers = openControllers;
        this.add = add;
        this.defaultOutput = defaultOutput;
    }

    public void update(StateT state) {
        for (ClosedController controller : closedControllers) {
            controller.update(state);
        }
    }

    public void update(StateT state, StateT targetState) {
        setTargetState(targetState);
        update(state);
    }

    public void setTargetState(StateT state) {
        for (ClosedController controller : closedControllers) {
            controller.setTargetState(state);
        }
    }

    public OutputT getOutput() {
        OutputT output;
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
