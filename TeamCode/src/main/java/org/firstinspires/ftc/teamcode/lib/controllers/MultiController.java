package org.firstinspires.ftc.teamcode.lib.controllers;

import java.util.List;
import java.util.function.BiFunction;

/**
 * Wraps any number of closed loop controllers with any number of open loop controllers for easy feedforward.
 * @param <StateType> The type of the input to the system (what your sensors read).
 * @param <OutputType> The type of the output to the system (what your actuators take as input to move).
 */
public class MultiController<StateType, OutputType> {

    private List<ClosedController<StateType, OutputType>> closedControllers;

    private List<OpenController<StateType, OutputType>> openControllers;

    private BiFunction<OutputType, OutputType, OutputType> add;

    private OutputType defaultOutput;

    /**
     * @param closedControllers The set of closed controllers
     * @param openControllers The set of open controllers.
     * @param add A lambda adding two members of the output type together. For example, for a double this would look like (a, b) -> a + b
     * @param defaultOutput The output that is given when no controllers exist.
     */
    public MultiController(List<ClosedController<StateType, OutputType>> closedControllers,
                           List<OpenController<StateType, OutputType>> openControllers,
                           BiFunction<OutputType, OutputType, OutputType> add,
                           OutputType defaultOutput) {

        this.closedControllers = closedControllers;
        this.openControllers = openControllers;
        this.add = add;
        this.defaultOutput = defaultOutput;
    }

    public void update(StateType state) {
        for (ClosedController controller : closedControllers) {
            controller.update(state);
        }
        for (OpenController controller : openControllers) {
            controller.update(state);
        }
    }

    public void update(StateType state, StateType targetState) {
        setTargetState(targetState);
        update(state);
    }

    public void setTargetState(StateType state) {
        for (ClosedController controller : closedControllers) {
            controller.setTargetState(state);
        }
    }

    public OutputType getOutput() {
        OutputType output;
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
