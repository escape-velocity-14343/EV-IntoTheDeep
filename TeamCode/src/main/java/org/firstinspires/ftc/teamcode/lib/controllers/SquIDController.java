package org.firstinspires.ftc.teamcode.lib.controllers;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import org.firstinspires.ftc.teamcode.lib.controllers.ClosedController;

public class SquIDController implements ClosedController<Double, Double> {

    // im lazy and don't want to bother
    private PIDController internalController;

    private double output;


    /**
     * Default constructor with just the coefficients
     *
     * @param kq Quadratic proportional term.
     * @param ki Integral term.
     * @param kd Derivative term.
     */
    public SquIDController(double kq, double ki, double kd) {
        internalController = new PIDController(kq, ki, kd);
    }

    /**
     * The extended constructor.
     *
     * @param kq Quadratic proportional term.
     * @param ki Integral term.
     * @param kd Derivative term.
     * @param sp Target state.
     * @param pv Current state.
     */
    public SquIDController(double kq, double ki, double kd, double sp, double pv) {
        internalController = new PIDController(kq, ki, kd, sp, pv);
    }

    @Override
    public void update(Double state) {
        output = Math.sqrt(internalController.calculate(state));
    }

    @Override
    public void setTargetState(Double state) {
        internalController.setSetPoint(state);
    }

    @Override
    public Double getOutput() {
        return output;
    }

    public void setSquID(double kq, double ki, double kd) {
        setQ(kq);
        setI(ki);
        setD(kd);
    }

    public void reset() {
        internalController.reset();
    }

    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    public void setTolerance(double positionTolerance, double velocityTolerance) {
        internalController.setTolerance(positionTolerance, velocityTolerance);
    }

    public boolean atSetPoint() {
        return internalController.atSetPoint();
    }

    public double getPositionError() {
        return internalController.getPositionError();
    }

    public double getVelocityError() {
        return internalController.getVelocityError();
    }

    public void setIntegrationBounds(double integralMin, double integralMax) {
        internalController.setIntegrationBounds(integralMin, integralMax);
    }

    public void clearTotalError() {
        internalController.clearTotalError();
    }

    public void setQ(double kq) {
        internalController.setP(kq);
    }

    public void setI(double ki) {
        internalController.setI(ki);
    }

    public void setD(double kd) {
        internalController.setD(kd);
    }

    public double getQ() {
        return internalController.getP();
    }

    public double getI() {
        return internalController.getI();
    }

    public double getD() {
        return internalController.getD();
    }

}
