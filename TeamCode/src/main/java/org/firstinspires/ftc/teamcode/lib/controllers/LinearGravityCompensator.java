package org.firstinspires.ftc.teamcode.lib.controllers;

public class LinearGravityCompensator implements OpenController<Double, Double> {

    private double kg;
    private double angle;

    public LinearGravityCompensator(double kg) {
        this.kg = kg;
    }

    /**
     * @param angle In Radians. Gravity operates on the y-axis, so an angle of pi/2 would correspond with the maximum gravitational force and thus
     * the highest feedforward.
     */
    @Override
    public void update(Double angle) {
        this.angle = angle;
    }

    @Override
    public Double getOutput() {
        return kg * Math.sin(angle);
    }

    public double getKg() {
        return kg;
    }

    public void setKg(double kg) {
        this.kg = kg;
    }
}
