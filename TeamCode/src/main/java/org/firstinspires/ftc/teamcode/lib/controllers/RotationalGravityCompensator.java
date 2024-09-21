package org.firstinspires.ftc.teamcode.lib.controllers;

public class RotationalGravityCompensator implements OpenController<Double, Double> {

    private double kg;
    private double angle;

    public RotationalGravityCompensator(double kg) {
        this.kg = kg;
    }

    /**
     * @param angle In Radians. Gravity operates on the y-axis, so an angle of 0 or pi would correspond with the maximum gravitational force and thus
     * the highest feedforward.
     */
    @Override
    public void update(Double angle) {
        this.angle = angle;
    }

    @Override
    public Double getOutput() {
        return kg * Math.cos(angle);
    }

    public double getKg() {
        return kg;
    }

    public void setKg(double kg) {
        this.kg = kg;
    }
}
