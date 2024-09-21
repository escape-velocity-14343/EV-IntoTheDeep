package org.firstinspires.ftc.teamcode.lib.controllers;

import androidx.annotation.Nullable;

public class StaticFeedforwardController implements OpenController<Double, Double> {
    private double kF;

    public StaticFeedforwardController(double kf) {
        this.kF = kf;
    }

    public void setkF(double kf) {
        this.kF = kf;
    }

    public double getkF() {
        return kF;
    }

    @Override
    public Double getOutput() {
        return kF;
    }

}
