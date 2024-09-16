package org.firstinspires.ftc.teamcode.lib.utils;

import org.ejml.simple.SimpleMatrix;

public class RotationMatrix {
    /**
     * @param theta In Radians.
     */
    public static SimpleMatrix getRotationMatrix2D(double theta) {
        return new SimpleMatrix(new double[][]{{Math.cos(theta), -Math.sin(theta)},
                                                {Math.sin(theta), Math.cos(theta)}});
    }
}
