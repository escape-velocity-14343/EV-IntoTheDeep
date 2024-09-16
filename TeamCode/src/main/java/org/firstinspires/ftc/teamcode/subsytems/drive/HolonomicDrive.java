package org.firstinspires.ftc.teamcode.subsytems.drive;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.lib.utils.RotationMatrix;

public abstract class HolonomicDrive extends SubsystemBase {

    {
        setDefaultCommand(drive(0, 0, 0));
    }

    public abstract Command drive(double forward, double strafe, double rotate);

    /**
     * @param botHeading In radians.
     */
    public Command driveFieldCentric(double forward, double strafe, double rotate, double botHeading) {
        SimpleMatrix movement = new SimpleMatrix(new double[][]{{forward, strafe}});
        SimpleMatrix rotMat = RotationMatrix.getRotationMatrix2D(-botHeading);
        movement = rotMat.mult(movement);
        return drive(movement.get(0, 0), movement.get(1, 0), rotate);
    }

}
