package org.firstinspires.ftc.teamcode.subsytems.drive;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumDrive extends HolonomicDrive {

    private DcMotor fr, fl, br, bl;

    public MecanumDrive(DcMotor fr, DcMotor fl, DcMotor br, DcMotor bl) {
        this.fr = fr;
        this.fl = fl;
        this.br = br;
        this.bl = bl;
    }

    @Override
    public Command drive(double forward, double strafe, double rotate) {
        return new CommandBase() {
            private boolean done = false;
            @Override
            public void execute() {
                double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
                double frontLeftPower = (forward + strafe + rotate) / denominator;
                double backLeftPower = (forward - strafe + rotate) / denominator;
                double frontRightPower = (forward - strafe - rotate) / denominator;
                double backRightPower = (forward + strafe - rotate) / denominator;
                fl.setPower(frontLeftPower);
                bl.setPower(backLeftPower);
                fr.setPower(frontRightPower);
                br.setPower(backRightPower);
                done = true;
            }

            @Override
            public boolean isFinished() {
                return done;
            }
        };
    }


}
