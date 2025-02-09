package org.firstinspires.ftc.teamcode.OldRobot.Util.linearslides;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.GameMap;

public class OuttakeSlide extends LinearSlide {
    /**
     * Class for using the Outtake slide
     */
    public OuttakeSlide(DcMotor motor) {
        super(motor, 4900, 0, 0);
    }

    @Override
    public boolean extendToIter(double pos, double max_error) {
        if ((pos < min_extend) || (pos > max_extend)){
            throw new Error("Requested slide extension out of bounds.");
        }
        motor.setPower(Math.signum(pos -motor.getCurrentPosition()));
        double distance = pos - motor.getCurrentPosition();
        return Math.abs(distance) < max_error;
    }
    @Override
    public void stop(){if (motor.getCurrentPosition() > 2000) {
        motor.setPower(0.1);
    } else {
        motor.setPower(0.0);
    }
    }
    public void moveDown(){
        this.extendToBreaking(0, 50);
    }
    public void moveUp(){
        this.extendToBreaking(4850, 50);
    }
    public int getPos(){
        return motor.getCurrentPosition();
    }
}
