package org.firstinspires.ftc.teamcode.util.linearslides;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

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
        double added_power = -0.6;
        if (pos > motor.getCurrentPosition()) {
            added_power = 0.6;
        }
        double distance = pos - motor.getCurrentPosition();
        motor.setPower((distance / (max_extend - min_extend)) + added_power);
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
