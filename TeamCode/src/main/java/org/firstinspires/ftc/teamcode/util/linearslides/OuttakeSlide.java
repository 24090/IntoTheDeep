package org.firstinspires.ftc.teamcode.util.linearslides;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

public class OuttakeSlide extends LinearSlide {
    /**
     * Class for using the Outtake slide
     */
    public OuttakeSlide(DcMotor motor) {
        super(motor, 4650, 0, 0);
    }

    @Override
    public boolean extendToIter(double pos, double max_error) {
        if ((pos < min_extend) || (pos > max_extend)){
            throw new Error("Requested slide extension out of bounds.");
        }
        double added_power = -0.2;
        if (pos > motor.getCurrentPosition()) {
            added_power = 0.7;
        }
        double distance = pos - motor.getCurrentPosition();
        motor.setPower((distance / (max_extend - min_extend)) + added_power);
        return Math.abs(distance) < max_error;
    }
    @Override
    public void stop(){
        motor.setPower(0.1);
    }
    public Thread moveUp(){
        return extendToAsync(4550, 250);
    }
    public Thread moveDown(){
        return extendToAsync(0, 50);
    }
}
