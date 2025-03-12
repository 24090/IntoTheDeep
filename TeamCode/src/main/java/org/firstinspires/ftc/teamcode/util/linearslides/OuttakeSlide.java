package org.firstinspires.ftc.teamcode.util.linearslides;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.GameMap;

public class OuttakeSlide extends LinearSlide {
    /**
     * Class for using the Outtake slide
     */
    public OuttakeSlide(DcMotor motor) {
        super(motor, 2300, 0, 0);
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
    public void stop(){
        motor.setPower(0.1);
    }
    public void moveDown(){
        this.extendToBreaking(0, 50);
    }
    public void moveDownAsync(){
        this.extendToAsync(0, 50);
    }
    public void moveUp(){
        this.extendToBreaking(4850, 50);
    }
    public int getPos(){
        return motor.getCurrentPosition();
    }
}
