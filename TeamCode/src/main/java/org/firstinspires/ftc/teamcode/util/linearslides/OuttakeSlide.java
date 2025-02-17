package org.firstinspires.ftc.teamcode.util.linearslides;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OuttakeSlide extends LinearSlide {
    /**
     * Class for using the Outtake slide
     * @param hwmap the hardware map, used to find the motor
     */
    public OuttakeSlide(DcMotor motor) {
        super(motor, 2300, 0, 0, 50);
    }

    @Override
    public double powerFunction(){
        return Math.signum(target_pos - getPosition());
    }

    @Override
    public void stop(){
        motor.setPower(0.1);
    }

    public void moveDown(){
        this.goTo(0, 50);
        this.waitForMovement();
    }
    public void moveUp(){
        this.goTo(5000, 50);
        this.waitForMovement();
    }
    public int getPos() {
        return motor.getCurrentPosition();
    }
}
