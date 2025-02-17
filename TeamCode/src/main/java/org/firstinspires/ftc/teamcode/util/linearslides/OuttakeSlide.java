package org.firstinspires.ftc.teamcode.util.linearslides;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OuttakeSlide extends LinearSlide {
    /**
     * Class for using the Outtake slide
     * @param hwmap the hardware map, used to find the motor
     */
    public OuttakeSlide(HardwareMap hwmap) {
        super(hwmap.get(DcMotor.class, "outtake_slide_motor"), 5050, 0, 0, 50);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public double powerFunction(){
        return Math.signum(target_pos - getPosition());
    }

    @Override
    public void stop(){
        if (getPosition() > 2000) {
            motor.setPower(0.1);
        } else {
            motor.setPower(0.0);
        }
    }

    public void moveDown(){
        this.goTo(0, 50);
    }
    public void moveUp(){
        this.goTo(5000, 50);
    }
}
