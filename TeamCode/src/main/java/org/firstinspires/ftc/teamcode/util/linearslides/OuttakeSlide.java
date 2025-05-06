package org.firstinspires.ftc.teamcode.util.linearslides;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OuttakeSlide extends LinearSlide {
    public static final double MAX_EXTEND = 3650;
    public static final double MIN_EXTEND = 0;

    /**
     * Class for using the Outtake slide
     * @param hwmap the hardware map, used to find the motor
     */
    public OuttakeSlide(HardwareMap hwmap) {
        super(hwmap.get(DcMotor.class, "outtake_slide_motor"), MIN_EXTEND, MAX_EXTEND, 0, 50);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
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

    public void down(){
        this.goTo(0);
    }

    public void up(){
        this.goTo(3600);
    }
}
