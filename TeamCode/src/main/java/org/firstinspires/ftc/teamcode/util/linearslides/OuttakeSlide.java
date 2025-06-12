package org.firstinspires.ftc.teamcode.util.linearslides;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OuttakeSlide extends LinearSlide {
    public static final double MAX_EXTEND = 1850 ;
    public static final double MIN_EXTEND = 0;

    /**
     * Class for using the Outtake slide
     * @param hwmap the hardware map, used to find the motor
     */
    public OuttakeSlide(HardwareMap hwmap) {
        super(hwmap.get(DcMotor.class, "outtake_slide_left"), MIN_EXTEND, MAX_EXTEND, 0, 50);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public double powerFunction(){
        return Math.signum(target_pos - getPosition());
    }

    @Override
    public void stop(){
        if (getPosition() > 1000) {
            motor.setPower(0.2);
        } else {
            motor.setPower(0.0);
        }
    }

    public void down(){
        this.goTo(0);
    }

    public void up(){
        this.goTo(1850);
    }
}
