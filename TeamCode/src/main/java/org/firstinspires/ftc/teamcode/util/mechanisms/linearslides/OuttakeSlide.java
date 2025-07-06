package org.firstinspires.ftc.teamcode.util.mechanisms.linearslides;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OuttakeSlide extends LinearSlide {
    public static final int MAX_EXTEND = 1825;
    public static final int MIN_EXTEND = 0;

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
        return (target_pos - getPosition())/75.0;
    }

    @Override
    public void stop(){
        if (target_pos > 1100) {
            motor.setPower(0.08 + 0.1 * (target_pos - getPosition())/50);
        } else {
            motor.setPower(0.1 * (target_pos - getPosition())/50);
        }
    }

    public void down(){
        this.goTo(0);
    }

    public void up(){
        this.goTo(1825);
    }
}
