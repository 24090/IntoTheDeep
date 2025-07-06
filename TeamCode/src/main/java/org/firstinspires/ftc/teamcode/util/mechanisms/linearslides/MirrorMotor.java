package org.firstinspires.ftc.teamcode.util.mechanisms.linearslides;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MirrorMotor {
    DcMotor mirror_motor;
    DcMotor main_motor;

    public MirrorMotor(DcMotor mirror_motor, DcMotor main_motor, DcMotorSimple.Direction direction) {
        this.mirror_motor = mirror_motor;
        this.main_motor = main_motor;
        mirror_motor.setDirection(direction);
        mirror_motor.setZeroPowerBehavior(main_motor.getZeroPowerBehavior());
    }

    public void update(){
        mirror_motor.setPower(main_motor.getPower());
    }
}
