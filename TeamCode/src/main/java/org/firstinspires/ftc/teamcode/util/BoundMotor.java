package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;

public class BoundMotor {
    DcMotor target_motor;
    DcMotor motor;
    Thread thread;
    BoundMotor(DcMotor motor, DcMotor target_motor){
        this.motor = motor;
        this.target_motor = target_motor;
        this.thread = new Thread(this::loop);
    }
    void startThread(){
        thread.start();
    }
    void loop(){
        while (true){
            motor.setPower(target_motor.getPower());
        }
    }
}
