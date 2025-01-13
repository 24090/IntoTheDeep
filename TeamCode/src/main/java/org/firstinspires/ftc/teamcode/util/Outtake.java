package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.linearslides.OuttakeSlide;

public class Outtake {
    Servo outtake_servo;
    public OuttakeSlide linear_slide;
    public Outtake(Servo outtake_servo, DcMotor motor) {
        this.outtake_servo = outtake_servo;
        this.linear_slide = new OuttakeSlide(motor);
    }
    public void open(){
        outtake_servo.setPosition(0);
    }
    public void close(){
        outtake_servo.setPosition(1);
    }
    public void up(){linear_slide.moveUp();
    }
    public void down(){
        linear_slide.moveDown();
    }
    public void readyTransfer(){
        close();
        down();
    }
    public void scoreProcess() throws InterruptedException {
        open();
        readyTransfer();
    }
}
