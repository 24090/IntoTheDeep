package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.linearslides.OuttakeSlide;

public class Outtake {
    Servo outtake_servo;
    public OuttakeSlide linear_slide;
    public Outtake(HardwareMap hwmap) {
        this.outtake_servo = hwmap.get(Servo.class, "outtake_servo");
        this.linear_slide = new OuttakeSlide(hwmap);
    }
    public void open(){
        outtake_servo.setPosition(0);
    }
    public void close(){
        outtake_servo.setPosition(0.66);
    }
    public void up(){
        linear_slide.moveUp();
    }
    public void down(){
        linear_slide.moveDown();
    }
}
