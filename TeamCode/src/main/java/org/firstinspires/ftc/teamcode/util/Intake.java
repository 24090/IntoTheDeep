package org.firstinspires.ftc.teamcode.util;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.linearslides.IntakeSlide;

public class Intake {
    Servo intake_servo_a1;
    Servo intake_servo_b;
    Servo intake_servo_a2;
    public IntakeSlide linear_slide;
    public Intake(HardwareMap hwmap){
        this.intake_servo_a1 = hwmap.get(Servo.class, "intake_servo_a1");
        this.intake_servo_a2 = hwmap.get(Servo.class, "intake_servo_a2");
        this.intake_servo_b  = hwmap.get(Servo.class, "intake_servo_b");
        this.linear_slide = new IntakeSlide(hwmap);
        intake_servo_a1.setPosition(0.83);
        intake_servo_a2.setPosition(1-0.83);
    }
    public double moveUp(){
        intake_servo_a1.setPosition(0.83);
        intake_servo_a2.setPosition(1-0.83);
        return Math.abs((0.83 - intake_servo_a1.getPosition())) + Math.abs((1-0.83 - intake_servo_a2.getPosition()));
    }
    public double moveDown(){
        intake_servo_a1.setPosition(0.05);
        intake_servo_a2.setPosition(1 - 0.05);
        return Math.abs((0.05 - intake_servo_a1.getPosition())) + Math.abs((1-0.05 - intake_servo_a2.getPosition()));
    }
    public void grab(){
        intake_servo_b.setPosition(0);
    }
    public void hold(){
        intake_servo_b.setPosition(0.3);
    }
    public void stop(){
        intake_servo_b.setPosition(0.5);
    }
    public void release(){
        intake_servo_b.setPosition(0.7);
    }
    public void slideOut(){
        linear_slide.moveOut();
    }
    public void slideIn(){linear_slide.moveIn();}
    public void slideTo(double pos){linear_slide.goTo(pos, linear_slide.max_error);}
    public void transferSample(){
        this.hold();
        this.moveUp();
        this.slideIn();
        linear_slide.waitForMovement();
        while (intake_servo_a1.getPosition() < 0.83 && intake_servo_a2.getPosition() > 1-0.83){}
        this.release();
        try {Thread.sleep(1000);} catch(InterruptedException e) {
            e.printStackTrace();
        }
        this.stop();
    }
    public void readyIntake(){
        this.moveDown();
        this.slideOut();
        while (intake_servo_a1.getPosition() > 0.05){}
        this.grab();
    }
    public void slideStop(){
        linear_slide.stopThread();
        linear_slide.stop();
    }
}
