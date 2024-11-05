package org.firstinspires.ftc.teamcode.util;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {
    Servo intake_servo_a1;
    Servo intake_servo_b;
    Servo intake_servo_a2;
    public DcMotor intake_slide_motor;
    double kp = 0.1;
    double max = 1100;
    public Intake(Servo intake_servo_a1, Servo intake_servo_a2, Servo intake_servo_b, DcMotor intake_slide_motor){
        this.intake_servo_a1 = intake_servo_a1;
        this.intake_servo_a2 = intake_servo_a2;
        this.intake_servo_b  = intake_servo_b;
        this.intake_slide_motor = intake_slide_motor;
        intake_servo_a1.setPosition(0.77);
    }
    public void moveUp(){

        intake_servo_a1.setPosition(0.75);
        intake_servo_a2.setPosition(1 - 0.75);
    }
    public void moveDown(){
        intake_servo_a1.setPosition(0.03);
        intake_servo_a2.setPosition(1);
    }
    public void grab(){
        intake_servo_b.setPosition(0);
    }
    public void hold(){
        intake_servo_b.setPosition(0.46);
    }
    public void stop(){
        intake_servo_b.setPosition(0.5);
    }
    public void release(){
        intake_servo_b.setPosition(0.6);
    }
    public void slideOut(){
        if (intake_slide_motor.getCurrentPosition() > 1000){
            intake_slide_motor.setPower(0);
        } else {
            intake_slide_motor.setPower(0.4);
        }
    }
    public void slideTo(){
        intake_slide_motor.setPower(1);
        while (intake_slide_motor.getCurrentPosition() <= 1000) {}
        slideStop();
    }
    public void slideIn(){
        while (intake_slide_motor.getCurrentPosition() > 0){
            intake_slide_motor.setPower((intake_slide_motor.getCurrentPosition()*-0.001)-0.1);
        }
        if (intake_slide_motor.getCurrentPosition() < 0){
            intake_slide_motor.setPower(0);
        }
    }
    public Thread slideInAsync(){
        Thread thread = new Thread(this::slideIn);
        thread.start();
        return thread;
    }
    public void power(double power){
        intake_slide_motor.setPower(power);
    }
    public void transferProcess() throws InterruptedException {
        this.moveUp();
        this.slideIn();
        while (intake_servo_a1.getPosition() < 0.75){}
        this.release();
        Thread.sleep(1000);
        this.stop();

    }
    public Thread slideOutAsync(){
        Thread thread = new Thread(this::slideOut);
        thread.start();
        return thread;
    }
    public void slideStop(){
        intake_slide_motor.setPower(0);
    }
}
