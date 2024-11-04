package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {
    double initial_motor_ticks;
    Servo intake_servo_a1;
    Servo intake_servo_a2;
    Servo intake_servo_b;
    DcMotor intake_slide_motor;
    public Intake(Servo intake_servo_a1, Servo intake_servo_a2, Servo intake_servo_b, DcMotor intake_slide_motor){
        this.intake_servo_a1 = intake_servo_a1;
        this.intake_servo_a2 = intake_servo_a2;
        this.intake_servo_b  = intake_servo_b;
        this.intake_slide_motor = intake_slide_motor;
        initial_motor_ticks = intake_slide_motor.getCurrentPosition();
    }
    public void moveUp(){
        intake_servo_a1.setPosition(0.77);
        intake_servo_a2.setPosition(0.77);
    }
    public void moveDown(){
        intake_servo_a1.setPosition(0.03);
        intake_servo_a2.setPosition(0.03);
    }
    public void grab(){
        intake_servo_b.setPosition(0);
    }
    public void hold(){
        intake_servo_b.setPosition(0.45);
    }
    public void stop(){
        intake_servo_b.setPosition(0.5);
    }
    public void release(){
        intake_servo_b.setPosition(0.6);
    }
    public void slideOut(){
        intake_slide_motor.setPower(1);
        while (intake_slide_motor.getCurrentPosition() <= initial_motor_ticks + 1000) {}
        slideStop();
    }
    public void slideTo(){
        intake_slide_motor.setPower(1);
        while (intake_slide_motor.getCurrentPosition() <= initial_motor_ticks + 1000) {}
        slideStop();
    }
    public void slideIn(){
        intake_slide_motor.setPower(-1);
        while (intake_slide_motor.getCurrentPosition() >= initial_motor_ticks) {}
        slideStop();
    }
    public Thread slideInAsync(){
        Thread thread = new Thread(this::slideIn);
        thread.start();
        return thread;
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
