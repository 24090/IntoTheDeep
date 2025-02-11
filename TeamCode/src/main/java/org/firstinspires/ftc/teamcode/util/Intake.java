package org.firstinspires.ftc.teamcode.util;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.linearslides.IntakeSlide;

public class Intake {
    Servo intake_servo_a1;
    Servo intake_servo_b;
    Servo intake_servo_a2;
    public IntakeSlide linear_slide;
    public Intake(Servo intake_servo_a1, Servo intake_servo_a2, Servo intake_servo_b, DcMotor motor){
        this.intake_servo_a1 = intake_servo_a1;
        this.intake_servo_a2 = intake_servo_a2;
        this.intake_servo_b  = intake_servo_b;
        this.linear_slide = new IntakeSlide(motor);
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
        intake_servo_b.setPosition(0.44);
    }
    public void stop(){
        intake_servo_b.setPosition(0.5);
    }
    public void release(){
        intake_servo_b.setPosition(0.7);
    }
    public void slideOut(){
        linear_slide.extendToBreaking(1000, 50);
    }
    public void slideIn(){linear_slide.extendToBreaking(-50, 50);}

    public Thread slideInAsync(){
        return linear_slide.extendToAsync(0, 50);
    }

    public void transferSample(){
        this.hold();
        this.moveUp();
        this.slideIn();
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
    public Thread slideOutAsync(){
        Thread thread = new Thread(this::slideOut);
        thread.start();
        return thread;
    }
    public void slideStop(){
        linear_slide.stop();
    }
}
