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
    double kp = 0.1;
    public enum state {
        CONTAINED,
        STARTING,
        READY_GRAB,
        GRABBING,
        TRANSFER
    }
    double timer_start_ms = 0;
    state target_state = state.CONTAINED;
    state current_state = state.STARTING;
    public boolean stateMachine(){
        switch (current_state){
            case STARTING:
                switch (target_state){
                    case STARTING:
                        break;
                    default:
                        toContained();
                        break;
                }
                break;
            case CONTAINED:
                switch (target_state){
                    case CONTAINED:
                        break;
                    case STARTING:
                        moveUp();
                        if ((intake_servo_a1.getPosition()<0.75) ||(intake_servo_a2.getPosition()>1-0.75)){
                            break;
                        }
                        current_state = state.STARTING;
                        break;
                    case READY_GRAB:
                    case GRABBING:
                    case TRANSFER:
                        toReadyGrab();
                        break;
                }
                break;
            case READY_GRAB:
                switch (target_state){
                    case CONTAINED:
                    case STARTING:
                        toContained();
                        break;
                    case READY_GRAB:
                        // DO NOTHING
                        break;
                    case GRABBING:
                    case TRANSFER:
                        grab();
                        current_state = state.GRABBING;
                        break;
                }
                break;
            case GRABBING:
                switch (target_state){
                    case CONTAINED:
                    case STARTING:
                        toContained();
                        break;
                    case READY_GRAB:
                        toReadyGrab();
                        stop();
                        current_state = state.READY_GRAB;
                        break;
                    case GRABBING:
                        // DO NOTHING
                        break;
                    case TRANSFER:
                        hold();
                        current_state = state.TRANSFER;
                        break;
                }
                break;
            case TRANSFER:
                switch (target_state){
                    case CONTAINED:
                    case STARTING:
                        toContained();
                        break;
                    case GRABBING:
                    case READY_GRAB:
                        toReadyGrab();
                        break;
                    case TRANSFER:
                        moveUp();

                        if ( !linear_slide.extendToIter(0, 50)
                             ||(intake_servo_a1.getPosition()<0.75)
                             ||(intake_servo_a2.getPosition()>1-0.75)
                           ){
                            break;
                        }
                        linear_slide.stop();
                        release();
                        if (timer_start_ms != -1){
                            timer_start_ms = System.currentTimeMillis();
                        } else if ((System.currentTimeMillis() - timer_start_ms) > 1000){
                            stop();
                            timer_start_ms = -1;
                            current_state = state.STARTING;
                            target_state = state.CONTAINED;
                            return true;
                        }
                        break;
                }
                break;
        }
        return false;
    }
    public void toContained(){
        moveDown();
        stop();
        if (linear_slide.extendToIter(1000, 50)
            &&(intake_servo_a1.getPosition() <=     0.03)
            &&(intake_servo_a2.getPosition() >= 1 - 0.03)
        ){
            linear_slide.stop();
            current_state = state.CONTAINED;
        }

    }
    public void toReadyGrab(){
        moveDown();
        stop();
        if (linear_slide.extendToIter(1000,50)
            &&(intake_servo_a1.getPosition() <=    0.03)
            &&(intake_servo_a2.getPosition() >= 1 - 0.03)
        ){
            current_state = state.READY_GRAB;
        }

    }
    public Intake(Servo intake_servo_a1, Servo intake_servo_a2, Servo intake_servo_b, DcMotor motor){
        this.intake_servo_a1 = intake_servo_a1;
        this.intake_servo_a2 = intake_servo_a2;
        this.intake_servo_b  = intake_servo_b;
        this.linear_slide = new IntakeSlide(motor);
        intake_servo_a1.setPosition(0.79);
        intake_servo_a2.setPosition(1.0-0.79);
    }
    public double moveUp(){
        intake_servo_a1.setPosition(0.79);
        intake_servo_a2.setPosition(1 - 0.79);
        return Math.abs((0.79 - intake_servo_a1.getPosition())) + Math.abs((1-0.79 - intake_servo_a2.getPosition()));
    }
    public double moveDown(){
        intake_servo_a1.setPosition(0.026);
        intake_servo_a2.setPosition(1 - 0.026);
        return Math.abs((0.026 - intake_servo_a1.getPosition())) + Math.abs((1-0.026 - intake_servo_a2.getPosition()));
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
        while (intake_servo_a1.getPosition() < 0.75 && intake_servo_a2.getPosition() > 1-0.75){}
        this.release();
        try {Thread.sleep(1000);} catch(InterruptedException e) {
            e.printStackTrace();
        }
        this.stop();
    }
    public void readyIntake(){
        this.moveDown();
        this.slideOut();
        while (intake_servo_a1.getPosition() > 0.03){}
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
