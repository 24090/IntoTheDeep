package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.linearslides.OuttakeSlide;

public class Outtake {

    OuttakeSlide slide_right;
    BoundMotor bound_slide_left;
    Servo claw_servo;
    //left arm (looking from the back)
    Servo arm_left;
    Servo arm_right;
    public enum State{
        CLOSED,
        GRABBING,
        SCORING,
        TRANSFER

    }
    public State outtake_state = State.CLOSED;
    public Outtake(HardwareMap hwmap) {
        this.claw_servo = hwmap.get(Servo.class, "es0");
        this.arm_left = hwmap.get(Servo.class, "cs4");
        this.arm_right = hwmap.get(Servo.class, "cs5");
        slide_right = new OuttakeSlide(hwmap.get(DcMotor.class, "cm1"));
        bound_slide_left = new BoundMotor(hwmap.get(DcMotor.class, "cm2"), hwmap.get(DcMotor.class, "cm1"));
        bound_slide_left.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        bound_slide_left.startThread();
    }

    public void score_position(){
        outtake_state = State.SCORING;

        slide_right.goTo(1200, 50);
        slide_right.waitForMovement();

        claw_servo.setPosition(1);
        arm_left.setPosition(0.33);
        arm_right.setPosition(-0.33);
    }

    public void grabPosition(){
        outtake_state = State.GRABBING;

        slide_right.goTo(100, 50);
        slide_right.waitForMovement();

        claw_servo.setPosition(1);
        arm_left.setPosition(0.66);
        arm_right.setPosition(-0.66);
    }

    public void grabScoreToggle() {
        if (outtake_state == State.GRABBING) {
            score_position();
        } else {
            grabPosition();
        }
    }

    public void close() {
        outtake_state = State.CLOSED;

        slide_right.moveDown();

        claw_servo.setPosition(0);
        arm_left.setPosition(1);
        arm_right.setPosition(-1);
    }

    public void toggleClaw() {
        if ((outtake_state == State.GRABBING) || (outtake_state == State.SCORING)) {
            if (claw_servo.getPosition() == 0) {
                claw_servo.setPosition(1);
            } else {
                claw_servo.setPosition(0);
            }
        }
    }

    public void transferPosition() {
        outtake_state = State.TRANSFER;

        slide_right.moveDown();

        claw_servo.setPosition(1);
        arm_left.setPosition(0.8);
        arm_right.setPosition(-0.8);
    }

    public void activateClaw() {
        if (claw_servo.getPosition() == 0) {
            claw_servo.setPosition(1);
        } else {
            claw_servo.setPosition(0);
        }
    }

    public void score() {
        if (outtake_state == State.SCORING) {
            slide_right.goTo(800, 50);

            slide_right.waitForMovement();
        }
    }
}
