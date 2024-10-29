package org.firstinspires.ftc.teamcode.util;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {
    public DcMotor arm_motor;
    public Servo arm_servo;
    double initial_motor_ticks;
    public Intake(Servo arm_servo, DcMotor arm_motor){
        this.arm_motor = arm_motor;
        this.arm_servo = arm_servo;
        initial_motor_ticks = arm_motor.getCurrentPosition();
        arm_motor.setTargetPosition((int) initial_motor_ticks);
        arm_motor.setMode(RUN_USING_ENCODER);
    }
    public void moveUp(){
        arm_motor.setTargetPosition((int) initial_motor_ticks + 5000);
    }
    public void moveDown(){
        arm_motor.setTargetPosition(0);
    }
    public void grab(){
        arm_servo.setPosition(180);
    }
    public void release(){
        arm_servo.setPosition(0);
    }
}
