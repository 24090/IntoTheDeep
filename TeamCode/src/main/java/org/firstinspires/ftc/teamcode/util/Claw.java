package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Claw {
    ServoImplEx claw_servo;
    ServoImplEx wrist_servo_movement;
    ServoImplEx wrist_servo_turret;
    ServoImplEx elbow_servo_left;
    ServoImplEx elbow_servo_right;

    public static final int ELBOW_LEFT_IN = 950;
    public static final int ELBOW_LEFT_OUT = 2000;
    public static final int ELBOW_RIGHT_IN = 2250;
    public static final int ELBOW_RIGHT_OUT = 1200;
    public static final int WRIST_MOVEMENT_IN = 1600;
    public static final int WRIST_MOVEMENT_OUT = 2250;
    public static final int WRIST_TURRET_NEG90 = 1000;
    public static final int WRIST_TURRET_90 = 2200;
    public static final int CLAW_OPEN = 1700;
    public static final int CLAW_CLOSED = 1350;

    public Claw(HardwareMap hardwareMap){
        claw_servo = hardwareMap.get(ServoImplEx.class, "claw_servo");
            claw_servo.setPwmRange(new PwmControl.PwmRange(CLAW_CLOSED, CLAW_OPEN));
            claw_servo.setDirection(Servo.Direction.REVERSE);
        wrist_servo_movement = hardwareMap.get(ServoImplEx.class, "wrist_servo_movement");
            wrist_servo_movement.setPwmRange(new PwmControl.PwmRange(WRIST_MOVEMENT_IN, WRIST_MOVEMENT_OUT));
        wrist_servo_turret = hardwareMap.get(ServoImplEx.class, "wrist_servo_turret");
            wrist_servo_turret.setPwmRange(new PwmControl.PwmRange(WRIST_TURRET_NEG90, WRIST_TURRET_90));
        elbow_servo_left = hardwareMap.get(ServoImplEx.class, "elbow_servo_left");
            elbow_servo_left.setPwmRange(new PwmControl.PwmRange(ELBOW_LEFT_IN, ELBOW_LEFT_OUT));
        elbow_servo_right = hardwareMap.get(ServoImplEx.class, "elbow_servo_right");
            elbow_servo_right.setPwmRange(new PwmControl.PwmRange(ELBOW_RIGHT_OUT, ELBOW_RIGHT_IN));
    }

    public void rotate(double value_radians){
        wrist_servo_turret.setPosition(
                (1+((value_radians/Math.PI)%1))%1
        );
    }

    public void grab(){
        claw_servo.setPosition(1);
    }

    public void open(){
        claw_servo.setPosition(0);
    }

    public void toReadyGrabPos() {
        elbow_servo_left.setPosition(0.92);
        elbow_servo_right.setPosition(0.08);
        wrist_servo_movement.setPosition(1);
    }

    public void toTransferPos() {
        elbow_servo_left.setPosition(0);
        elbow_servo_right.setPosition(1);
        wrist_servo_movement.setPosition(0);
        wrist_servo_turret.setPosition(0.5);
    }


    public void toGrabPos() {
        elbow_servo_left.setPosition(1);
        elbow_servo_right.setPosition(0);
        wrist_servo_movement.setPosition(1);
    }
}
