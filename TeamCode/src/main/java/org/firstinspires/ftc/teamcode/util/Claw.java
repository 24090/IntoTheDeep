package org.firstinspires.ftc.teamcode.util;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Claw {
    ServoImplEx claw_servo;
    ServoImplEx wrist_servo_left;
    ServoImplEx wrist_servo_right;
    ServoImplEx elbow_servo_left;
    ServoImplEx elbow_servo_right;

    public static int ELBOW_LEFT_IN = 1600;
    public static int ELBOW_LEFT_OUT = 2400;
    public static int ELBOW_RIGHT_IN = 1400;
    public static int ELBOW_RIGHT_OUT = 600;
    public static double WRIST_SCALE_FACTOR = (2*PI);
    public static double LEFT_WRIST_OFFSET = 0.5;
    public static double RIGHT_WRIST_OFFSET = 0.5;
    double current_positioning_angle;
    double current_turret_angle;
    public Claw(HardwareMap hardwareMap){
        claw_servo = hardwareMap.get(ServoImplEx.class, "claw_servo");
        wrist_servo_right = hardwareMap.get(ServoImplEx.class, "wrist_servo_right");
            wrist_servo_right.setPwmRange(new PwmControl.PwmRange(500, 2500));
        wrist_servo_left = hardwareMap.get(ServoImplEx.class, "wrist_servo_left");
            wrist_servo_left.setPwmRange(new PwmControl.PwmRange(500, 2500));
        elbow_servo_left = hardwareMap.get(ServoImplEx.class, "elbow_servo_left");
            elbow_servo_left.setPwmRange(new PwmControl.PwmRange(ELBOW_LEFT_IN, ELBOW_LEFT_OUT));
        elbow_servo_right = hardwareMap.get(ServoImplEx.class, "elbow_servo_right");
            elbow_servo_right.setPwmRange(new PwmControl.PwmRange(ELBOW_RIGHT_OUT, ELBOW_RIGHT_IN));
    }
    public void setWrist(double positioning_angle, double turret_angle){
        current_positioning_angle = positioning_angle;
        current_turret_angle = turret_angle;
        wrist_servo_left.setPosition((positioning_angle + turret_angle)/(2* WRIST_SCALE_FACTOR) + LEFT_WRIST_OFFSET);
        wrist_servo_left.setPosition((positioning_angle - turret_angle)/(2* WRIST_SCALE_FACTOR) + RIGHT_WRIST_OFFSET);
    }
    public void rotate_turret(double turret_angle){
        setWrist(current_positioning_angle, turret_angle);
    }
    public void rotate_positioning(double positioning_angle){
        setWrist(positioning_angle, current_turret_angle);
    }
    public void grab(){
        claw_servo.setPosition(0);
    }

    public void open(){
        claw_servo.setPosition(0.4);
    }
    public void toggleGrab(){
        if (claw_servo.getPosition() > 0.5) {
            open();
        } else {
            grab();
        }
    }
    public void toReadyGrabPos() {
        elbow_servo_left.setPosition(0.85);
        elbow_servo_right.setPosition(0.15);
        setWrist(0, current_turret_angle);
    }

    public void toTransferPos() {
        elbow_servo_left.setPosition(0);
        elbow_servo_right.setPosition(1);
        setWrist(PI, 0);
    }


    public void toGrabPos() {
        elbow_servo_left.setPosition(1);
        elbow_servo_right.setPosition(0);
        setWrist(0, current_turret_angle);
    }
}
