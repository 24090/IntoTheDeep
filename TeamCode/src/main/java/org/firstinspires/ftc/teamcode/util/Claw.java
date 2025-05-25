package org.firstinspires.ftc.teamcode.util;

import static java.lang.Math.PI;
import static java.lang.Math.floorMod;

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

    public static int ELBOW_LEFT_IN = 1000;
    public static int ELBOW_LEFT_OUT = 2200;
    public static int ELBOW_RIGHT_IN = 2500;
    public static int ELBOW_RIGHT_OUT = 1300;
    public static double WRIST_LEFT_IN = 1;
    public static double WRIST_LEFT_OUT_0 = 0.4;
    public static double WRIST_LEFT_OUT_180 = 0.8;
    public static double WRIST_RIGHT_IN = 0;
    public static double WRIST_RIGHT_OUT_0 = 0.6;
    public static double WRIST_RIGHT_OUT_180 = 1;

    public Claw(HardwareMap hardwareMap){
        claw_servo = hardwareMap.get(ServoImplEx.class, "claw_servo");
            claw_servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        wrist_servo_right = hardwareMap.get(ServoImplEx.class, "wrist_servo_right");
            wrist_servo_right.setPwmRange(new PwmControl.PwmRange(500, 2500));
        wrist_servo_left = hardwareMap.get(ServoImplEx.class, "wrist_servo_left");
            wrist_servo_left.setPwmRange(new PwmControl.PwmRange(500, 2500));
        elbow_servo_left = hardwareMap.get(ServoImplEx.class, "elbow_servo_left");
            elbow_servo_left.setPwmRange(new PwmControl.PwmRange(ELBOW_LEFT_IN, ELBOW_LEFT_OUT));
        elbow_servo_right = hardwareMap.get(ServoImplEx.class, "elbow_servo_right");
            elbow_servo_right.setPwmRange(new PwmControl.PwmRange(ELBOW_RIGHT_OUT, ELBOW_RIGHT_IN));
    }


    public void rotate(double turret_angle){
        double rots = (1+(turret_angle/(PI))%1)%1;
        wrist_servo_left.setPosition((1-rots) * WRIST_LEFT_OUT_0 + rots * WRIST_LEFT_OUT_180);
        wrist_servo_right.setPosition((1-rots) * WRIST_RIGHT_OUT_0 + rots * WRIST_RIGHT_OUT_180);
    }

    public void grab(){
        claw_servo.setPosition(1);
    }

    public void open(){
        claw_servo.setPosition(0);
    }

    public void toggleGrab(){
        if (claw_servo.getPosition() > 0.5) {
            open();
        } else {
            grab();
        }
    }

    public void toReadyGrabPos(double angle) {
        elbow_servo_left.setPosition(0.85);
        elbow_servo_right.setPosition(0.15);
        rotate(angle);
    }
    public void toReadyGrabPos() {
        toReadyGrabPos(0);
    }
    public void toTransferPos() {
        elbow_servo_left.setPosition(0);
        elbow_servo_right.setPosition(1);
        wrist_servo_left.setPosition(WRIST_LEFT_IN);
        wrist_servo_right.setPosition(WRIST_RIGHT_IN);
    }


    public void toGrabPos() {
        elbow_servo_left.setPosition(1);
        elbow_servo_right.setPosition(0);
    }
}
