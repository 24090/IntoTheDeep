package org.firstinspires.ftc.teamcode.util.mechanisms.intake;

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

    public double turret_angle;
    public static double CLAW_OPEN = 0;
    public static double CLAW_LOOSE = 0.6;
    public static double CLAW_GRAB = 1;

    public static double ELBOW_LEFT_IN = 0.52;
    public static double ELBOW_LEFT_READY = 0.66;
    public static double ELBOW_LEFT_OUT = 0.98;

    public static double ELBOW_RIGHT_IN = 0.82;
    public static double ELBOW_RIGHT_READY = 0.75;
    public static double ELBOW_RIGHT_OUT = 0.6;

    public static double WRIST_LEFT_IN = 0.93;
    public static double WRIST_LEFT_OUT_0 = 0.35;
    public static double WRIST_LEFT_OUT_90 = 0.6;
    public static double WRIST_LEFT_READY_0 = 0.42;
    public static double WRIST_LEFT_READY_90 = 0.67;

    public static double WRIST_RIGHT_IN = 0.07;
    public static double WRIST_RIGHT_OUT_0 = 0.65;
    public static double WRIST_RIGHT_OUT_90 = 0.9;
    public static double WRIST_RIGHT_READY_0 = 0.58;
    public static double WRIST_RIGHT_READY_90 = 0.83;

    public Claw(HardwareMap hardwareMap){
        claw_servo = hardwareMap.get(ServoImplEx.class, "claw_servo");
            claw_servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        wrist_servo_right = hardwareMap.get(ServoImplEx.class, "wrist_servo_right");
            wrist_servo_right.setPwmRange(new PwmControl.PwmRange(500, 2500));
        wrist_servo_left = hardwareMap.get(ServoImplEx.class, "wrist_servo_left");
            wrist_servo_left.setPwmRange(new PwmControl.PwmRange(500, 2500));
        elbow_servo_left = hardwareMap.get(ServoImplEx.class, "elbow_servo_left");
            elbow_servo_left.setPwmRange(new PwmControl.PwmRange(500, 2500));
        elbow_servo_right = hardwareMap.get(ServoImplEx.class, "elbow_servo_right");
            elbow_servo_right.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }


    public void wrist_ready(){
        if (turret_angle == PI/2){
            wrist_servo_left.setPosition(WRIST_LEFT_READY_90);
            wrist_servo_right.setPosition(WRIST_RIGHT_READY_90);
        } else if (turret_angle == 0){
            wrist_servo_left.setPosition(WRIST_LEFT_READY_0);
            wrist_servo_right.setPosition(WRIST_RIGHT_READY_0);
        } else {
            double rots = (1+(turret_angle/(PI/2))%1)%1;
            wrist_servo_left.setPosition((1-rots) * WRIST_LEFT_READY_0 + rots * WRIST_LEFT_READY_90);
            wrist_servo_right.setPosition((1-rots) * WRIST_RIGHT_READY_0 + rots * WRIST_RIGHT_READY_90);
        }
    }

    public void wrist_grab(){
        if (turret_angle == PI/2){
            wrist_servo_left.setPosition(WRIST_LEFT_OUT_90);
            wrist_servo_right.setPosition(WRIST_RIGHT_OUT_90);
        } else if (turret_angle == 0){
            wrist_servo_left.setPosition(WRIST_LEFT_OUT_0);
            wrist_servo_right.setPosition(WRIST_RIGHT_OUT_0);
        } else {
            double t = (1+(turret_angle/(PI/2))%1)%1;
            wrist_servo_left.setPosition((1-t) * WRIST_LEFT_OUT_0 + t * WRIST_LEFT_OUT_90);
            wrist_servo_right.setPosition((1-t) * WRIST_RIGHT_OUT_0 + t * WRIST_RIGHT_OUT_90);
        }
    }

    public void grab(){
        claw_servo.setPosition(CLAW_GRAB);
    }
    public void loose(){
        claw_servo.setPosition(CLAW_LOOSE);
    }
    public void open(){
        claw_servo.setPosition(CLAW_OPEN);
    }

    public void toggleGrab(){
        if (claw_servo.getPosition() > 0.5) {
            open();
        } else {
            grab();
        }
    }

    public void toReadyGrabPos(double angle) {
        elbow_servo_left.setPosition(ELBOW_LEFT_READY);
        elbow_servo_right.setPosition(ELBOW_RIGHT_READY);
        turret_angle = angle;
        wrist_ready();
    }
    public void toReadyGrabPos() {
        toReadyGrabPos(0);
    }
    public void toTransferPos() {
        elbow_servo_left.setPosition(ELBOW_LEFT_IN);
        elbow_servo_right.setPosition(ELBOW_RIGHT_IN);
        wrist_servo_left.setPosition(WRIST_LEFT_IN-0.05);
        wrist_servo_right.setPosition(WRIST_RIGHT_IN+0.05);
    }


    public void toGrabPos() {
        elbow_servo_left.setPosition(ELBOW_LEFT_OUT);
        elbow_servo_right.setPosition(ELBOW_RIGHT_OUT);
        wrist_grab();
        open();
    }
}
