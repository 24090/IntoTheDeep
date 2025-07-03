package org.firstinspires.ftc.teamcode.util.mechanisms.intake;

import static java.lang.Math.PI;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Claw {
    ServoImplEx claw_servo;
    ServoImplEx wrist_servo_left;
    ServoImplEx wrist_servo_right;
    ServoImplEx elbow_servo_left;
    ServoImplEx elbow_servo_right;

    public NormalizedColorSensor color_sensor;
    public enum ColorSensorOut {
        RED,
        YELLOW,
        BLUE,
        NONE
    }
    public double turret_angle;
    public static double CLAW_OPEN = 0.82;
    public static double CLAW_LOOSE = 0.54;
    public static double CLAW_GRAB = 0.54;

    public static double ELBOW_LEFT_IN = 0.47;
    public static double ELBOW_LEFT_READY = 0.66;
    public static double ELBOW_LEFT_OUT = 0.77;

    public static double ELBOW_RIGHT_IN = 0.82;
    public static double ELBOW_RIGHT_READY = 0.75;
    public static double ELBOW_RIGHT_OUT = 0.46;

    public static double WRIST_LEFT_IN = 1;
    public static double WRIST_LEFT_OUT_0 = 0;
    public static double WRIST_LEFT_OUT_90 = 0.53;
    public static double WRIST_LEFT_READY_0 = 0.35;
    public static double WRIST_LEFT_READY_90 = 0.65;

    public static double WRIST_RIGHT_IN = 0;
    public static double WRIST_RIGHT_OUT_0 = 0.65;
    public static double WRIST_RIGHT_OUT_90 = 0.89;
    public static double WRIST_RIGHT_READY_0 = 0.59;
    public static double WRIST_RIGHT_READY_90 = 0.77;

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
        color_sensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
    }

    public void wrist_ready(){
        turret_angle = (turret_angle%(PI) + PI)%(PI);
        if (Math.abs(turret_angle - PI/2) < PI/4){
            wrist_servo_left.setPosition(WRIST_LEFT_READY_90);
            wrist_servo_right.setPosition(WRIST_RIGHT_READY_90);
        } else {
            wrist_servo_left.setPosition(WRIST_LEFT_READY_0);
            wrist_servo_right.setPosition(WRIST_RIGHT_READY_0);
        }
    }

    public void wrist_grab(){
        turret_angle = (turret_angle%(PI) + PI)%(PI);
        if (Math.abs(turret_angle - PI/2) < PI/4){
            wrist_servo_left.setPosition(WRIST_LEFT_OUT_90);
            wrist_servo_right.setPosition(WRIST_RIGHT_OUT_90);
        } else {
            wrist_servo_left.setPosition(WRIST_LEFT_OUT_0);
            wrist_servo_right.setPosition(WRIST_RIGHT_OUT_0);
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
    public void toReadyGrabPos() {
        toReadyGrabPos(turret_angle);
    }
    public void toReadyGrabPos(double angle) {
        elbow_servo_left.setPosition(ELBOW_LEFT_READY);
        elbow_servo_right.setPosition(ELBOW_RIGHT_READY);
        turret_angle = angle;
        wrist_ready();
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

    public ColorSensorOut getSensedColor(){
        int color_int =  color_sensor.getNormalizedColors().toColor();
        float[] hsv = new float[3];
        Color.colorToHSV(color_int, hsv);
        if (hsv[1] < 0.3){
            return ColorSensorOut.NONE;
        }
        if (hsv[0] <= 45 || hsv[0] > 290) {
            return ColorSensorOut.RED;
        }
        if (hsv[0] >= 45 && hsv[0] <= 160) {
            return ColorSensorOut.YELLOW;
        }
        if (hsv[0] >= 200 && hsv[0] <= 290) {
            return ColorSensorOut.BLUE;
        }
        return ColorSensorOut.NONE;
    }
    public void disableClaw(){
        claw_servo.setPwmDisable();
        wrist_servo_left.setPwmDisable();
        wrist_servo_right.setPwmDisable();
        elbow_servo_right.setPwmDisable();
        elbow_servo_left.setPwmDisable();
    }
}
