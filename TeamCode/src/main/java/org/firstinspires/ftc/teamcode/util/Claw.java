package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    LinearOpMode opMode;
    HardwareMap hardwareMap;
    Servo clawServo;
    Servo wrist1Servo;
    Servo wrist2Servo;
    Servo elbow1Servo;
    Servo elbow2Servo;
    public Claw(LinearOpMode opMode, HardwareMap hardwareMap){
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        clawServo = this.hardwareMap.get(Servo.class, "clawServo");
        wrist1Servo = this.hardwareMap.get(Servo.class, "wrist1Servo");
        wrist2Servo = this.hardwareMap.get(Servo.class, "wrist2Servo");
        elbow1Servo = this.hardwareMap.get(Servo.class, "elbow1Servo");
        elbow2Servo = this.hardwareMap.get(Servo.class, "elbow2Servo");
    }
    public void clawServoSetPosition(double value){
        clawServo.setPosition(value);
    }
    public void wrist1ServoSetPosition(double value){
        wrist1Servo.setPosition(value);
    }
    public void wrist2ServoSetPosition(double value){
        wrist2Servo.setPosition(value);
    }
    public void elbow1ServoSetPosition(double value){
        elbow1Servo.setPosition(value);
    }
    public void elbow2ServoSetPosition(double value){
        elbow2Servo.setPosition(value);
    }
}
