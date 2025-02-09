package org.firstinspires.ftc.teamcode.NewRobot.Util.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servos {
    public double lowerDegreeBound;
    public double upperDegreeBound;
    public HardwareMap hwMap;
    Servo servo;
    public double initPos;
    public String servoName;
    public double lowerBound;
    public double upperBound;
    public Servos(String servoName, HardwareMap hwMap, double initPos, double lowerBound, double upperBound, double lowerDegreeBound, double upperDegreeBound){
        this.lowerDegreeBound = lowerDegreeBound;
        this.upperDegreeBound = upperDegreeBound;
        this.hwMap = hwMap;
        this.initPos = initPos;
        servo = this.hwMap.get(Servo.class, servoName);
        servo.scaleRange(lowerBound, upperBound);
        servo.setPosition(initPos);

    }
    public void setPosInDegrees(double targetDegree){
        double range = Math.abs(upperDegreeBound - lowerDegreeBound);
        double inServoPos = ((targetDegree - lowerDegreeBound)/range);
        servo.setPosition(inServoPos);
    }
    public void reverse(){
        servo.setDirection(Servo.Direction.REVERSE);
    }
    public void setPosition(double position){
        servo.setPosition(position);
    }
    public double getPosition(){
        return servo.getPosition();
    }
}
