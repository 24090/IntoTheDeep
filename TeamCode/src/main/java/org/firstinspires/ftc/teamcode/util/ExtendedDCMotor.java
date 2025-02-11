package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ExtendedDCMotor {
    DcMotorEx motor;
    HardwareMap hardwareMap;
    PIDMotorcontrol pidController;
    double minBoundPos;
    double maxBoundPos;
    double minBoundVelo;
    double maxBoundVelo;
    public double kP;
    public double kD;
    public double kI;
    double ticksPerRev;
    public ExtendedDCMotor(DcMotorEx motorEx, HardwareMap hwMap, String deviceName, boolean RUN_USING_ENCODER, boolean brakes, double maxPos, double minPos, double maxVelo, double minVelo, double proportional, double deriv, double integral, double tickPerRot, boolean isReversed){
        minBoundPos = minPos;
        maxBoundPos = maxPos;
        minBoundVelo = minVelo;
        maxBoundVelo = maxPos;
        kP = proportional;
        kD = deriv;
        kI = integral;
        ticksPerRev = tickPerRot;
        motor = motorEx;
        hardwareMap = hwMap;
        motor = hardwareMap.get(DcMotorEx.class, deviceName);
        if (RUN_USING_ENCODER){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (brakes){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (isReversed){
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        pidController = new PIDMotorcontrol(kP,kD,kI, motor);
    }
    public void setSpeed(double speed){
        if ((minBoundVelo < speed) && (maxBoundVelo > speed)){
            motor.setPower(pidController.calculateVeloPID(speed));
        }
    }
    public void goToPos(double pos){
        if ((minBoundPos < pos) && (maxBoundPos > pos)){
            motor.setPower(pidController.calculatePosPID(pos));
        }
    }
    public void brake(){
        if (motor.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE){
            motor.setPower(0);
        }
        else{
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
}