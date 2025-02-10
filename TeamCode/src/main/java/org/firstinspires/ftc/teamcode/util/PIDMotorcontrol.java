package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PIDMotorcontrol {
    double kP;
    double kD;
    double kI;
    double lastError = 0;
    DcMotorEx motorName;
    double lastTime = System.currentTimeMillis();
    double integralTerm = 0;
    double lastReference = 0;
    double maxIntegralTerm = 0;
    public PIDMotorcontrol(double propValue, double derivValue, double intValue, DcMotorEx motor){
        kP = propValue;
        kD = derivValue;
        kI = intValue;
        motorName = motor;
    }
    public double calculatePosPID(double reference){
        double error = reference - motorName.getCurrentPosition();
        double timeFrame = System.currentTimeMillis() - lastTime;
        double proportionalTerm = kP * error;
        double derivativeTerm = kD * (error-lastError/timeFrame);
        integralTerm = kI * (integralTerm + error * timeFrame);
        if (integralTerm > maxIntegralTerm) {
            integralTerm = maxIntegralTerm;
        }
        if (integralTerm < -maxIntegralTerm) {
            integralTerm = -maxIntegralTerm;
        }
        if (reference != lastReference) {
            integralTerm = 0;
        }
        lastReference = reference;
        lastError = error;
        lastTime = System.currentTimeMillis();
        return proportionalTerm + derivativeTerm + integralTerm;
    }
    public double calculateVeloPID(double reference){
        double error = reference - motorName.getVelocity();
        double timeFrame = System.currentTimeMillis() - lastTime;
        double proportionalTerm = kP * error;
        double derivativeTerm = kD * (error-lastError/timeFrame);
        integralTerm = kI * (integralTerm + error * timeFrame);
        if (integralTerm > maxIntegralTerm) {
            integralTerm = maxIntegralTerm;
        }
        if (integralTerm < -maxIntegralTerm) {
            integralTerm = -maxIntegralTerm;
        }
        if (reference != lastReference) {
            integralTerm = 0;
        }
        lastReference = reference;
        lastError = error;
        lastTime = System.currentTimeMillis();
        return proportionalTerm + derivativeTerm + integralTerm;
    }
}
