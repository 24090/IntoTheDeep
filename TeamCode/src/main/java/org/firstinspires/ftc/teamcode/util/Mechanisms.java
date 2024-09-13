package org.firstinspires.ftc.teamcode.util;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mechanisms {
    HardwareMap hwMap;
    DcMotorEx armmotor;
    ExtendedDCMotor armMotor;
    public Mechanisms(HardwareMap hardware){
        hwMap = hardware;
        armMotor = new ExtendedDCMotor(armmotor,hwMap,"armMotor", false,true,99999, -99999, 1, 0, 1,0,0, 537, false);
    }
    public void setArmPos(double pos){
        armMotor.goToPos(pos);
    }

}
