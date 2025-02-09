package org.firstinspires.ftc.teamcode.NewRobot.Util.IntegratedMechs;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.NewRobot.Util.LinearSlides.NewIntakeSlide;
import org.firstinspires.ftc.teamcode.NewRobot.Util.Servo.Servos;

public class Arm {
    Hand hand;
    Servos elbow;
    NewIntakeSlide intakeSlide;
    public enum ArmPos{INGRAB, READYCLIP, STANDBY}
    ArmPos armPos = ArmPos.STANDBY;
    public Arm(HardwareMap hardwareMap, Hand hand, Servos elbow, NewIntakeSlide intakeSlide){
        this.intakeSlide = intakeSlide;
        this.hand = hand;
        this.elbow = elbow;
    }
    public void cycleState(){
        switch (armPos){
            case STANDBY:
                armPos = ArmPos.INGRAB;
            case INGRAB:
                armPos = ArmPos.READYCLIP;
            case READYCLIP:
                armPos = ArmPos.STANDBY;
        }
    }
    public void actState(){

    }

}
