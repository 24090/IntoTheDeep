package org.firstinspires.ftc.teamcode.NewRobot.Util.IntegratedMechs;

import org.firstinspires.ftc.teamcode.NewRobot.Util.Servo.Servos;

public class Hand {
    Servos claw;
    Servos rotWrist;
    Servos latWrist;
    double input;
    public enum HandPos{INGRAB, READYCLIP, CLIPPED, STANDBY}
    HandPos handPos = HandPos.STANDBY;
    public Hand(Servos claw, Servos rotWrist, Servos latWrist){
        this.claw = claw;
        this.rotWrist = rotWrist;
        this.latWrist = latWrist;
    }
    public void cycleState(){
        switch (handPos){
            case CLIPPED:
                handPos = HandPos.STANDBY;
            case STANDBY:
                handPos = HandPos.INGRAB;
            case READYCLIP:
                handPos = HandPos.READYCLIP;
            case INGRAB:
                handPos = HandPos.CLIPPED;
        }
    }
    public void actState(HandPos handPos) {
        switch (handPos) {
            case CLIPPED:
                claw.setPosition(0);
                rotWrist.setPosInDegrees(0);
                latWrist.setPosInDegrees(2);
            case STANDBY:
                claw.setPosition(0);
                rotWrist.setPosInDegrees(0);
                latWrist.setPosInDegrees(90);
            case READYCLIP:
                claw.setPosition(1);
                rotWrist.setPosInDegrees(0);
                latWrist.setPosInDegrees(2);
            case INGRAB:
                claw.setPosition(0);
                rotWrist.setPosition(input);
                latWrist.setPosition(-90);
        }
    }
    public void changeRotWristInput(double input){
        this.input = input;
    }
}
