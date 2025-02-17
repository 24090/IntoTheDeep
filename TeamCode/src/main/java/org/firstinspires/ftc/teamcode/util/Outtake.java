package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.linearslides.LinearSlide;
import org.firstinspires.ftc.teamcode.util.linearslides.OuttakeSlide;

import java.util.Objects;

public class Outtake {

    OuttakeSlide outtakeSlideRight;
    OuttakeSlide outtakeSLideLeft;
    Servo servo0;
    //left arm (looking from the back)
    Servo servo1;
    Servo servo2;

    public String outtake_state = "closed";
    public Outtake(HardwareMap hwmap) {
        this.servo0 = hwmap.get(Servo.class, "es0");
        this.servo1 = hwmap.get(Servo.class, "cs4");
        this.servo2 = hwmap.get(Servo.class, "cs5");
        outtakeSlideRight = new OuttakeSlide(hwmap.get(DcMotor.class, "cm1"));
        outtakeSLideLeft = new OuttakeSlide(hwmap.get(DcMotor.class, "cm2"));
        outtakeSLideLeft.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void open() {
        if(Objects.equals(outtake_state, "closed") || Objects.equals(outtake_state, "scoring") || Objects.equals(outtake_state, "transfer")) {
            outtake_state = "grabbing";

            outtakeSlideRight.goTo(100, 50);
            outtakeSLideLeft.goTo(100, 50);
            outtakeSlideRight.waitForMovement();
            outtakeSLideLeft.waitForMovement();

            servo0.setPosition(1);
            servo1.setPosition(0.66);
            servo2.setPosition(-0.66);
        } else if(Objects.equals(outtake_state, "grabbing")) {
            outtake_state = "scoring";

            outtakeSlideRight.goTo(1200, 50);
            outtakeSLideLeft.goTo(1200, 50);
            outtakeSlideRight.waitForMovement();
            outtakeSLideLeft.waitForMovement();

            servo0.setPosition(1);
            servo1.setPosition(0.33);
            servo2.setPosition(-0.33);
        }
    }

    public void close() {
        outtake_state = "closed";

        outtakeSlideRight.moveDown();
        outtakeSLideLeft.moveDown();

        servo0.setPosition(0);
        servo1.setPosition(1);
        servo2.setPosition(-1);
    }

    public void grab() {
        if (Objects.equals(outtake_state, "grabbing") || Objects.equals(outtake_state, "scoring")) {
            if (servo0.getPosition() == 0) {
                servo0.setPosition(1);
            } else {
                servo0.setPosition(0);
            }
        }
    }

    public void transferPos() {
        outtake_state = "transfer";

        outtakeSlideRight.moveDown();
        outtakeSLideLeft.moveDown();

        servo0.setPosition(1);
        servo1.setPosition(0.8);
        servo2.setPosition(-0.8);
    }

    public void activateClaw() {
        if (servo0.getPosition() == 0) {
            servo0.setPosition(1);
        } else {
            servo0.setPosition(0);
        }
    }

    public void score() {
        if (Objects.equals(outtake_state, "scoring")) {
            outtakeSlideRight.goTo(800, 50);
            outtakeSLideLeft.goTo(800, 50);

            outtakeSlideRight.waitForMovement();
            outtakeSLideLeft.waitForMovement();
        }
    }
}
