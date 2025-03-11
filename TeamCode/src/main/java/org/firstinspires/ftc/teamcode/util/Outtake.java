package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public Outtake(Servo servo0, Servo servo1, Servo servo2, DcMotor motor0, DcMotor motor1) {
        this.servo0 = servo0;
        this.servo1 = servo1;
        this.servo2 = servo2;

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeSlideRight = new OuttakeSlide(motor0);
        outtakeSLideLeft = new OuttakeSlide(motor1);
    }

    public void open() {
        if(Objects.equals(outtake_state, "closed") || Objects.equals(outtake_state, "scoring") || Objects.equals(outtake_state, "transfer")) {
            outtake_state = "grabbing";

            outtakeSlideRight.extendToBreaking(100, 50);
            outtakeSLideLeft.extendToBreaking(100, 50);

            servo0.setPosition(1);
            servo1.setPosition(0.66);
            servo2.setPosition(-0.66);
        } else if(Objects.equals(outtake_state, "grabbing")) {
            outtake_state = "scoring";

            outtakeSlideRight.extendToBreaking(1200, 50);
            outtakeSLideLeft.extendToBreaking(1200, 50);

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
            outtakeSlideRight.extendToBreaking(1000, 50);
            outtakeSLideLeft.extendToBreaking(1000, 50);
        }
    }
}
