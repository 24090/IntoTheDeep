package org.firstinspires.ftc.teamcode.util;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.linearslides.LinearSlide;

import java.util.Objects;

public class Intake {

    //MechanismActions actions = new MechanismActions();
    Servo servo0;
    Servo servo1;
    Servo servo2;
    Servo servo3;
    DcMotor motor0;

    public String intake_state = "closed";

    public Intake(Servo servo0, Servo servo1, Servo servo2, Servo servo3, DcMotor motor0){
        this.servo0 = servo0;
        this.servo1 = servo1;
        this.servo2 = servo2;
        this.servo3 = servo3;
        this.motor0 = motor0;
    }

    public void grab() {
        if (Objects.equals(intake_state, "open")) {
            if (servo0.getPosition() == 0) {
                servo0.setPosition(1);
            } else {
                servo0.setPosition(0);
            }
        }
    }

    public void longIntake() {
        intake_state = "open";

        extendSlide();
        openArm();
    }

    public void shortIntake() {
        intake_state = "open";

        retractSlide();
        openArm();
    }

    public void close() {
        intake_state = "closed";

        closeArm();
        retractSlide();
    }

    public void moveWristRight() {
        if (Objects.equals(intake_state, "open")) {
            servo1.setPosition(servo1.getPosition()+0.003);
        }
    }

    public void moveWristLeft() {
        if (Objects.equals(intake_state, "open")) {
            servo1.setPosition(servo1.getPosition()-0.003);
        }
    }

    private void openArm() {
        if (Objects.equals(intake_state, "open")) {
            servo0.setPosition(0);
        } else {
            servo0.setPosition(1);
        }
        servo1.setPosition(0.57);
        servo2.setPosition(0.87);
        servo3.setPosition(0.6);
    }

    private void closeArm() {
        servo0.setPosition(0);
        servo1.setPosition(0.57);
        servo2.setPosition(0.2);
        servo3.setPosition(0);
    }

    public void transferPos() {
        intake_state = "transfer";

        actions.setSlidePosition(motor0, -500);
        servo0.setPosition(0);
        servo1.setPosition(0.57);
        servo2.setPosition(0.2);
        servo3.setPosition(0.15);
    }

    public void activateClaw() {
        if (servo0.getPosition() == 0) {
            servo0.setPosition(1);
        } else {
            servo0.setPosition(0);
        }
    }

    private void retractSlide() {
        actions.setSlidePosition(motor0, 0);
    }
    private void extendSlide() {
        actions.setSlidePosition(motor0, -1300);
    }
}
