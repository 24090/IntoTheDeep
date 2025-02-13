package org.firstinspires.ftc.teamcode.util;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.linearslides.IntakeSlide;

import java.util.Objects;

public class Intake {
    MechanismActions actions;
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

        retractSlide();
        closeArm();
    }

    private void openArm() {
        if (Objects.equals(intake_state, "open")) {
            servo0.setPosition(0);
        } else {
            servo0.setPosition(1);
        }
        servo1.setPosition(0.57);
        servo2.setPosition(0.6);
        servo3.setPosition(0.8);
    }

    private void closeArm() {
        servo0.setPosition(0);
        servo1.setPosition(0);
        servo2.setPosition(0);
        servo3.setPosition(0);
    }

    private void retractSlide() {
        actions.setSlidePosition(motor0, 1300);
    }
    private void extendSlide() {
        actions.setSlidePosition(motor0, 0);
    }
}
