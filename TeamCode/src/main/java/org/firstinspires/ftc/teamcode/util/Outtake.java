package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.linearslides.LinearSlide;
import org.firstinspires.ftc.teamcode.util.linearslides.OuttakeSlide;

import java.util.Objects;

public class Outtake {

    MechanismActions actions = new MechanismActions();
    Servo servo0;
    //left arm (looking from the back)
    Servo servo1;
    Servo servo2;
    DcMotor motor0;
    DcMotor motor1;

    public String outtake_state = "closed";
    public Outtake(Servo servo0, Servo servo1, Servo servo2, DcMotor motor0, DcMotor motor1) {
        this.servo0 = servo0;
        this.servo1 = servo1;
        this.servo2 = servo2;
        this.motor0 = motor0;
        this.motor1 = motor1;
    }

    public void open() {
        if(Objects.equals(outtake_state, "closed") || Objects.equals(outtake_state, "scoring")) {
            outtake_state = "grabbing";

            actions.setSlidePosition(motor0, 200);
            actions.setSlidePosition(motor1, -200);

            servo0.setPosition(1);
            servo1.setPosition(0.66);
            servo2.setPosition(-0.66);
        } else if(Objects.equals(outtake_state, "grabbing")) {
            outtake_state = "scoring";

            actions.setSlidePosition(motor0, 1500);
            actions.setSlidePosition(motor1, -1500);

            servo0.setPosition(0);
            servo1.setPosition(0.33);
            servo2.setPosition(-0.33);
        }
    }

    public void close() {
        outtake_state = "closed";

        actions.setSlidePosition(motor0, 0);
        actions.setSlidePosition(motor1, 0);

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

    public void score() {
        if (Objects.equals(outtake_state, "scoring")) {
            actions.setSlidePosition(motor0, 1000);
            actions.setSlidePosition(motor1, -1000);
        }
    }
}
