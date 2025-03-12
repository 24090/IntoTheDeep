package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "testing", name = "Outtake Testing")
public class OuttakeTesting extends LinearOpMode {
    //MechanismActions actions;
    Servo s0 = hardwareMap.get(Servo.class, "es0");
    Servo s1 = hardwareMap.get(Servo.class, "cs4");
    Servo s2 = hardwareMap.get(Servo.class, "cs5");
    DcMotor m0 = hardwareMap.get(DcMotor.class, "cm1");
    DcMotor m1 = hardwareMap.get(DcMotor.class, "cm2");

    public void runOpMode() throws InterruptedException {

        if (gamepad1.a) {
            s0.setPosition(s0.getPosition()+0.01);
        }
        if (gamepad1.y) {
            s0.setPosition(s0.getPosition()-0.01);
        }
        if (gamepad1.b) {
            s1.setPosition(s0.getPosition()+0.01);
            s2.setPosition(s0.getPosition()-0.01);
        }
        if (gamepad1.x) {
            s1.setPosition(s0.getPosition()-0.01);
            s2.setPosition(s0.getPosition()+0.01);
        }
        if (gamepad1.dpad_up) {
            //actions.setSlidePosition(m0, m0.getCurrentPosition()+10);
            //actions.setSlidePosition(m1, m1.getCurrentPosition()-10);
        }
        if (gamepad1.dpad_down) {
            //actions.setSlidePosition(m0, m0.getCurrentPosition()-10);
            //actions.setSlidePosition(m1, m1.getCurrentPosition()+10);
        }

        telemetry.addData("s0", s0.getPosition());
        telemetry.addData("s1", s1.getPosition());
        telemetry.addData("s2", s2.getPosition());
        telemetry.addData("s0", s0.getPosition());
        telemetry.addData("s0", s0.getPosition());
        telemetry.update();
    }


}
