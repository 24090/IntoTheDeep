package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.MechanismActions;

@TeleOp(group = "testing", name = "Intake Testing")
public class IntakeTesting extends LinearOpMode {
    MechanismActions actions;
    Servo s0 = hardwareMap.get(Servo.class, "cs0");
    Servo s1 = hardwareMap.get(Servo.class, "cs1");
    Servo s2 = hardwareMap.get(Servo.class, "cs2");
    Servo s3 = hardwareMap.get(Servo.class, "cs3");
    DcMotor m0 = hardwareMap.get(DcMotor.class, "cm0");

    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.a) {
                s1.setPosition(s1.getPosition()+0.01);
            }
            if (gamepad1.y) {
                s1.setPosition(s1.getPosition()-0.01);
            }
            if (gamepad1.dpad_up) {
                s2.setPosition(s2.getPosition()+0.01);
            }
            if (gamepad1.dpad_down) {
                s2.setPosition(s2.getPosition()-0.01);
            }
            if (gamepad1.dpad_right) {
                s3.setPosition(s3.getPosition()+0.01);
            }
            if (gamepad1.dpad_left) {
                s3.setPosition(s3.getPosition()-0.01);
            }
            if (gamepad1.b) {
                actions.setSlidePosition(m0, m0.getCurrentPosition()+10);
            }
            if (gamepad1.x) {
                actions.setSlidePosition(m0, m0.getCurrentPosition()-10);
            }
            telemetry.addData("s0", s0.getPosition());
            telemetry.addData("s1", s1.getPosition());
            telemetry.addData("s2", s2.getPosition());
            telemetry.addData("s3", s3.getPosition());
            telemetry.addData("m0", m0.getCurrentPosition());
            telemetry.update();
        }
    }


}
