package org.firstinspires.ftc.teamcode.OldRobot.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "testing", name = "Set Servos")
public class SetServos  extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo intake_servo_a1 = hardwareMap.get(Servo.class, "intake_servo_a1");
        Servo intake_servo_a2 = hardwareMap.get(Servo.class, "intake_servo_a2");
        Servo intake_servo_b =hardwareMap.get(Servo.class, "intake_servo_b");
        Servo outtake_servo = hardwareMap.get(Servo.class, "outtake_servo");
        // All servos in "down" position
        intake_servo_a1.setPosition(0);
        intake_servo_a2.setPosition(1);
        outtake_servo.setPosition(1);
        waitForStart();
        // All servos in "up" position
        intake_servo_a1.setPosition(1);
        intake_servo_a2.setPosition(0);
        outtake_servo.setPosition(0);
        while (opModeIsActive()){
            // run servos back and forth
            intake_servo_b.setPosition(0);
            wait(1000);
            intake_servo_b.setPosition(1);
            wait(1000);
        }
    }
}