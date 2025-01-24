package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "testing", name = "ServoATesting")
public class ServoATesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo intake_servo_a1 = hardwareMap.get(Servo.class, "intake_servo_a1");
        Servo intake_servo_a2 = hardwareMap.get(Servo.class, "intake_servo_a2");
        double v = 0;
        waitForStart();
        while (opModeIsActive()){
            intake_servo_a1.setPosition(v);
            intake_servo_a2.setPosition(1-v);
            if (gamepad1.left_bumper){
                v -= 0.05;
            } else if (gamepad1.right_bumper){
                v += 0.05;
            } else if (gamepad1.dpad_left){
                v -= 0.01;
            } else if (gamepad1.dpad_right){
                v += 0.05;
            }
            sleep(500);
            telemetry.addData("pos", v);
            telemetry.update();
        }


    }
}
