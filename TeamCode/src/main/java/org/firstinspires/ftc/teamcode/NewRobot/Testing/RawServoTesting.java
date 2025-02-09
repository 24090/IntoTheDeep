package org.firstinspires.ftc.teamcode.NewRobot.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class RawServoTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo1 = hardwareMap.get(Servo.class, "servo3");
        Servo servo2 = hardwareMap.get(Servo.class, "servo5");
        Servo servo3 = hardwareMap.get(Servo.class, "servo6");
        double k = 0;
        double i = 0;
        double l = 0;
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.dpad_up){
                k = k+0.03;
            }
            if (gamepad1.dpad_down){
                k= k-0.03;
            }
            if (gamepad1.dpad_right){
                l = l+0.03;
            }
            if (gamepad1.dpad_left){
                l= l-0.03;
            }
            if (gamepad1.y){
                i = i+0.03;
            }
            if (gamepad1.a){
                i= i-0.03;
            }
            if (k> 1){
                k=1;
            }
            if (k< 0){
                k=0;
            }
            if (l> 1){
                l=1;
            }
            if (l< 0){
                l=0;
            }
            if (i> 1){
                i=1;
            }
            if (i< 0){
                i=0;
            }
            sleep(50);
            servo1.setPosition(k);
            servo2.setPosition(i);
            servo3.setPosition(l);
            telemetry.addData("pos1", k);
            telemetry.addData("pos2", i);
            telemetry.addData("pos3", l);
            telemetry.update();
        }
    }
}
