package org.firstinspires.ftc.teamcode.NewRobot.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NewRobot.Util.Servo.Servos;

@TeleOp
public class ServoTesting extends LinearOpMode {
    public double fromDegrees1(double input){
        return (input/90) + 0.5;
    }
    public double fromDegrees2(double input){
        return (input/165);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo1 = hardwareMap.get(Servo.class, "servo1");
        Servo servo2 = hardwareMap.get(Servo.class, "servo2");
        Servo servo3 = hardwareMap.get(Servo.class, "servo3");
        Servo servo4 = hardwareMap.get(Servo.class, "servo4");
        Servo servo5 = hardwareMap.get(Servo.class, "servo5");
        Servo servo6 = hardwareMap.get(Servo.class, "servo6");
        servo1.setPosition(0.5);
        servo2.setPosition(0.5);
        servo3.setPosition(0.5);
        servo2.scaleRange(0.24,0.88);
        servo3.scaleRange(0.17,0.93);
        servo4.scaleRange(0,0.6);
        // bound1
        // up 0.63 low 0.41
        // bound2 0.24, 0.88
        //
        // bound3
        // up 1 low 0,28
        double i = 0.5;
        double j = 0.5;
        double k = 0.0;
        double l = 0.5;
        double o = 0.5;
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.b){
                i = 0.63;
            }
            if (gamepad1.x){
                i = 0.41;
            }
            if (gamepad1.y && k<90){
                k = k + 1;
            }
            if (gamepad1.a && k>-90){
                k = k - 1;
            }
            if (gamepad1.dpad_left && j<90){
                j = j + 1;
            }
            if (gamepad1.dpad_right && j>-90){
                j = j - 1;
            }
            if (gamepad1.dpad_down && l<165){
                l = l + 1;
            }
            if (gamepad1.dpad_up && l>0){
                l = l - 1;
            }
            if (gamepad1.left_bumper){
                l = 13.5;
                k = 2;
                i=0.44;
                servo5.setPosition(0.0);
            }
            if (gamepad1.right_bumper){
                servo5.setPosition(0.4);
            }
            if (gamepad1.left_stick_y > 0 && o < 1){
                o = o+0.01;
            }
            if (gamepad1.left_stick_y < 0 && o > 0){
                o = o-0.01;
            }
            sleep(20);
            servo1.setPosition(i);
            servo2.setPosition(this.fromDegrees1(j));
            servo3.setPosition(this.fromDegrees1(k));
            servo4.setPosition(this.fromDegrees2(l));
            servo6.setPosition(o);
            telemetry.addData("servo1pos", l);
            telemetry.addData("servo3pos", o);
            telemetry.update();
        }

    }
}
