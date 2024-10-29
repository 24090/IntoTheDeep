package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(group = "testing", name = "Arm Testing")
public class ArmTesting extends LinearOpMode {
    public void runOpMode(){
        Servo intake_servo_a1 = hardwareMap.get(Servo.class, "intake_servo_a1");
        Servo intake_servo_a2 = hardwareMap.get(Servo.class, "intake_servo_a2");
        Servo intake_servo_b = hardwareMap.get(Servo.class, "intake_servo_b");
        intake_servo_a1.scaleRange(0, 1);
        intake_servo_a2.scaleRange(0, 1);
        DcMotor intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
        waitForStart();
        while (opModeIsActive()){
            intake_servo_a1.setPosition(0);
            intake_servo_a2.setPosition(0);
            sleep((long) 1000);
            intake_servo_b.setPosition(0);
            sleep((long) 1000);
            intake_servo_a1.setPosition(1);
            intake_servo_a2.setPosition(1);
            sleep((long) 1000);
            intake_servo_b.setPosition(300);
            sleep((long) 1000);
        }
    }


}
