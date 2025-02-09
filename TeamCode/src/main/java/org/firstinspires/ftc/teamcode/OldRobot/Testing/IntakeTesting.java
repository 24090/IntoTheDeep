package org.firstinspires.ftc.teamcode.OldRobot.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OldRobot.Util.Intake;

@TeleOp(group = "testing", name = "Intake Testing")
public class IntakeTesting extends LinearOpMode {
    Intake intake;
    public void runOpMode() throws InterruptedException {
        intake = new Intake(
                hardwareMap.get(Servo.class, "intake_servo_a1"),
                hardwareMap.get(Servo.class, "intake_servo_a2"),
                hardwareMap.get(Servo.class, "intake_servo_b"),
                hardwareMap.get(DcMotor.class, "intake_motor")
        );
        waitForStart();
        while (opModeIsActive()){
//            intake_servo_b.setPosition(0.5);
//            intake_servo_a2.setPosition(0.7);
//            intake_servo_a1.setPosition(0);
//            intake_servo_a2.setPosition(0);
//            sleep((long) 1000);
//            intake_servo_b.setPosition(0);
//            sleep((long) 1000);
//            intake_servo_b.setPosition(0.5);
//            intake_servo_a1.setPosition(0.7);
//            sleep((long) 1000);
//            intake_servo_b.setPosition(1);
//            sleep((long) 1000);
            if (gamepad1.dpad_left){
                intake.moveDown();
            } else if (gamepad1.dpad_right){
                intake.moveUp();
            }
            if (gamepad1.y){
                intake.grab();
            } else if (gamepad1.x){
                intake.hold();
            } else if (gamepad1.a){
                intake.release();
            }
            if (gamepad1.right_bumper){
                intake.transferSample();
            } else if (gamepad1.left_bumper){
                intake.slideOut();
            } else {
                intake.slideStop();
            }
            telemetry.addData("pos", intake.linear_slide.motor.getCurrentPosition());
            telemetry.update();
        }
    }


}
