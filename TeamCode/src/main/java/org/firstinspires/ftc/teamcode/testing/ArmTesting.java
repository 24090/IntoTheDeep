package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Intake;

@TeleOp(group = "testing", name = "Arm Testing")
public class ArmTesting extends LinearOpMode {
    Intake intake;
    public void runOpMode(){
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
            if (gamepad2.dpad_left){
                intake.moveDown();
            } else if (gamepad2.dpad_right){
                intake.moveUp();
            }
            if (gamepad2.y){
                intake.grab();
            } else if (gamepad2.x){
                intake.hold();
            } else if (gamepad2.a){
                intake.release();
            } else {
                intake.stop();
            }
            if (gamepad1.right_bumper){
                intake.slideIn();
            } else if (gamepad1.left_bumper){
                intake.slideOut();
            } else {
                intake.slideStop();
            }
        }
    }


}
