package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Outtake;

import java.util.HashMap;
import java.util.Map;

@TeleOp(group = "testing", name = "Intake/Outtake State Machine Testing")
public class InOutStateMachineTesting extends LinearOpMode {
    Intake intake;
    Outtake outtake;
    Boolean has_sample;
    @Override
    public void runOpMode(){
        intake = new Intake(
                hardwareMap.get(Servo.class, "intake_servo_a1"),
                hardwareMap.get(Servo.class, "intake_servo_a2"),
                hardwareMap.get(Servo.class, "intake_servo_b"),
                hardwareMap.get(DcMotor.class, "intake_motor")
        );
        outtake = new Outtake(
                hardwareMap.get(Servo.class, "outtake_servo"),
                hardwareMap.get(DcMotor.class, "outtake_slide_motor")
        );
        while (opModeIsActive()){
            has_sample = has_sample ||  intake.stateMachine();
            has_sample = has_sample && !outtake.stateMachine();
            if (gamepad1.dpad_left){
                // intake next
            } else if (gamepad1.dpad_right){
                // intake last
            } else if (gamepad1.left_bumper) {
                // intake reset
            }
            if (gamepad1.dpad_left){
                // outtake next
            } else if (gamepad1.dpad_right){
                // outtake last
            } else if (gamepad1.left_bumper) {
                // outtake reset
            }
        }
    }
}
