package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.MechanismActions;

@TeleOp(group = "testing", name = "Intake Testing")
public class IntakeTesting extends LinearOpMode {
    Intake intake;
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);
        intake.linear_slide.startThread();
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.dpad_down){
                intake.moveDown();
            } else if (gamepad1.dpad_up){
                intake.moveUp();
            }
            if (gamepad1.dpad_left) {
                intake.linear_slide.moveIn();
            } else if (gamepad1.dpad_right) {
                intake.linear_slide.moveOut();
            }
            if (gamepad1.y){
                intake.grab();
            } else if (gamepad1.x){
                intake.hold();
            } else if (gamepad1.a){
                intake.release();
            }
            telemetry.addData("pos", intake.linear_slide.getPosition());
            telemetry.update();
        }
        intake.linear_slide.stopThread();
    }

}
