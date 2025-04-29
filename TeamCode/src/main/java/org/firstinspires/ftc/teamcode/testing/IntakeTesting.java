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
                intake.claw.toGrabPos();
            } else if (gamepad1.dpad_up){
                intake.claw.toTransferPos();
            }
            if (gamepad1.dpad_left) {
                intake.linear_slide.moveIn();
            } else if (gamepad1.dpad_right) {
                intake.linear_slide.moveOut();
            }
            if (gamepad1.y){
                intake.claw.grab();
            }else if (gamepad1.a){
                intake.claw.open();
            }
            telemetry.addData("pos", intake.linear_slide.getPosition());
            telemetry.update();
        }
        intake.linear_slide.stopThread();
    }

}
