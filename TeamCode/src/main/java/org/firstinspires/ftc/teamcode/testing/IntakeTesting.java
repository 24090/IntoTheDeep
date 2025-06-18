package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Intake;

@TeleOp(group = "testing", name = "Intake Testing")
public class IntakeTesting extends LinearOpMode {
    Intake intake;
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            intake.slide.movementLoop();
            if (gamepad1.dpad_down){
                intake.claw.toReadyGrabPos();
            } else if (gamepad1.dpad_up){
                intake.claw.toTransferPos();
            }
            if (gamepad1.dpad_left) {
                intake.slide.moveIn();
            } else if (gamepad1.dpad_right) {
                intake.slide.moveOut();
            }
            if (gamepad1.y){
                intake.claw.grab();
            }else if (gamepad1.a){
                intake.claw.open();
            }
            telemetry.addData("pos", intake.slide.getPosition());
            telemetry.update();
        }
    }

}
