package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.mechanisms.outtake.OuttakeClaw;

@TeleOp(group = "testing", name = "Outtake Claw Testing")
public class OuttakeClawTesting extends LinearOpMode {
    OuttakeClaw claw;
    public void runOpMode() throws InterruptedException {
        claw = new OuttakeClaw(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.dpad_left){
                claw.open();
            } else if (gamepad1.dpad_right){
                claw.grab();
            }
            if (gamepad1.x){
                claw.toTransferPos();
            }
            if (gamepad1.y){
                claw.toSamplePos();
            }
            if (gamepad1.y){
                claw.readySpecimen();
            }
            telemetry.update();
        }
    }


}
