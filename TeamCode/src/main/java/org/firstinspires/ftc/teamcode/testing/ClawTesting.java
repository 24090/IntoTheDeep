package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Claw;
@TeleOp
public class ClawTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(hardwareMap);
        double r = 0;
        claw.rotateClaw(0);
        claw.toTransferPos();
        claw.open();
        waitForStart();
        double last_time = 0.0;
        while(opModeIsActive()){
            if (gamepad1.dpad_up){
                r += 2 * (last_time - time);
                claw.rotateClaw(r);
            }
            if (gamepad1.dpad_down){
                r -= 2 * (last_time - time);
                claw.rotateClaw(r);
            }
            if (gamepad1.a){
                claw.toReadyGrabPos();
            }
            if (gamepad1.b){
                claw.toTransferPos();
            }
            if (gamepad1.x){
                claw.grab();
            }
            if (gamepad1.y){
                claw.open();
            }
            telemetry.addData("rotation", r);
            telemetry.update();
            last_time = time;
        }
    }
}
