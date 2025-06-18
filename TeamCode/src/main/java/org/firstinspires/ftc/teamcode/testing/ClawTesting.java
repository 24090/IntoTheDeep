package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Claw;

@TeleOp(group = "testing", name = "Claw Testing")
public class ClawTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(hardwareMap);
        double turret_angle = 0;
        double positioning_angle = 0;
        claw.toTransferPos();
        claw.open();
        waitForStart();
        double last_time = 0.0;
        while(opModeIsActive()){
            if (gamepad1.dpad_up){
                claw.turret_angle += 10 * (last_time - time);
                claw.wrist_ready();
            }
            if (gamepad1.dpad_down){
                claw.turret_angle -= 10 * (last_time - time);
                claw.wrist_ready();
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
            telemetry.addData("turret", turret_angle);
            telemetry.addData("positioning", positioning_angle);
            telemetry.update();
            last_time = time;
        }
    }
}
