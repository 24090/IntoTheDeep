package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Outtake;

@TeleOp(group = "testing", name = "Outtake Testing")
public class OuttakeTesting extends LinearOpMode {
    Outtake outtake;
    public void runOpMode() throws InterruptedException {
        outtake = new Outtake(hardwareMap);
        outtake.slide.startThread();
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.dpad_left){
                outtake.open();
            } else if (gamepad1.dpad_right){
                outtake.close();
            }
            if (gamepad1.dpad_up){
                outtake.slide.up();
            } else if (gamepad1.dpad_down){
                outtake.down();
            }
            if (gamepad1.left_bumper){
                outtake.down();
            }
            if (gamepad1.right_bumper){
                outtake.slide.up();
                outtake.open();
            }
            telemetry.addData("pos", outtake.slide.getPosition());
            telemetry.update();
        }
        outtake.slide.stopThread();
    }


}
