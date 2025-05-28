package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Outtake;

@TeleOp(group = "testing", name = "Hang Testing")
public class HangTesting extends LinearOpMode {
    Outtake outtake;
    public void runOpMode() throws InterruptedException {
        outtake = new Outtake(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            outtake.backgroundIter();
            if (gamepad1.x){
                outtake.readyHang();
            } else if (gamepad1.y){
                outtake.hang();
            }
            telemetry.addData("pos", outtake.slide.getPosition());
            telemetry.update();
        }
    }


}
