package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Outtake;

@TeleOp(group = "testing", name = "Outtake Testing")
public class OuttakeTesting extends LinearOpMode {
    Outtake outtake;
    public void runOpMode() throws InterruptedException {
        outtake = new Outtake(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.dpad_left){
                outtake.open();
            } else if (gamepad1.dpad_right){
                outtake.close();
            }
            if (gamepad1.dpad_up){
                outtake.up();
            } else if (gamepad1.dpad_down){
                outtake.down();
            }
            if (gamepad1.left_bumper){
                outtake.close();
                wait(300);
                outtake.down();
            }
            if (gamepad1.right_bumper){
                outtake.up();
                outtake.open();
            }
            telemetry.addData("pos", outtake.linear_slide.getPosition());
            telemetry.update();
        }
    }


}
