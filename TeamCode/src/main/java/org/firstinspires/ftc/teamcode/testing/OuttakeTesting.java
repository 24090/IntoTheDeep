package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.mechanisms.outtake.Outtake;

@TeleOp(group = "testing", name = "Outtake Testing")
public class OuttakeTesting extends LinearOpMode {
    Outtake outtake;
    public void runOpMode() throws InterruptedException {
        outtake = new Outtake(hardwareMap);
        DcMotor outtake_motor = hardwareMap.get(DcMotor.class, "outtake_slide_right");
        waitForStart();
        while (opModeIsActive()){
            outtake.backgroundIter();
            if (gamepad1.dpad_left){
                outtake.claw.open();
            } else if (gamepad1.dpad_right){
                outtake.claw.grab();
            }
            if (gamepad1.x){
                outtake.readyTransfer();
            }
            if (gamepad1.y){
                outtake.readySample();
            }
            if (gamepad1.a){
                outtake.readySpecimen();
            }
            telemetry.addData("pos", outtake.slide.getPosition());
            telemetry.addData("power", outtake_motor.getPower());
            telemetry.update();
        }
    }


}
