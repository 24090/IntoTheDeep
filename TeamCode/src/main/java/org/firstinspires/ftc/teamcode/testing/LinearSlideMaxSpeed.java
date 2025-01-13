package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Outtake;

public class LinearSlideMaxSpeed extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake_motor");
        DcMotor outtake = hardwareMap.get(DcMotor.class, "outtake_slide_motor");
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.a) {
                intake.setPower(1);
            } else if (gamepad1.b) {
                intake.setPower(-1);
            } else if (gamepad1.x) {
                outtake.setPower(1);
            } else if (gamepad1.y) {
                outtake.setPower(-1);
            } else {
                intake.setPower(0);
                outtake.setPower(0);
            }
        }
    }
}
