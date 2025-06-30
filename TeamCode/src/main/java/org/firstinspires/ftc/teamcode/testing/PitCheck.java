package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.util.mechanisms.outtake.Outtake;

public class PitCheck extends LinearOpMode {
    Outtake outtake;
    Intake intake;
    int runningInt = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.a && runningInt == 0){
                intake.claw.grab();
                runningInt = 1;
                while (gamepad1.a && opModeIsActive()){
                }
            }
            if (gamepad1.a && runningInt == 1){
                intake.claw.open();
                runningInt = 0;
                while (gamepad1.a && opModeIsActive()){
                }
            }
            if (gamepad1.x){
                intake.claw.disableClaw();
                telemetry.addLine("Intake Claw Functioning");
                telemetry.update();
                break;
            }
            if (gamepad1.b){
                intake.claw.disableClaw();
                telemetry.addLine("Intake Claw Functioning");
                telemetry.update();
                break;
            }
        }
        while (opModeIsActive()){
            if (gamepad1.a && runningInt == 0){
                intake.claw.grab();
                runningInt = 1;
                while (gamepad1.a && opModeIsActive()){
                }
            }
            if (gamepad1.a && runningInt == 1){
                intake.claw.open();
                runningInt = 0;
                while (gamepad1.a && opModeIsActive()){
                }
            }
            if (gamepad1.x){
                intake.claw.disableClaw();
                telemetry.addLine("Intake Claw Functioning");
                telemetry.update();
                break;
            }
            if (gamepad1.b){
                intake.claw.disableClaw();
                telemetry.addLine("Intake Claw Functioning");
                telemetry.update();
                break;
            }
        }
    }
}
