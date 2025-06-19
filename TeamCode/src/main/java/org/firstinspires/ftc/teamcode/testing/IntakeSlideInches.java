package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.util.CustomActions.runBlocking;

import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Intake;

@TeleOp(group = "testing", name = "Extend Intake Slide to Inches")
public class IntakeSlideInches extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        intake.claw.toReadyGrabPos();
        double distance_inches = intake.slide.trimIn(0);
        waitForStart();
        double last_time = time;
        double dt;
        while (opModeIsActive()){
            telemetry.addLine("DPAD UP -> increase distance \n DPAD DOWN -> decrease distance \n A -> GO");
            telemetry.addData("distance (in)", distance_inches);
            telemetry.update();

            dt = last_time - time;
            last_time = time;
            if (gamepad1.dpad_up) {
                distance_inches = intake.slide.trimIn(distance_inches + 3*dt);
            } else if (gamepad1.dpad_down) {
                distance_inches = intake.slide.trimIn(distance_inches - 3*dt);
            }
            if (gamepad1.a){
                runBlocking(intake.readyGrabAction(distance_inches, 0));
            }
        }
    }
}
