package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Intake;

@TeleOp(group = "testing", name = "Extend Intake Slide to Inches")
public class IntakeSlideInches extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        intake.claw.toReadyGrabPos();
//        telemetry.addData("Press X to increase distance, Press Y to decrease distance");
        telemetry.update();
        double distance_inches = intake.slide.trimIn(0);
        waitForStart();
//        runBlocking(
//                new ForeverAction(
//                        new ParallelAction(
//                            new InstantAction(()->intake.linear_slide.goTo(intake.linear_slide.inToTicks(distance_inches))),
//                            new InstantAction( () -> {
//                                telemetry.addLine("Press X to increase distance, Press Y to decrease distance");
//                                telemetry.addData("Current Distance", distance_inches);
//                                telemetry.update();
//                            }),
//                        )
//                )
//        )
    }
}
