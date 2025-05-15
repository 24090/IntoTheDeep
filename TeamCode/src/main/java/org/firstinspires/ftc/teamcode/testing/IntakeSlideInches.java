package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.util.customactions.RunBlocking.runBlocking;

import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.customactions.ForeverAction;
import org.firstinspires.ftc.teamcode.util.customactions.FutureAction;
import org.firstinspires.ftc.teamcode.util.linearslides.IntakeSlide;

@TeleOp(group = "testing", name = "Extend Intake Slide to Inches")
public class IntakeSlideInches extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        intake.claw.toReadyGrabPos();
//        telemetry.addData("Press X to increase distance, Press Y to decrease distance");
        telemetry.update();
        double distance_inches = intake.linear_slide.trimIn(0);
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
