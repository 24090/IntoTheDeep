package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.util.CustomActions.foreverAction;
import static org.firstinspires.ftc.teamcode.util.CustomActions.runBlocking;
import static org.firstinspires.ftc.teamcode.util.CustomActions.triggerAction;
import static org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions.fullTransferAction;
import static org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions.specFullTransferAction;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.util.mechanisms.outtake.Outtake;

import java.util.function.Supplier;

public class PitCheck extends LinearOpMode {
    Outtake outtake;
    Intake intake;
    int runningInt = 0;
    Action telemetryAction(String message){
        return new InstantAction(() -> telemetry.addLine(message));
    }
    Action testAction(Supplier<Action> test, String message){
        return new SequentialAction(
            new RaceAction(
                triggerAction(() -> gamepad1.back),
                foreverAction(() -> new ParallelAction(
                    test.get(),
                    new SequentialAction(
                        telemetryAction(message),
                        telemetryAction("Press BACK to Continue"),
                        new InstantAction(telemetry::update)
                    )
                ))
            ),
            triggerAction(() -> !gamepad1.back)
        );
    }
    @Override
    public void runOpMode() throws InterruptedException {
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
        waitForStart();
        // INTAKE
        // claw
        //   pickup 0
        runBlocking(new SequentialAction(
        testAction(() -> new SequentialAction(
            new InstantAction(() -> intake.claw.turret_angle = 0),
            intake.pickUpAction(),
            new SleepAction(1)
        ), "Is 0º pickup working?"),
        //   pickup 90
        testAction(() -> new SequentialAction(
                new InstantAction(() -> intake.claw.turret_angle = 0),
                intake.pickUpAction(),
                new SleepAction(1)
        ), "Is 90º pickup working?"),

        //   transfer pos
        intake.readyTransferAction(),
        testAction(NullAction::new,"Is the intake transfer position correct (hitting the bar)"),
        // slide
        testAction(() ->
            new SequentialAction(
                intake.readyGrabAction(Intake.MinDistance, 0),
                new SleepAction(2),
                intake.readyGrabAction(Intake.MaxDistance, 0),
                new SleepAction(2)
            ),
            "Is the intake slide going in and out without wiggles and jiggles?"
        ),
        // OUTTAKE
        // claw
        //  transfer spec
        outtake.readySpecTransferAction(),
        testAction(NullAction::new,"Is the outtake claw SPECIMEN transfer position correct"),
        //  transfer sample
        outtake.readyTransferAction(),
        testAction(NullAction::new,"Is the outtake claw SAMPLE transfer position correct"),
        //  score spec
        testAction(outtake::readySpecimenAction,"Is the outtake SPECIMEN score position correct (without wiggles and jiggles)"),
        //  score sample
        testAction(outtake::readySampleAction,"Is the outtake SAMPLE score position correct (without wiggles and jiggles)"),
        // I+O
        // transfer spec
        testAction(() -> new SequentialAction(
            intake.pickUpAction(),
            new InstantAction(outtake.claw::open),
            specFullTransferAction(intake, outtake)
        ),"Is the SPECIMEN transfer correct"),
        // transfer sample
        //   pickup 0
        testAction(() -> new SequentialAction(
                new InstantAction(() -> intake.claw.turret_angle = 0),
                intake.pickUpAction(),
                new InstantAction(outtake.claw::open),
                fullTransferAction(intake, outtake)
        ),"Is the SAMPLE transfer correct at ZERO DEGREES"),
        //   pickup 90
        testAction(() -> new SequentialAction(
                new InstantAction(() -> intake.claw.turret_angle = PI/2),
                intake.pickUpAction(),
                new InstantAction(outtake.claw::open),
                fullTransferAction(intake, outtake)
        ),"Is the SAMPLE transfer correct at NINETY DEGREES"),
        // PEDRO
        testAction(NullAction::new, "Flip the robot on its side (no wheels touching the ground)"),
        // motors
        testAction(NullAction::new, "Make sure the motors are all screwed in and don't interfere with side plates"),
        // back pod
        testAction(NullAction::new, "Make sure the back odo pod is in the right position, and remember to run run and test motor directions, localization test, and vision testing")
        // pinpoint
        // CONTROLLERS
        ));
    }
}
