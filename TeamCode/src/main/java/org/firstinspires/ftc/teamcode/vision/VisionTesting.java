package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.util.CustomActions.foreverAction;
import static org.firstinspires.ftc.teamcode.util.CustomActions.futureAction;
import static org.firstinspires.ftc.teamcode.util.CustomActions.runBlocking;
import static org.firstinspires.ftc.teamcode.util.CustomActions.triggerAction;
import static org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions.fullTransferAction;
import static org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions.reachSample;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Claw;
import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.util.mechanisms.outtake.Outtake;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(group = "testing")
public class VisionTesting extends LinearOpMode {
    Follower follower;
    Intake intake;
    Outtake outtake;
    Vision vision;
    Sample sample = new Sample();
    boolean end = false;
    Action getAction(){
        return new SequentialAction(
            new RaceAction(
                vision.findSample(sample),
                foreverAction(follower::update)
            ),
            futureAction(() -> reachSample(sample.pose, intake, follower)),
            new RaceAction(
                new SequentialAction(
                    intake.pickUpAction(),
                    futureAction(() -> new SleepAction(
                        Math.abs((sample.pose.getHeading()%(PI) + PI)%(PI)) < PI/4? 0.6: 0.3
                    )),
                    fullTransferAction(intake, outtake)
                ),
                foreverAction(follower::update)
            ),
            new InstantAction(() -> {
                telemetry.addData("query", intake.claw.getSensedColor());
                telemetry.update();
                end = intake.claw.getSensedColor() != Claw.ColorSensorOut.NONE;
            })
        );
    }
    @Override
    public void runOpMode() throws InterruptedException {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        vision = new Vision(
                telemetry,
                hardwareMap
        );
        SampleLocationPipeline.AllowedColors.yellow = true;
        SampleLocationPipeline.AllowedColors.blue = true;
        outtake.readyTransfer();

        waitForStart();
        runBlocking(new RaceAction(
            triggerAction(() -> end),
            foreverAction(this::getAction)
        ));
    }
}
