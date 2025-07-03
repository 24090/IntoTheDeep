package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.util.CustomActions.foreverAction;
import static org.firstinspires.ftc.teamcode.util.CustomActions.futureAction;
import static org.firstinspires.ftc.teamcode.util.CustomActions.runBlocking;
import static org.firstinspires.ftc.teamcode.util.CustomActions.triggerAction;
import static org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions.fullTransferAction;
import static org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions.moveLineAction;
import static org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions.reachSample;
import static org.firstinspires.ftc.teamcode.vision.SampleLocationPipeline.AllowedColors.blue;
import static org.firstinspires.ftc.teamcode.vision.SampleLocationPipeline.AllowedColors.red;
import static org.firstinspires.ftc.teamcode.vision.SampleLocationPipeline.AllowedColors.yellow;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.GameMap;
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
    boolean found_sample = false;
    boolean clawCheck() {
        Claw.ColorSensorOut color = intake.claw.getSensedColor();
        return (color == Claw.ColorSensorOut.BLUE && blue) || (color == Claw.ColorSensorOut.YELLOW && yellow) || (color == Claw.ColorSensorOut.RED && red);
    }
    Action getSampleAction(){
        return new SequentialAction(
                new RaceAction(
                        vision.findSample(sample),
                        foreverAction(() ->
                                new SequentialAction(
                                        new SleepAction(2),
                                        new InstantAction(intake.sweeper::moveOut),
                                        new SleepAction(0.5),
                                        new InstantAction(intake.sweeper::moveIn),
                                        new SleepAction(1.5),
                                        moveLineAction(
                                                follower,
                                                follower.getPose(),
                                                new Pose(
                                                        follower.getPose().getX(),
                                                        follower.getPose().getY() + GameMap.RobotWidth/2,
                                                        follower.getPose().getHeading())
                                        ),
                                        new SleepAction(2)
                                )
                        ),
                        foreverAction(follower::update)
                ),
                new InstantAction(intake.sweeper::moveIn),
                futureAction(() -> reachSample(sample.pose, intake, follower)),
                new RaceAction(
                        new SequentialAction(
                                intake.pickUpAction(),
                                futureAction(() -> new SleepAction(
                                        Math.abs((sample.pose.getHeading()%(PI) + PI)%(PI) - PI/2) < PI/4? 0.6: 0.3
                                )),
                                new InstantAction(outtake.claw::open),
                                fullTransferAction(intake, outtake)
                        ),
                        foreverAction(follower::update)
                ),
                new InstantAction(() ->
                        found_sample = clawCheck()
                )
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
        yellow = true;
        blue = true;
        outtake.readyTransfer();

        waitForStart();
        runBlocking(new RaceAction(
            triggerAction(() -> found_sample),
            foreverAction(this::getSampleAction)
        ));
    }
}
