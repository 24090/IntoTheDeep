package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.util.CustomActions.foreverAction;
import static org.firstinspires.ftc.teamcode.util.CustomActions.futureAction;
import static org.firstinspires.ftc.teamcode.util.CustomActions.runBlocking;
import static org.firstinspires.ftc.teamcode.util.CustomActions.triggerAction;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions;
import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Claw;
import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.util.mechanisms.outtake.Outtake;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(group = "testing")
public class VisionTesting extends LinearOpMode {
    Sample sample = new Sample();

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        Intake intake = new Intake(hardwareMap);
        boolean[] has_sample = {false};
        Outtake outtake = new Outtake(hardwareMap);
        Vision vision = new Vision(
                telemetry,
                hardwareMap
        );
        outtake.readyTransfer();
        Action action = new SequentialAction(
                new InstantAction(() -> sample.pose = null),
                vision.findSample(sample),
                futureAction(() ->
                        RobotActions.reachSample(sample.pose, intake, follower)
                ),
                new SleepAction(0.5),
                intake.pickUpAction(),
                new InstantAction(() ->
                        has_sample[0] = (intake.claw.getSensedColor() != Claw.ColorSensorOut.NONE)
                ),
                RobotActions.fullTransferAction(intake, outtake)
        );
        waitForStart();
        while (opModeIsActive()){
            if (!has_sample[0]) {
                runBlocking(
                        new RaceAction(
                            foreverAction(follower::update),
                            triggerAction(() -> !opModeIsActive()),
                            action
                        )
                );
            } else {
                break;
            }

        }
    }
}
