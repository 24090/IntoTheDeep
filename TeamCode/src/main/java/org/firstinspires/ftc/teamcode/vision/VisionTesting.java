package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.util.CustomActions.foreverAction;
import static org.firstinspires.ftc.teamcode.util.CustomActions.futureAction;
import static org.firstinspires.ftc.teamcode.util.CustomActions.runBlocking;
import static org.firstinspires.ftc.teamcode.util.CustomActions.triggerAction;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions;
import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.util.mechanisms.outtake.Outtake;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(group = "testing")
public class VisionTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Vision vision = new Vision(
                telemetry,
                hardwareMap
        );
        waitForStart();
        while (opModeIsActive()){
            Sample sample = new Sample();
            runBlocking(
                    new ParallelAction(
                        new SequentialAction(
                            vision.findSample(sample),
                            futureAction( () ->
                                RobotActions.reachSample(sample.pose, intake, follower)
                            ),
                            new SleepAction(0.5),
                            intake.pickUpAction(),
                            RobotActions.fullTransferAction(intake, outtake),
                            triggerAction(() -> gamepad1.a)
                        ),
                        foreverAction(() -> {
                            if (sample.pose != null) {
                                telemetry.addData("sample x", sample.pose.getX());
                                telemetry.addData("sample y", sample.pose.getY());
                                telemetry.addData("sample Θ", sample.pose.getHeading());
                                telemetry.update();
                            }

                        })
                    )

            );
        }
    }
}
