package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.util.customactions.RunBlocking.runBlocking;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.RobotActions;
import org.firstinspires.ftc.teamcode.util.customactions.ForeverAction;
import org.firstinspires.ftc.teamcode.util.customactions.FutureAction;
import org.firstinspires.ftc.teamcode.util.customactions.TriggerAction;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(group = "testing")
public class VisionTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        Intake intake = new Intake(hardwareMap);
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
                            new FutureAction( () ->
                                RobotActions.reachSample(sample.pose, intake, follower)
                            ),
                            new SleepAction(0.5),
                            intake.pickUpAction(),
                            intake.fullTransferAction(),
                            new TriggerAction(() -> gamepad1.a)
                        ),
                        new ForeverAction(() -> {
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
