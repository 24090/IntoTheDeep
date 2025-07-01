package org.firstinspires.ftc.teamcode.controlled;

import static org.firstinspires.ftc.teamcode.util.CustomActions.foreverAction;
import static org.firstinspires.ftc.teamcode.util.CustomActions.runBlocking;
import static org.firstinspires.ftc.teamcode.util.CustomActions.triggerAction;
import static org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions.specFullTransferAction;
import static org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions.fullTransferAction;
import static org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions.moveLineAction;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.PoseStorer;
import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.util.mechanisms.outtake.Outtake;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * TODO:
 *  give pose to next OpMode/take pose from previous
 *  assisted aim if necessary
 *  dpad mode if necessary
 */
@TeleOp(name = "Controller")
public class Controlled extends LinearOpMode{
    FtcDashboard dash = FtcDashboard.getInstance();
    Follower follower;
    Intake intake;
    Outtake outtake;
    public void runOpMode(){
        double last_time = 0;
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(PoseStorer.pose);
        boolean old_sweep = false;
        intake = new Intake(hardwareMap);
        intake.claw.toReadyGrabPos(0);
        outtake = new Outtake(hardwareMap);
        outtake.readyTransfer();
        Runnable movement = () -> {
            follower.setTeleOpMovementVectors(
                    -gamepad1.left_stick_y - gamepad2.left_stick_y / 7,
                    -gamepad1.left_stick_x - gamepad2.left_stick_x / 4,
                    -gamepad1.right_stick_x - gamepad2.right_stick_x / 7
            );
            follower.update();
        };
        follower.startTeleopDrive();
        waitForStart();
        while (opModeIsActive()){
            outtake.backgroundIter();
            intake.slide.movementLoop();
            movement.run();
            if (gamepad1.dpad_down){
                follower.breakFollowing();
                runBlocking(
                    new RaceAction(
                        triggerAction(() -> !gamepad1.dpad_down),
                        new ParallelAction(
                            moveLineAction(
                                follower,
                                follower.getPose(),
                                new Pose(19.25,144-19.25, -PI / 4)
                            ),
                            outtake.readySampleAction()
                        )
                    )
                );
                follower.breakFollowing();
                follower.startTeleopDrive();
            }
            if (gamepad1.right_bumper){
                outtake.readySpecimen();
            }
            if (gamepad1.y){
                outtake.readySample();
            } else if (gamepad1.a){
                outtake.readyTransfer();
            } else if (gamepad1.b){
                outtake.claw.open();
            }
            if (gamepad2.right_bumper){
                runBlocking(
                    new RaceAction(
                        foreverAction(movement::run),
                        fullTransferAction(intake, outtake)
                    )
                );
            }
            if (gamepad2.right_stick_button){
                intake.claw.turret_angle = 0;
                intake.readyGrab(
                        Intake.MaxDistance
                );
                outtake.claw.open();
            }
            if (gamepad2.back && !old_sweep){
                intake.sweeper.toggle();
            }
            if (gamepad2.dpad_up){
                runBlocking(
                    new RaceAction(
                        foreverAction(movement::run),
                        specFullTransferAction(intake, outtake)
                    )
                );
            }
            intake.slide.goTo(intake.slide.trimTicks(
                    intake.slide.target_pos +
                    (int) ((gamepad2.dpad_up? 1 : (gamepad2.dpad_down? -1 :0)) * 70 * (time - last_time))
            ));
            if (gamepad2.a){
                runBlocking(
                        new RaceAction(
                                foreverAction(movement::run),
                                foreverAction(outtake::backgroundIter),
                                foreverAction(intake.slide::movementLoop),
                                intake.pickUpAction()
                        )
                );
            }
            if (gamepad2.y){
                intake.claw.open();
            }
            if (gamepad2.b){
                intake.claw.turret_angle = 0;
                intake.claw.toReadyGrabPos(intake.claw.turret_angle);
                intake.claw.wrist_ready();
                outtake.claw.open();
            } else if (gamepad2.x){
                intake.claw.turret_angle = PI/2;
                intake.claw.toReadyGrabPos(intake.claw.turret_angle);
                intake.claw.wrist_ready();
                outtake.claw.open();
            }
            last_time = time;
            old_sweep = gamepad2.back;
        }
    }
}
