package org.firstinspires.ftc.teamcode.controlled;

import static java.lang.Math.PI;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
//import org.firstinspires.ftc.teamcode.util.ColorDistance;
import org.firstinspires.ftc.teamcode.util.GameMap;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.MechanismActions;
import org.firstinspires.ftc.teamcode.util.Outtake;
import org.firstinspires.ftc.teamcode.util.PoseStorer;
import org.firstinspires.ftc.teamcode.vision.Vision;

/**
 * TODO:
 *  give pose to next OpMode/take pose from previous
 *  assisted aim if necessary
 *  dpad mode if necessary
 */
@TeleOp(name = "Controller")
public class Controlled extends LinearOpMode{
    FtcDashboard dash = FtcDashboard.getInstance();
    int next_auto_action = 0;
    public void runOpMode(){
        final Pose2d score_pose = new Pose2d(GameMap.NetRedCorner.plus(new Vector2d(16.5, 16.5)), PI / 4);
        MecanumDrive drive = new MecanumDrive(hardwareMap, PoseStorer.pose);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake;
        outtake = new Outtake(hardwareMap);
        MechanismActions actions = new MechanismActions(intake,outtake,this);
        Thread t = new Thread(() -> {
            while (opModeIsActive()){
                if (intake.linear_slide.getPosition() > 100){
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x/3));
                } else {
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x));
                }

                drive.updatePoseEstimate();
            }
        });
        Vision vision = new Vision(drive, intake, telemetry, hardwareMap);
        waitForStart();
        t.start();
        while (opModeIsActive()){
            if (gamepad1.right_trigger > 0.6) {
                t.interrupt();
                Action auto_action = null;
                if (next_auto_action == 2) {
                    auto_action = drive.actionBuilder(drive.pose)
                            .stopAndAdd(intake.fullTransferAction())
                            .setTangent(0)
                            .strafeToSplineHeading(score_pose.position, score_pose.heading, new VelConstraint() {
                                @Override
                                public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                                    return pose2dDual.value().position.minus(score_pose.position).norm() < 24 ? 30 : 100;
                                }
                            })
                            .afterDisp(score_pose.position.minus(drive.pose.position).norm() - 12, actions.OuttakeSlideUpAction())
                            .stopAndAdd(actions.ScoreAction())
                            .build();
                }else if (next_auto_action == 1){
                    auto_action = vision.LookAtSample();
                } else if (next_auto_action == 0){
                    auto_action = drive.actionBuilder(drive.pose)
                            .stopAndAdd(intake.readyTransferAction())
                            .setTangent(0)
                            .strafeToSplineHeading(GameMap.SubmersibleRedEdge.minus(new Vector2d(GameMap.RobotLength + 28,-24)), 0)
                            .setTangent(0)
                            .strafeToSplineHeading(GameMap.SubmersibleRedEdge.minus(new Vector2d(GameMap.RobotLength + 4,-24)), 0)
                            .build();
                }
                next_auto_action = (next_auto_action + 1)%2;
                TelemetryPacket packet = new TelemetryPacket();
                while (gamepad1.right_trigger > 0.6) {
                    assert auto_action != null;
                    if (!auto_action.run(packet)) break;
                    drive.updatePoseEstimate();
                    auto_action.preview(packet.fieldOverlay());
                    dash.sendTelemetryPacket(packet);
                }
                t.start();

            }
            if (gamepad1.left_bumper){
                intake.readyGrab(
                        gamepad1.left_trigger * (GameMap.MaxIntakeDistance - GameMap.MinIntakeDistance),
                        0 // TODO: Claw rotation
                );
            } else if (gamepad1.right_bumper){
                intake.readyTransfer();
            }
            if (gamepad1.dpad_up){
                intake.claw.grab();
            } else if (gamepad1.dpad_down){
                intake.claw.open();
            }
            if (gamepad1.y){
                Actions.runBlocking(actions.OuttakeSlideUpAction());
            } else if (gamepad1.a){
                Actions.runBlocking(actions.OuttakeSlideDownAction());
            } else if (gamepad1.b){
                outtake.open();
            } else if (gamepad1.x){
                outtake.close();
            }
        }
    }
}
