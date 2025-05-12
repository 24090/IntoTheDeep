package com.example.meepmeeptesting;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.zip.Inflater;

public class MeepMeepTesting {
    public static void main(String[] args) {

        final double RobotLength = 18.5;
        final double RobotWidth = 16.9;
        final double Intake_MaxDistance = 14 + RobotLength/2;
        final double Intake_MinDistance = 0.75 + RobotLength/2;
        final Vector2d inner_sample = new Vector2d(-48.75, -27);
        final Vector2d corner = new Vector2d(-72, -72);
        final Pose2d start_pose = new Pose2d(corner.plus(new Vector2d(24 + RobotLength / 2, RobotWidth/ 2)), 0);
        final Pose2d score_pose = new Pose2d(corner.plus(new Vector2d(18.5, 17)), PI / 4);
        final Vector2d inner_spike_mark_position = inner_sample.minus(new Vector2d(0, Intake_MaxDistance));
        final Vector2d neutral_spike_mark_position = inner_sample.minus(new Vector2d(10, 0)).minus(new Vector2d(0, Intake_MaxDistance - 0.5));
        final Vector2d outer_spike_mark_position = inner_sample.minus(new Vector2d(20, 0)).plus(
                Rotation2d.fromDouble(-0.95).vec().times(Intake_MaxDistance - 0.5)
        );
        final Vector2d inner_distance = inner_sample.minus(inner_spike_mark_position);
        final Vector2d center_distance = inner_sample.minus(new Vector2d(10, 0)).minus(neutral_spike_mark_position);
        final Vector2d outer_distance = inner_sample.minus(new Vector2d(20, 0)).minus(outer_spike_mark_position);
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity robot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, PI, PI, 15)
                .setDimensions(RobotWidth, RobotLength)
                .build();

        Action action = robot.getDrive().actionBuilder(start_pose)
                .setTangent(0)
                .strafeToSplineHeading(score_pose.position, score_pose.heading)
                .stopAndAdd(new NullAction())
                .setTangent(0)
                .afterTime(0, new NullAction())
                .strafeToSplineHeading(inner_spike_mark_position, inner_distance.angleCast().toDouble(), (pose2dDual, posePath, v) -> 10)
                .stopAndAdd(new NullAction())
                .setTangent(0)
                .afterTime(0, new NullAction())
                .strafeToSplineHeading(score_pose.position, score_pose.heading, (pose2dDual, posePath, v) -> 12)
                .stopAndAdd(new NullAction())
                .setTangent(0)
                .afterTime(0, new NullAction())
                .strafeToSplineHeading(neutral_spike_mark_position, center_distance.angleCast().toDouble(), (pose2dDual, posePath, v) -> 7)
                .stopAndAdd(new NullAction())
                .setTangent(0)
                .afterTime(0, new NullAction())
                .strafeToSplineHeading(score_pose.position, score_pose.heading,  (pose2dDual, posePath, v) -> 12)
                .stopAndAdd(new NullAction())
                .setTangent(0)
                .afterTime(0, new NullAction())
                .strafeToSplineHeading(outer_spike_mark_position, outer_distance.angleCast().toDouble(), (pose2dDual, posePath, v) -> 8)
                .stopAndAdd(new NullAction())
                .setTangent(0)
                .afterTime(0, new NullAction())
                .strafeToSplineHeading(score_pose.position, score_pose.heading, (pose2dDual, posePath, v) -> 12)
                .stopAndAdd(new NullAction())
                .setTangent(0)
                .afterTime(0, new NullAction())
                .splineToSplineHeading(new Pose2d(-34.06, -16.81, Math.toRadians(38.60)), Math.toRadians(38.60))
                .splineToSplineHeading(new Pose2d(-25.87, -13.08, 0), 0)
                .setTangent(0)
                .stopAndAdd(new NullAction())
                .stopAndAdd(new NullAction())
                .setTangent(0)
                .afterTime(0, new NullAction())
                .build();
        robot.runAction(action);
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .addEntity(robot)
                .start();
    }
}
