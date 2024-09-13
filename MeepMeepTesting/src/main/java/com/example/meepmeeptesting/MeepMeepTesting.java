package com.example.meepmeeptesting;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, PI, PI, 15)
                .build();

        myBot.runAction(rightNeutralSpikeMarksToRedSubmersibleEdgeBlocked(myBot));
        Image image = null;
        try {image = ImageIO.read(new File("./MeepMeepTesting/src/main/java/com/example/meepmeeptesting/ftc-map.png"));
        } catch (IOException ex) {ex.printStackTrace();}
        meepMeep.setBackground(image)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
    public static Action redNetZoneToRedSpikeMarks(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.NetRedCorner.times(0.666), -3*PI/4))
            .strafeToSplineHeading(GameMap.SpikeMarkRedCenter.minus(new Vector2d(1, 11)), PI/2)
            .build();
    }
    public static Action redNetZoneToLeftNeutralSpikeMarks(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.NetRedCorner.times(0.666), -3*PI/4))
                .strafeToSplineHeading(GameMap.SpikeMarkNeutralLeftCenter.minus(new Vector2d(-2, 11)), PI/2)
                .build();
    }
    public static Action redNetZoneToRightNeutralSpikeMarks(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.NetRedCorner.times(0.666), -3*PI/4))
                .strafeToSplineHeading(GameMap.NetRedCorner.times(0.666).minus(new Vector2d(-40, 0)), 0)
                .splineTo(GameMap.SpikeMarkNeutralRightCenter.minus(new Vector2d(1, 12)), PI/2)
                .build();
    }
    public static Action redNetZoneToRedAscentZone(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.NetRedCorner.times(0.666), -3*PI/4))
                .strafeToSplineHeading(GameMap.NetRedCorner.times(0.5), -PI/4)
                .splineTo(GameMap.AscentZoneEdgeRed.minus(new Vector2d(12,0)), PI/2)
                .build();
    }
    public static Action redNetZoneToRedObservationZone(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.NetRedCorner.times(0.666), -3*PI/4))
                .strafeToSplineHeading(GameMap.ObservationRedCorner.plus(new Vector2d(-36, 24)), -1*PI/4)
                .build();
    }
    public static Action redNetZoneToRedSubmersibleEdge(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.NetRedCorner.times(0.666), -3*PI/4))
                .strafeToSplineHeading(GameMap.SubmersibleRedEdge.plus(new Vector2d(0, -12)), PI/2)
                .build();
    }
    public static Action redSpikeMarksToLeftNeutralSpikeMarks(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.SpikeMarkRedCenter.minus(new Vector2d(1, 11)), PI/2))
                .strafeToSplineHeading(GameMap.SpikeMarkNeutralLeftCenter.minus(new Vector2d(-2, 11)), PI/2)
                .build();
    }
    public static Action redSpikeMarksToRightNeutralSpikeMarks(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.SpikeMarkRedCenter.minus(new Vector2d(1, 11)), PI/2))
                .strafeToSplineHeading(GameMap.SpikeMarkNeutralRightCenter.minus(new Vector2d(1, 13)), PI/2)
                .build();
    }
    public static Action redSpikeMarksToRedAscentZone(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.SpikeMarkRedCenter.minus(new Vector2d(1, 11)), PI/2))
                .strafeToSplineHeading(GameMap.SpikeMarkRedCenter.minus(new Vector2d(64, 13)), PI/2)
                .splineTo(GameMap.AscentZoneEdgeRed.minus(new Vector2d(12, 0)),PI/2)
                .build();
    }
    public static Action redSpikeMarksToRedObservationZone(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.SpikeMarkRedCenter.minus(new Vector2d(1, 11)), PI/2))
                .strafeToSplineHeading(GameMap.ObservationRedCorner.minus(new Vector2d(36, -24)), -1*PI/4)
                .build();
    }
    public static Action redSpikeMarksToRedSubmersibleEdge(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.SpikeMarkRedCenter.minus(new Vector2d(1, 11)), PI/2))
                .strafeToSplineHeading(GameMap.SubmersibleRedEdge.plus(new Vector2d(0, -12)), PI/2)
                .build();
    }
    public static Action leftNeutralSpikeMarksToRightNeutralSpikeMarks(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.SpikeMarkNeutralLeftCenter.minus(new Vector2d(-2, 11)), PI/2))
                .strafeToSplineHeading(GameMap.SpikeMarkNeutralLeftCenter.plus(new Vector2d(62, -11)), PI/2)
                .splineTo(GameMap.SpikeMarkNeutralRightCenter.minus(new Vector2d(1, 13)), 0)
                .build();
    }
    public static Action leftNeutralSpikeMarksToRedAscentZone(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.SpikeMarkNeutralLeftCenter.minus(new Vector2d(-2, 11)), PI/2))
                .splineTo(GameMap.AscentZoneEdgeRed.minus(new Vector2d(12, 0)), 0)
                .build();
    }
    public static Action leftNeutralSpikeMarksToRedObservationZone(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.SpikeMarkNeutralLeftCenter.minus(new Vector2d(-2, 11)), PI/2))
                .strafeToSplineHeading(GameMap.ObservationRedCorner.minus(new Vector2d(36, -24)), -1*PI/4)
                .build();
    }
    public static Action leftNeutralSpikeMarksToRedSubmersibleEdge(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.SpikeMarkNeutralLeftCenter.minus(new Vector2d(-2, 11)), PI/2))
                .strafeToSplineHeading(GameMap.SubmersibleRedEdge.plus(new Vector2d(0, -12)), PI/2)
                .build();
    }
    public static Action rightNeutralSpikeMarksToRedAscentZone(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.SpikeMarkNeutralRightCenter.minus(new Vector2d(1, 13)), PI/2))
                .splineTo(new Vector2d(25, -37), -PI)
                .strafeToSplineHeading(new Vector2d(-10, -37), -PI)
                .splineTo(GameMap.AscentZoneEdgeRed.minus(new Vector2d(12, 0)), 0)
                .build();
    }
    public static Action rightNeutralSpikeMarksToRedObservationZone(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.SpikeMarkNeutralRightCenter.minus(new Vector2d(1, 13)), PI/2))
                .strafeToSplineHeading(GameMap.ObservationRedCorner.minus(new Vector2d(36, -24)), -1*PI/4)
                .build();
    }
    public static Action rightNeutralSpikeMarksToRedSubmersibleEdge(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.SpikeMarkNeutralRightCenter.minus(new Vector2d(1, 13)), PI/2))
                .strafeToSplineHeading(new Vector2d(25, -40), -PI)
                .strafeToSplineHeading(GameMap.SubmersibleRedEdge.plus(new Vector2d(0, -12)), PI/2)
                .build();
    }
    public static Action redAscentZoneToRedObservationZone(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.AscentZoneEdgeRed.minus(new Vector2d(12, 0)), 0))
                .strafeToSplineHeading(new Vector2d(-26, -30), -1*PI/8)
                .strafeToSplineHeading(GameMap.ObservationRedCorner.minus(new Vector2d(36, -24)), -1*PI/4)
                .build();
    }
    public static Action redAscentZoneToRedSubmersibleEdge(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.AscentZoneEdgeRed.minus(new Vector2d(12, 0)), 0))
                .strafeToSplineHeading(new Vector2d(-26, -40), -1*PI/8)
                .strafeToSplineHeading(GameMap.SubmersibleRedEdge.plus(new Vector2d(0, -12)), PI/2)
                .build();
    }
    public static Action redObservationZoneToRedSubmersibleEdge(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.ObservationRedCorner.minus(new Vector2d(36, -24)), -1*PI/4))
                .strafeToSplineHeading(GameMap.SubmersibleRedEdge.plus(new Vector2d(0, -12)), PI/2)
                .build();
    }
    public static Action redNetZoneToRightNeutralSpikeMarksBlocked(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.NetRedCorner.times(0.666), -3*PI/4))
                .strafeToSplineHeading(GameMap.NetRedCorner.times(0.666).minus(new Vector2d(-40, 0)), PI/4)
                .splineTo(GameMap.SpikeMarkNeutralRightCenter.minus(new Vector2d(1, 12)), PI/4)
                .build();
    }
    public static Action redSpikeMarksToRightNeutralSpikeMarksBlocked(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.SpikeMarkRedCenter.minus(new Vector2d(1, 11)), PI/2))
                .strafeToSplineHeading(GameMap.SpikeMarkRedCenter.minus(new Vector2d(36, 11)), PI/4+0.15)
                .strafeToSplineHeading(GameMap.SpikeMarkNeutralRightCenter.minus(new Vector2d(1, 13)), PI/2)
                .build();
    }
    public static Action leftNeutralSpikeMarksToRedAscentZoneBlocked(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.SpikeMarkNeutralLeftCenter.minus(new Vector2d(-2, 11)), PI/2))
                .strafeToSplineHeading(GameMap.SpikeMarkNeutralLeftCenter.minus(new Vector2d(-22, 11)), PI/2)
                .strafeToSplineHeading(GameMap.AscentZoneEdgeRed.minus(new Vector2d(12, 0)), 0)
                .build();
    }
    public static Action rightNeutralSpikeMarksToRedAscentZoneBlocked(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.SpikeMarkNeutralRightCenter.minus(new Vector2d(1, 13)), PI/2))
                .strafeToSplineHeading(new Vector2d(25, -37), -PI)
                .strafeToSplineHeading(new Vector2d(-10, -37), -PI)
                .splineTo(GameMap.AscentZoneEdgeRed.minus(new Vector2d(12, 0)), 0)
                .build();
    }
    public static Action rightNeutralSpikeMarksToRedObservationZoneBlocked(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.SpikeMarkNeutralRightCenter.minus(new Vector2d(1, 13)), PI/2))
                .strafeToSplineHeading(new Vector2d(30, -24), PI/2)
                .strafeToSplineHeading(GameMap.ObservationRedCorner.minus(new Vector2d(36, -24)), -1*PI/4)
                .build();
    }
    public static Action rightNeutralSpikeMarksToRedSubmersibleEdgeBlocked(RoadRunnerBotEntity robot){
        return robot.getDrive().actionBuilder(new Pose2d(GameMap.SpikeMarkNeutralRightCenter.minus(new Vector2d(1, 13)), PI/2))
                .strafeToSplineHeading(new Vector2d(25, -40), -PI)
                .strafeToSplineHeading(GameMap.SubmersibleRedEdge.plus(new Vector2d(0, -12)), PI/2)
                .build();
    }

}