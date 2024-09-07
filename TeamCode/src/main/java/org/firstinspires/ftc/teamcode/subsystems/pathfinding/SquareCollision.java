package org.firstinspires.ftc.teamcode.subsystems.pathfinding;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.Vector;
import java.util.stream.IntStream;

import kotlin.Pair;

public class SquareCollision implements CollisionObject {
    private Vector2d[] points = {new Vector2d(-10,-10), new Vector2d(10,-10), new Vector2d(10,10), new Vector2d(-10,10)};
    public Vector2d[] getPointsAtTime(double time){
        return points;
    }
    public Line[] getOutlineAtTime(double time){
        // time has no effect here
        return  IntStream.range(0, points.length)
                        .mapToObj((int i) -> new Line(points[i],points[(i+1)%points.length])).toArray(Line[]::new)
                ;
    }

    public Pose2d getPerspectiveAtTime(double t) {
        return new Pose2d(0, 0, 0);
    }
}
