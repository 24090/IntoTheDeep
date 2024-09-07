package org.firstinspires.ftc.teamcode.subsystems.pathfinding;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MappedPosePath;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;
import java.util.Vector;
import java.util.stream.IntStream;
import java.util.stream.Stream;

import kotlin.Pair;

// Quint
public class RobotCollision implements CollisionObject {
    private Vector2d[] points;
    private Trajectory trajectory;
    public RobotCollision(Vector2d[] polygon_points, Trajectory trajectory){
        points = polygon_points;
        this.trajectory = trajectory;
    }
    public Vector2d[] getPointsAtTime(double time){
        return IntStream.range(0, points.length).mapToObj(i -> getPointAtTime(i, time)).toArray(Vector2d[]::new);
    }
    public Line[] getOutlineAtTime(double time){
        // time has no effect here (YET)
        return (Line[])
                IntStream.range(0, points.length)
                .mapToObj(
                        (int i) -> (Line) new Line(points[i], points[(i + 1) % points.length])
                )
                .toArray(Line[]::new);
    }
    public Vector2d getPointAtTime(int point_index, double t){
        Pose2d robot_pose = getPerspectiveAtTime(t);
        Vector2d rotated_point = robot_pose.heading.times(points[point_index]);
        return rotated_point.plus(robot_pose.position);
    }
    public Pose2d getPerspectiveAtTime(double t){
        TimeTrajectory time_trajectory = new TimeTrajectory(trajectory);
        return time_trajectory.get(t).value();
    }
}