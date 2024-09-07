package org.firstinspires.ftc.teamcode.subsystems.pathfinding;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import java.util.Arrays;
import java.util.Vector;
import java.util.logging.Logger;
import java.util.stream.IntStream;

import kotlin.Pair;

public class Collision {

    // https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
    private static Boolean isOrderedCCW(Vector2d a, Vector2d b, Vector2d c){
        return (c.y-a.y)*(b.x-a.x) > (b.y-a.y)*(c.x-a.x);
    }

    // https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
    private static Boolean testLineCollision(Line a, Line b){
        return (
            (isOrderedCCW(a.getFirst(), b.getFirst(), b.getSecond()) != isOrderedCCW(a.getSecond(), b.getFirst(), b.getSecond()))
            && (isOrderedCCW(a.getFirst(), a.getSecond(), b.getFirst()) != isOrderedCCW(a.getFirst(), a.getSecond(), b.getSecond()))
        );
    }
    private static Vector2d changePerspective(Vector2d point, Pose2d perspective){
        point = point.minus(perspective.position);
        point = perspective.heading.inverse().times(point);
        return point;
    }
    private static Vector2d[] changePerspective(Vector2d[] points, Pose2d perspective){
        return Arrays.stream(points).map(p -> changePerspective(p, perspective)).toArray(Vector2d[]::new);
    }

    private static Line[] generateLines(Vector2d[] start_points, Vector2d[] end_points){
        return IntStream.range(0, start_points.length).mapToObj(i -> new Line(start_points[i], end_points[i])).toArray(Line[]::new);
    }
    /**
     *
     * @param centered_object Collision object for which collisions are tested for
     * @param moving_object Collision object for which collisions are tested for
     * @return if there is a collision
     */
    public static Boolean TestCollisionMovingCenter(CollisionObject moving_object, CollisionObject centered_object, double start_time, double end_time, double time_res){
        double current_time = start_time;
        Line[] outline_center_start;
        Vector2d[] points_moving_start;
        Line[] outline_center_end = centered_object.getOutlineAtTime(current_time);
        Vector2d[] points_moving_end = moving_object.getPointsAtTime(current_time);
        points_moving_end = changePerspective(points_moving_end, centered_object.getPerspectiveAtTime(current_time));
        // With A as center         
        for (;current_time<=end_time; current_time += time_res){
            outline_center_start = outline_center_end;
            points_moving_start = points_moving_end;
            outline_center_end = moving_object.getOutlineAtTime(current_time);
            points_moving_end = changePerspective(moving_object.getPointsAtTime(current_time), centered_object.getPerspectiveAtTime(current_time));
            Line[] lines_moving = generateLines(points_moving_start, points_moving_end);
            for (Line line: lines_moving){
                for (Line outline: outline_center_start){
                    if (testLineCollision(line, outline)){
                        return true;
                    }
                }
            }
        }
        return false;
    }
    public static Boolean TestCollision(CollisionObject a, CollisionObject b, double start_time, double end_time, double time_res){
        return TestCollisionMovingCenter(a, b, start_time, end_time, time_res) || TestCollisionMovingCenter(b, a, start_time, end_time, time_res);

    }
}
