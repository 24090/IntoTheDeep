package org.firstinspires.ftc.teamcode.subsystems.pathfinding;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.QuinticSpline2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;

import kotlin.Pair;

public interface CollisonObject{
    /**
     * Used to generate the approximate lines for collision equations that don't use lines directly (splines, circles, etc.)
     * @return A list of lines (each in the form of two points) used for collision checking
     */
    public Pair<Vector2d, Vector2d>[] getLines();
    /**
     * Used to adjust the relative position of objects
     * @param t A value between zero and one, with each value of t representing a different point along the "path" of the CollisionObject
     * @return The Pose2d the CollisionObject has at a certain point along the path, relative to the game map
     */
    public Pose2d getPerspective(double t);
}