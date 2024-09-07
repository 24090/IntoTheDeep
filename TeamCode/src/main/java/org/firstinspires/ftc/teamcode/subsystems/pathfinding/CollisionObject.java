package org.firstinspires.ftc.teamcode.subsystems.pathfinding;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import kotlin.Pair;

public interface CollisionObject {
    /**
     * Used to generate lines that describe points over time, for when the object is not the frame of reference
     * @return A list of timestamped lines (each in the form of two points)
     */
    public Vector2d[] getPointsAtTime(double t);

    /**
     * Used to generate a static outline for when the object is the frame of reference
     * @return A list of lines (each in the form of two points)
     */
    public Line[] getOutlineAtTime(double t);
    /**
     * Used to adjust the relative position of objects
     * @param t A value between zero and one, with each value of t representing a different point along the "path" of the CollisionObject
     * @return The Pose2d the CollisionObject has at a certain point along the path, relative to the game map
     */
    public Pose2d getPerspectiveAtTime(double t);
}