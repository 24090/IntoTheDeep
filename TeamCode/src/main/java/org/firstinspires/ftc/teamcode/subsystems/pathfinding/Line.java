package org.firstinspires.ftc.teamcode.subsystems.pathfinding;

import com.acmerobotics.roadrunner.Vector2d;

import kotlin.Pair;

public class Line{
    private Vector2d first;
    private Vector2d second;
    public Line(Vector2d first, Vector2d second) {
        this.first = first;
        this.second = second;
    }

    public Vector2d getFirst() {
        return first;
    }

    public Vector2d getSecond() {
        return second;
    }
}
