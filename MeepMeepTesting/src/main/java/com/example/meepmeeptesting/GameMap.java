package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Vector2d;

public class GameMap {
    // -------OBSERVATION CORNERS-------
    // The very corner of the field in which the observation zone is
    public static final Vector2d ObservationBlueCorner = new Vector2d(-72, 72);
    public static final Vector2d ObservationRedCorner = new Vector2d(72, -72);
    // -------NET CORNERS---------------
    // The very corner of the field in which the net zone is
    public static final Vector2d NetRedCorner = new Vector2d(-72, -72);
    public static final Vector2d NetBlueCorner = new Vector2d(72, 72);
    // -------ASCENT ZONE EDGES---------
    // The center of the edge of the submersible zone that touches the ascent zone
    public static final Vector2d AscentZoneEdgeBlue = new Vector2d(27.5/2, 0);
    public static final Vector2d AscentZoneEdgeRed = new Vector2d(-27.5/2, 0);
    // -------SUBMERSIBLE EDGES----------
    // The center of the edge of the edge of the submersible zone that doesn't touch the ascent zone
    public static final Vector2d SubmersibleBlueEdge = new Vector2d(0, 42.75/2);
    public static final Vector2d SubmersibleRedEdge = new Vector2d(0, -42.75/2);
    // -------SPIKE MARKS----------------
    // The center of the spike mark
    // RED
    public static final Vector2d SpikeMarkRedInner = new Vector2d(24, -24-3.5/2);
    public static final Vector2d SpikeMarkRedCenter = new Vector2d(34, -24-3.5/2);
    public static final Vector2d SpikeMarkRedOuter = new Vector2d(44, -24-3.5/2);
    // BLUE
    public static final Vector2d SpikeMarkBlueInner = new Vector2d(-24, 24+3.5/2);
    public static final Vector2d SpikeMarkBlueCenter = new Vector2d(-34, 24+3.5/2);
    public static final Vector2d SpikeMarkBlueOuter = new Vector2d(-44, 24+3.5/2);
    // NEUTRAL RIGHT
    // right = +x
    public static final Vector2d SpikeMarkNeutralRightInner = new Vector2d(24, 24+3.5/2);
    public static final Vector2d SpikeMarkNeutralRightCenter = new Vector2d(34, 24+3.5/2);
    public static final Vector2d SpikeMarkNeutralRightOuter = new Vector2d(44, 24+3.5/2);
    // NEUTRAL LEFT
    // left = -x
    public static final Vector2d SpikeMarkNeutralLeftInner = new Vector2d(-24, -24-3.5/2);
    public static final Vector2d SpikeMarkNeutralLeftCenter = new Vector2d(-34, -24-3.5/2);
    public static final Vector2d SpikeMarkNeutralLeftOuter = new Vector2d(-44, -24-3.5/2);
    // -------APRIL TAGS-----------------
    // the center of the april tag in x/y
    // BLUE
    public static final Vector2d AprilTagBlueCenter = new Vector2d(0, 72);
    public static final Vector2d AprilTagBlueLeft = new Vector2d(-72, 48);
    public static final Vector2d AprilTagBlueRight = new Vector2d(72, 48);
    public static final int AprilTagBlueCenterID = 12;
    public static final int AprilTagBlueLeftID = 11;
    public static final int AprilTagBlueRightID = 13;
    // RED
    public static final Vector2d AprilTagRedCenter = new Vector2d(0, -72);
    public static final Vector2d AprilTagRedLeft = new Vector2d(-72, -48);
    public static final Vector2d AprilTagRedRight = new Vector2d(72, -48);
    public static final int AprilTagRedCenterID = 15;
    public static final int AprilTagRedLeftID = 16;
    public static final int AprilTagRedRightID = 14;
}
