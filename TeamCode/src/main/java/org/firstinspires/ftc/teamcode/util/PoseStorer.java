package org.firstinspires.ftc.teamcode.util;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class PoseStorer {
    public static Pose2d pose = new Pose2d(GameMap.NetRedCorner.plus(new Vector2d(17.5, 17.5)), PI / 4);
}
