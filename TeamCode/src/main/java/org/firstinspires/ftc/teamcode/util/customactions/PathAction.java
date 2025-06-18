package org.firstinspires.ftc.teamcode.util.customactions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class PathAction{
    public static Action pathAction(Follower follower, PathChain path){
        return new SequentialAction(
                new InstantAction(()->follower.followPath(path, true)),
                new TriggerAction(()->(!follower.isBusy())&&(follower.getVelocityMagnitude()<2)&&(follower.getHeadingError()<0.04))
        );
    }
    public static Action moveLineAction(Follower follower, Pose a, Pose b) {
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(a), new Point(b)))
                .setLinearHeadingInterpolation(a.getHeading(), b.getHeading())
                .build();
        return pathAction(follower, path);
    }
}
