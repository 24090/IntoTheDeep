package org.firstinspires.ftc.teamcode.util.customactions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;

public class PathAction{
    public static Action pathAction(Follower follower, PathChain path){

        return new SequentialAction(
                new InstantAction(()->follower.followPath(path, true)),
                new TriggerAction(()->(!follower.isBusy())&&(follower.getVelocityMagnitude()<1)&&(follower.getHeadingError()<0.02))
        );
    }
}
