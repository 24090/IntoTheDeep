package org.firstinspires.ftc.teamcode.util.customactions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;

public class PathAction implements Action{
    PathChain path_chain;
    Follower follower;
    Boolean path_set = false;
    public PathAction(Follower follower, PathChain path_chain) {
        this.path_chain = path_chain;
        this.follower = follower;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!path_set) {
            follower.followPath(path_chain, true);
            path_set=true;
        }
        follower.update();
        return follower.isBusy();
    }
}
