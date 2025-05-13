package org.firstinspires.ftc.teamcode.util.customactions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Path;
public class PathAction implements Action{
    PathChain path_chain;
    Follower follower;

    public PathAction(PathChain path_chain, Follower follower) {
        this.path_chain = path_chain;
        this.follower = follower;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        follower.followPath(path_chain, true)
        return follower.isBusy();
    }
}
