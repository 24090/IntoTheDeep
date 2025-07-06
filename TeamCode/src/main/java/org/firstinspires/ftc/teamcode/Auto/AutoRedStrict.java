package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoRedStrict", group = "auto")
public class AutoRedStrict extends SampleAutoBase {
    @Override
    void setParams() {
        yellow = true;
        red = true;
        blue = false;
        strict_boundary = true;
    }
}
