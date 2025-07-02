package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public class SampleAutos {
    @Autonomous(name = "AutoRed", group = "auto")
    public class AutoRed extends SampleAutoBase {@Override void setParams() {
        yellow = true;
        red = true;
        blue = false;
        strict_boundary = false;
    }}
    @Autonomous(name = "AutoBlue", group = "auto")
    public class AutoBlue extends SampleAutoBase {@Override void setParams() {
        yellow = true;
        red = false;
        blue = true;
        strict_boundary = false;
    }}
    @Autonomous(name = "AutoYellow", group = "auto")
    public class AutoYellow extends SampleAutoBase {@Override void setParams() {
        yellow = true;
        red = false;
        blue = false;
        strict_boundary = false;
    }}
    @Autonomous(name = "AutoRedStrict", group = "auto")
    public class AutoRedStrict extends SampleAutoBase { @Override void setParams() {
        yellow = true;
        red = true;
        blue = false;
        strict_boundary = true;
    }}
    @Autonomous(name = "AutoBlueStrict", group = "auto")
    public class AutoBlueStrict extends SampleAutoBase {@Override void setParams() {
        yellow = true;
        red = false;
        blue = true;
        strict_boundary = true;
    }}
    @Autonomous(name = "AutoYellowStrict", group = "auto")
    public class AutoYellowStrict extends SampleAutoBase {@Override void setParams() {
        yellow = true;
        red = false;
        blue = false;
        strict_boundary = true;
    }}
}
