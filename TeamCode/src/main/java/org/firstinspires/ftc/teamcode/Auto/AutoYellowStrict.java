package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoYellowStrict", group = "auto")
public class AutoYellowStrict extends SampleAutoBase {
    @Override
    void setParams() {
        yellow = true;
        red = false;
        blue = false;
        strict_boundary = true;
    }
}
