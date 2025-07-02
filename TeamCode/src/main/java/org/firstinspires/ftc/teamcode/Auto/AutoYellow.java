package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoYellow", group = "auto")
public class AutoYellow extends SampleAutoBase {
    @Override
    void setParams() {
        yellow = true;
        red = false;
        blue = false;
        strict_boundary = false;
    }
}
