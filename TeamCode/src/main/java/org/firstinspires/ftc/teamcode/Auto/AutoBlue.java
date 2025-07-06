package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoBlue", group = "auto")
public class AutoBlue extends SampleAutoBase {
    @Override
    void setParams() {
        yellow = true;
        red = false;
        blue = true;
        strict_boundary = false;
    }
}
