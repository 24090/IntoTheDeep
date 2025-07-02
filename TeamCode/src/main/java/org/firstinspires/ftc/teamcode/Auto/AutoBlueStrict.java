package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoBlueStrict", group = "auto")
public class AutoBlueStrict extends SampleAutoBase {
    @Override
    void setParams() {
        yellow = true;
        red = false;
        blue = true;
        strict_boundary = true;
    }
}
