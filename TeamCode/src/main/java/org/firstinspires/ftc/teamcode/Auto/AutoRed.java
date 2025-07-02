package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoRed", group = "auto")
public class AutoRed extends SampleAutoBase {@Override void setParams() {
    yellow = true;
    red = true;
    blue = false;
    strict_boundary = false;
}}
