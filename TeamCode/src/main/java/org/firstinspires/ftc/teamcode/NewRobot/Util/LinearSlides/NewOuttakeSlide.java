package org.firstinspires.ftc.teamcode.NewRobot.Util.LinearSlides;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class NewOuttakeSlide extends NewLinearSlide {
    public HardwareMap hardwareMap;
    public String motorName;
    public NewOuttakeSlide(HardwareMap hardwareMap, String motorName, double maxExtension, double minExtension) {
        super(hardwareMap, motorName, maxExtension, minExtension, 0.2);
        this.hardwareMap = hardwareMap;
        this.motorName = motorName;
    }
    public Thread slideTo(double pos){
        return extendToAsync(pos, 50);
    }
}
