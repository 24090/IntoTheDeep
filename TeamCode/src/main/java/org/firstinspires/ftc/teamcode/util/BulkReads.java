package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class BulkReads {
    List<LynxModule> modules;
    public BulkReads(HardwareMap hwmap){
        modules = hwmap.getAll(LynxModule.class);
    }
    public void setCachingMode(LynxModule.BulkCachingMode mode){
        for (LynxModule module : modules){
            module.setBulkCachingMode(mode);
        }
    }

    public void readManual(){
        for (LynxModule module : modules) {
            module.clearBulkCache();
        }
    }
}
