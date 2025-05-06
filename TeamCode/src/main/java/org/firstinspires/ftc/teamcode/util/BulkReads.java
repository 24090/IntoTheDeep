package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BulkReads {
    public static void setCachingMode(HardwareMap hardwareMap, LynxModule.BulkCachingMode mode){
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(mode);
        }
    }

    public static void readManual(HardwareMap hardwareMap){
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.clearBulkCache();
        }
    }
}
