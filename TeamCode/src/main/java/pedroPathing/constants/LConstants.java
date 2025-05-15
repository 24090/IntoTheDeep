package pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

@Config
public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.002982618673122432;
        ThreeWheelConstants.strafeTicksToInches = 0.002969740152997143;
        ThreeWheelConstants.turnTicksToInches = -0.002987126756822587;
        ThreeWheelConstants.leftY = -3.80;
        ThreeWheelConstants.rightY = 3.80;
        ThreeWheelConstants.strafeX = 6.77;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "par0";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "par1";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "perp";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




