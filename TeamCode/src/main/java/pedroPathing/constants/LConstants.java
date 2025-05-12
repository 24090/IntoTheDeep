package pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

@Config
public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0;
        ThreeWheelConstants.strafeTicksToInches = 0;
        ThreeWheelConstants.turnTicksToInches = 0;
        ThreeWheelConstants.leftY = 0;
        ThreeWheelConstants.rightY = 0;
        ThreeWheelConstants.strafeX = 0;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "par0";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "par1";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "perp";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




