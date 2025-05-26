package pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
public class LConstants {
    static {
        PinpointConstants.forwardY = 3.80;
        PinpointConstants.strafeX = 6.77;
        PinpointConstants.distanceUnit = DistanceUnit.INCH;
        PinpointConstants.hardwareMapName = "pinpoint";
        PinpointConstants.useYawScalar = false;
        PinpointConstants.yawScalar = 1.0;
        PinpointConstants.useCustomEncoderResolution = false;
        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
        PinpointConstants.customEncoderResolution = 13.26291192;
        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    }
}




