package net.tecdroid.subsystems.LimeLight;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class LimeLightsController {
    LimeLightModule leftLimeLight;
    LimeLightModule rightLimeLight;

    public LimeLightsController() {
        leftLimeLight = new LimeLightModule(LimeLightConfiguration.leftDeviceConfig);
        rightLimeLight = new LimeLightModule(LimeLightConfiguration.rightDeviceConfig);

    }

    public Distance getLeftLimeLightDistance() {
        return leftLimeLight.getDistance();
    }

    public Distance getRightLimeLightDistance() {
        return rightLimeLight.getDistance();
    }

    public Angle getLeftLimeLightTX() {
        return leftLimeLight.getTx();
    }

}