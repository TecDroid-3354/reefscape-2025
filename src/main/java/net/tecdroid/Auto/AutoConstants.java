package net.tecdroid.Auto;

import net.tecdroid.util.PidfCoefficients;

public class AutoConstants {

    // TODO: TUNEAR LOS PIDS BIEN
    public static PidfCoefficients forwardPID = new PidfCoefficients(0.0003, 0.0, 0.008, 0.0);
    public static PidfCoefficients sidewaysPID = new PidfCoefficients(0.0003, 0.0, 0.008, 0.0);
    public static PidfCoefficients rotationPID = new PidfCoefficients(0.0003, 0.0, 0.008, 0.0);
}
