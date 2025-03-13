package net.tecdroid.auto;

import net.tecdroid.util.ControlGains;

public class AutonomousConstants {
    public static ControlGains forwardPID = new ControlGains(2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    public static ControlGains sidewaysPID = new ControlGains(2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    public static ControlGains rotationalPID = new ControlGains(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

}