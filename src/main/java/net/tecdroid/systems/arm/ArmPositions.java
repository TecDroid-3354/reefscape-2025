package net.tecdroid.systems.arm;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

public class ArmPositions {
    // Reef
    // L1
    // public static final ArmPose teAmoPasaye = new ArmPose(Angle [wrist], Angle [joint], Distance [elevator]);

    // public static final ArmPose reefL1 = new ArmPose(Degrees.of(0.0), Degrees.of(0.0), Meters.of(0.0));
    public final ArmPose reefL2 = new ArmPose(Rotations.of(0.34136), Rotations.of(0.2535), Meters.of(0.01));
    public final ArmPose reefL3 = new ArmPose(Rotations.of(0.3459), Rotations.of(0.2524), Meters.of(0.45));
    public final ArmPose reefL4 = new ArmPose(Rotations.of(0.35706), Rotations.of(0.25586), Meters.of(1.051));

    // public static final ArmPose coralStation = new ArmPose(Degrees.of(0.0), Degrees.of(0.0), Meters.of(0.0));
    // public static final ArmPose processor = new ArmPose(Degrees.of(0.0), Degrees.of(0.0), Meters.of(0.0));
}