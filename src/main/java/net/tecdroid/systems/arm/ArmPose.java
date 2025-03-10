package net.tecdroid.systems.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public record ArmPose(Angle wristAngle, Angle jointAngle, Distance elevatorExtension) {}
