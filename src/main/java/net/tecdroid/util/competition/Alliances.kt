package net.tecdroid.util.competition

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance

fun isBlueAlliance(): Boolean = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
fun isRedAlliance(): Boolean = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
