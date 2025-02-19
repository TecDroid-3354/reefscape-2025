package net.tecdroid.kt

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.measure.Angle

fun Angle.r2d() = Rotation2d(this)