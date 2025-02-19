package net.tecdroid.subsystems.drivetrain

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity

class SwerveDriveDriver {
    private var forwardsVelocitySupplier: () -> LinearVelocity
    private var sidewaysVelocitySupplier: () -> LinearVelocity
    private var angularVelocitySupplier: () -> AngularVelocity
    private var orientation = DriveOrientation.FieldOriented

    private val isFieldOriented: Boolean
        get() = orientation == DriveOrientation.FieldOriented

    init {
        this.forwardsVelocitySupplier = { MetersPerSecond.zero() }
        this.sidewaysVelocitySupplier = { MetersPerSecond.zero() }
        this.angularVelocitySupplier = { RadiansPerSecond.zero() }
    }

    fun obtainTargetSpeeds(currentAngle: Rotation2d): ChassisSpeeds {
        val vx = forwardsVelocitySupplier()
        val vy = sidewaysVelocitySupplier()
        val w = angularVelocitySupplier()

        return if (isFieldOriented) ChassisSpeeds.fromFieldRelativeSpeeds(
            vx,
            vy,
            w,
            currentAngle
        ) else ChassisSpeeds(vx, vy, w)
    }

    fun toggleOrientation() {
        orientation = when (orientation) {
            DriveOrientation.FieldOriented -> DriveOrientation.RobotOriented
            DriveOrientation.RobotOriented -> DriveOrientation.FieldOriented
        }
    }

    fun setLongitudinalVelocitySupplier(forwardsVelocitySupplier: () -> LinearVelocity) {
        this.forwardsVelocitySupplier = forwardsVelocitySupplier
    }

    fun setTransversalVelocitySupplier(sidewaysVelocitySupplier: () -> LinearVelocity) {
        this.sidewaysVelocitySupplier = sidewaysVelocitySupplier
    }

    fun setAngularVelocitySupplier(angularVelocitySupplier: () -> AngularVelocity) {
        this.angularVelocitySupplier = angularVelocitySupplier
    }

    enum class DriveOrientation {
        FieldOriented,
        RobotOriented
    }
}