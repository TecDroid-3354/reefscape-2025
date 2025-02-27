package net.tecdroid.subsystems.drivetrain

import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import net.tecdroid.kt.toRotation2d
import net.tecdroid.math.sgn
import kotlin.math.abs

class SwerveDriveDriver(
    private val maxLinearVelocity: LinearVelocity,
    private val maxAngularVelocity: AngularVelocity,
    private val accelerationPeriod: Time,
    private val decelerationPeriod: Time
) {
    var longitudinalVelocityFactorSource = { 0.0 }
    var transversalVelocityFactorSource = { 0.0 }
    var angularVelocityFactorSource = { 0.0 }

    private val longitudinalRateLimiter =
        SlewRateLimiter(1.0 / accelerationPeriod.`in`(Seconds))

    private val transversalRateLimiter =
        SlewRateLimiter(1.0 / accelerationPeriod.`in`(Seconds))

    private val angularRateLimiter =
        SlewRateLimiter(1.0 / accelerationPeriod.`in`(Seconds))

    private var orientation = DriveOrientation.FieldOriented

    private val isFieldOriented: Boolean
        get() = orientation == DriveOrientation.FieldOriented

    init {
        this.longitudinalVelocityFactorSource = { 0.0 }
        this.transversalVelocityFactorSource = { 0.0 }
        this.angularVelocityFactorSource = { 0.0 }
    }


    fun obtainTargetSpeeds(currentAngle: Angle): ChassisSpeeds {
        val xf = longitudinalVelocityFactorSource()
        val yf = transversalVelocityFactorSource()
        val wf = angularVelocityFactorSource()

        val xMagDir = abs(xf) to sgn(xf)
        val yMagDir = abs(yf) to sgn(yf)
        val wMagDir = abs(wf) to sgn(wf)

        val vx = maxLinearVelocity * longitudinalRateLimiter.calculate(xf)
        val vy = maxLinearVelocity * transversalRateLimiter.calculate(yf)
        val w = maxAngularVelocity * angularRateLimiter.calculate(wf)

        SmartDashboard.putString("vx", vx.toString())
        SmartDashboard.putString("vy", vy.toString())
        SmartDashboard.putString("vw", w.toString())

        return if (isFieldOriented) ChassisSpeeds.fromFieldRelativeSpeeds(
            vx,
            vy,
            w,
            currentAngle.toRotation2d()
        ) else ChassisSpeeds(vx, vy, w)
    }

    fun toggleOrientation() {
        orientation = when (orientation) {
            DriveOrientation.FieldOriented -> DriveOrientation.RobotOriented
            DriveOrientation.RobotOriented -> DriveOrientation.FieldOriented
        }
    }

    enum class DriveOrientation {
        FieldOriented,
        RobotOriented
    }
}