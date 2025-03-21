package net.tecdroid.subsystems.drivetrain

import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.Hertz
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.constants.integratorTabName
import net.tecdroid.util.units.toRotation2d

class SwerveDriveDriver(
    private val maxLinearVelocity: LinearVelocity,
    private val maxAngularVelocity: AngularVelocity,
    accelerationPeriod: Time,
    decelerationPeriod: Time = accelerationPeriod
) {
    var longitudinalVelocityFactorSource = { 0.0 }
    var transversalVelocityFactorSource = { 0.0 }
    var angularVelocityFactorSource = { 0.0 }

    private val da = accelerationPeriod.asFrequency()
    private val dd = -decelerationPeriod.asFrequency()

    private val longitudinalRateLimiter = SlewRateLimiter(da.`in`(Hertz), dd.`in`(Hertz), 0.0)
    private val transversalRateLimiter = SlewRateLimiter(da.`in`(Hertz), dd.`in`(Hertz), 0.0)
    private val angularRateLimiter = SlewRateLimiter(da.`in`(Hertz), dd.`in`(Hertz), 0.0)

    private var orientation = DriveOrientation.FieldOriented

    private val isFieldOriented: Boolean
        get() = orientation == DriveOrientation.FieldOriented

    fun obtainTargetSpeeds(currentAngle: Angle): ChassisSpeeds {
        val xf = longitudinalVelocityFactorSource()
        val yf = transversalVelocityFactorSource()
        val wf = angularVelocityFactorSource()

        val vx = maxLinearVelocity * longitudinalRateLimiter.calculate(xf)
        val vy = maxLinearVelocity * transversalRateLimiter.calculate(yf)
        val vw = maxAngularVelocity * angularRateLimiter.calculate(wf)

        return if (isFieldOriented) ChassisSpeeds.fromFieldRelativeSpeeds(
            vx,
            vy,
            vw,
            currentAngle.toRotation2d()
        ) else ChassisSpeeds(vx, vy, vw)
    }

    fun toggleOrientation() {
        orientation = when (orientation) {
            DriveOrientation.FieldOriented -> DriveOrientation.RobotOriented
            DriveOrientation.RobotOriented -> DriveOrientation.FieldOriented
        }
    }

    fun toggleOrientationCommand() : Command = Commands.runOnce(::toggleOrientation)

    fun setFieldOriented() {
        orientation = DriveOrientation.FieldOriented
    }

    fun setFieldOrientedCommand() : Command = Commands.runOnce(::setFieldOriented)

    fun setRobotOriented() {
        orientation = DriveOrientation.RobotOriented
    }

    fun setRobotOrientedCommand() : Command = Commands.runOnce(::setRobotOriented)

    enum class DriveOrientation {
        FieldOriented,
        RobotOriented
    }
}