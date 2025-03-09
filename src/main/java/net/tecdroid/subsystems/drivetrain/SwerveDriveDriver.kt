package net.tecdroid.subsystems.drivetrain

import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.Hertz
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.kt.toRotation2d

class SwerveDriveDriver(
    private val maxLinearVelocity: LinearVelocity,
    private val maxAngularVelocity: AngularVelocity,
    private val accelerationPeriod: Time,
    private val decelerationPeriod: Time = accelerationPeriod
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

    fun toggleOrientationCommand() : Command {
        return Commands.runOnce({
            toggleOrientation()
        })
    }

    fun createDefaultCommand(swerveDrive: SwerveDrive) {
        val heading = swerveDrive.heading

        swerveDrive.defaultCommand = Commands.run({
            swerveDrive.drive(obtainTargetSpeeds(heading))
        }, swerveDrive)
    }

    fun getMaxAngularVelocity() : AngularVelocity {
        return maxAngularVelocity;
    }

    fun getMaxLinearVelocity() : LinearVelocity {
        return maxLinearVelocity;
    }

    enum class DriveOrientation {
        FieldOriented,
        RobotOriented
    }
}