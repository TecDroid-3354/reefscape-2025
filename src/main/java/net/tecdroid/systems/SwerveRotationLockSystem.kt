package net.tecdroid.systems

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.drivetrain.SwerveDrive
import net.tecdroid.util.ControlGains
import net.tecdroid.util.degrees
import net.tecdroid.util.meters
import kotlin.math.max
import kotlin.math.min

enum class LockPositions {
    CoralStation,
    Proccesor
}

class SwerveRotationLockSystem (private val swerve: SwerveDrive, private val controller: CompliantXboxController){
    private val thetaGains = ControlGains(0.0075, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    private val thetaPIDController = PIDController(thetaGains.p, thetaGains.i, thetaGains.d)

    private fun getLimitedYaw(): Double {
        var limitedYaw: Double = swerve.heading.`in`(Units.Degrees) % 360
        if (limitedYaw < 0) {
            limitedYaw += 360.0
        }
        return limitedYaw
    }

    private fun clamp(max: Double, min: Double, v: Double): Double {
        return max(min, min(max, v))
    }

    // Change the target angle depending on the side on where the robot is
    fun getCoralStationAngleAccordingToRobotPosition() : Angle {
        val FIELD_WIDTH = 8.21.meters
        val y = swerve.poseEstimator.estimatedPosition.y

        // If is greater, the robot is on the right side
        if (y > FIELD_WIDTH.baseUnitMagnitude() / 2.0) {
            return 55.0.degrees
        } else {
            return 305.0.degrees
        }
    }


    fun lockRotationCMD(position: LockPositions) : Command = Commands.run({
        // Controller velocities
        val vx = MathUtil.applyDeadband(controller.leftY, 0.05) * 0.85
        val vy = MathUtil.applyDeadband(controller.leftX, 0.05) * 0.85

        val targetXVelocity = swerve.maxLinearVelocity * vx
        val targetYVelocity = swerve.maxLinearVelocity * vy

        // PID theta velocity

        // Get the target angle according to the target position
        val targetAngle = when (position) {
            LockPositions.CoralStation -> 140.0.degrees
            LockPositions.Proccesor -> 55.0.degrees
        }
        val wFactor = clamp(1.0, -1.0, thetaPIDController.calculate(getLimitedYaw(), targetAngle.`in`(Units.Degrees)))

        val targetWVelocity = Units.DegreesPerSecond.of(Math.toDegrees(swerve.maxSpeeds.omegaRadiansPerSecond.times(0.75)) * wFactor)

        swerve.driveFieldOriented(ChassisSpeeds(targetXVelocity, targetYVelocity, targetWVelocity))
    }, swerve)
}