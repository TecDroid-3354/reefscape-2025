package net.tecdroid.vision.limelight

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import frc.robot.LimelightHelpers
import net.tecdroid.util.units.degrees

data class LimelightConfig(
    val name: String,
)

class Limelight(private val config: LimelightConfig) : Sendable {
    val offsetFromTarget: Pose3d
        get() = LimelightHelpers.getTargetPose3d_RobotSpace(config.name)

    val getHorizontalAngleOffset: Angle
        get() = LimelightHelpers.getTX(config.name).degrees

    val hasTarget: Boolean
        get() = LimelightHelpers.getTV(config.name)

    fun getTargetId(): Int = LimelightHelpers.getFiducialID(config.name).toInt()

    override fun initSendable(builder: SendableBuilder) {
        with(builder) {
            builder.addDoubleProperty("X", { offsetFromTarget.x }) {}
            builder.addDoubleProperty("Y", { offsetFromTarget.y }) {}
            builder.addDoubleProperty("Z", { offsetFromTarget.z }) {}
            builder.addDoubleProperty("Roll", { offsetFromTarget.rotation.x }) {}
            builder.addDoubleProperty("Pitch", { offsetFromTarget.rotation.y }) {}
            builder.addDoubleProperty("Yaw", { offsetFromTarget.rotation.z }) {}
        }
    }

}

