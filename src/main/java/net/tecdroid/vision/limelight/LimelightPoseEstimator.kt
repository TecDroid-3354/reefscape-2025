package net.tecdroid.vision.limelight

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import frc.robot.LimelightHelpers

data class LimelightConfig(
    val name: String,
)

class Limelight(private val config: LimelightConfig) : Sendable {
    val pose: Pose3d
        get() = getDistanceToRobotCenter()

    fun hasTarget() : Boolean = LimelightHelpers.getTV(config.name)

    fun getDistanceToRobotCenter(): Pose3d = LimelightHelpers.getTargetPose3d_RobotSpace(config.name)
    fun getHorizontalOffset() = LimelightHelpers.getTX(config.name)

    override fun initSendable(builder: SendableBuilder) {
        with(builder) {
            builder.addDoubleProperty("X", { pose.x }) {}
            builder.addDoubleProperty("Y", { pose.y }) {}
            builder.addDoubleProperty("Z", { pose.z }) {}
            builder.addDoubleProperty("Roll", { pose.rotation.x }) {}
            builder.addDoubleProperty("Pitch", { pose.rotation.y }) {}
            builder.addDoubleProperty("Yaw", { pose.rotation.z }) {}
        }
    }

}

