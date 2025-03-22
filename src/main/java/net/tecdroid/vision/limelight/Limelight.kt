@file:Suppress("unused")

package net.tecdroid.vision.limelight

import edu.wpi.first.math.geometry.*
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Frequency
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Time
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.util.Color
import frc.robot.LimelightHelpers
import net.tecdroid.util.*

data class LimelightConfig(
    val name: String,
    val offset: Translation3d,
)

class Limelight(private val config: LimelightConfig) : Sendable {
    private val table = NetworkTableInstance.getDefault().getTable(config.name)

    private fun getTableEntry(name: String) = table.getEntry(name)

    private fun getDouble(name: String) = getTableEntry(name).getDouble(0.0)
    private fun setDouble(name: String, value: Double) = getTableEntry(name).setDouble(value)

    private fun getString(name: String) = getTableEntry(name).getString("")
    private fun setString(name: String, value: String) = getTableEntry(name).setString(value)

    private fun getDoubleArray(name: String) = getTableEntry(name).getDoubleArray(DoubleArray(0))
    private fun setDoubleArray(name: String, value: DoubleArray) = getTableEntry(name).setDoubleArray(value)

    private fun getStringArray(name: String) = getTableEntry(name).getStringArray(Array<String>(0) { "" })

    private fun rawDataToPose2d(data: DoubleArray) = if (data.size < 6) Pose2d()
        else Pose2d(
            Translation2d(data[0].meters, data[1].meters),
            Rotation2d(data[5].degrees)
        )

    private fun rawDataToPose3d(data: DoubleArray) =  if (data.size < 6) Pose3d()
        else Pose3d(
            Translation3d(data[0].meters, data[1].meters, data[2].meters),
            Rotation3d(data[3].degrees, data[4].degrees, data[5].degrees)
        )

    fun setRobotOrientation(robotRotation: Double) {
        LimelightHelpers.SetRobotOrientation(
            config.name,
            robotRotation,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        )
    }

    fun setIDFilters(validIds : IntArray) {
        LimelightHelpers.SetFiducialIDFiltersOverride(config.name, validIds)
    }

    fun setCameraPose(forward : Double, right : Double, up : Double, roll : Double, pitch : Double, yaw : Double) {
        LimelightHelpers.setCameraPose_RobotSpace(
            config.name, // Nombre de la cámara Limelight
            forward,               // Desplazamiento hacia adelante en metros desde el centro del robot
            right,                  // Desplazamiento lateral en metros (positivo a la derecha)
            up,                    // Altura en metros desde el suelo
            roll,                  // Ángulo de rotación en el eje X en grados
            pitch,                 // Ángulo de rotación en el eje Y en grados
            yaw                    // Ángulo de rotación en el eje Z en grados
        );
    }

    val hasTarget: Boolean
        get() = getDouble(LimelightTableKeys.Get.hasValidTarget) == 1.0

    val horizontalOffset: Angle
        get() = getDouble(LimelightTableKeys.Get.horizontalOffsetDegrees).degrees

    val verticalOffset: Angle
        get() = getDouble(LimelightTableKeys.Get.verticalOffsetDegrees).degrees

    val horizontalPixelOffset: Pixels
        get() = Pixels.of(getDouble(LimelightTableKeys.Get.horizontalOffsetDegrees).toInt())

    val verticalPixelOffset: Pixels
        get() = Pixels.of(getDouble(LimelightTableKeys.Get.verticalOffsetDegrees).toInt())

    val targetAreaOccupancy: Percentage
        get() = Percentage(getDouble(LimelightTableKeys.Get.verticalOffsetDegrees))

    private val rawTargetDataR2: DoubleArray
        get() = getDoubleArray(LimelightTableKeys.Get.targetingDataR2)

    val targetDataR2: LimelightTargetDataR2
        get() = LimelightTargetDataR2.fromT2d(rawTargetDataR2)

    val targetCount: Int
        get() = targetDataR2.count

    val classifierClassIndex: Int
        get() = targetDataR2.classifierClassIndex

    val detectorClassIndex: Int
        get() = targetDataR2.detectorClassIndex

    val classifierClassName: String
        get() = getString(LimelightTableKeys.Get.classifierPipelineClassname)

    val detectorClassName: String
        get() = getString(LimelightTableKeys.Get.detectorPipelineClassname)

    val pipelineLatency: Time
        get() = getDouble(LimelightTableKeys.Get.pipelineLatency).milliseconds

    val captureLatency: Time
        get() = getDouble(LimelightTableKeys.Get.captureLatency).milliseconds

    val pipelineIndex: Int
        get() = getDouble(LimelightTableKeys.Get.pipelineIndex).toInt()

    val pipelineTypeName: String
        get() = getString(LimelightTableKeys.Get.pipelineType)

    val json: String
        get() = getString(LimelightTableKeys.Get.jsonOutput)

    private val rawRobotPosition: DoubleArray
        get() = getDoubleArray(LimelightTableKeys.Get.robotPosition)

    private val rawRobotPositionMt2: DoubleArray
        get() = getDoubleArray(LimelightTableKeys.Get.robotPositionMt2)

    val robotPosition: Pose3d
        get() = rawDataToPose3d(rawRobotPosition)

    val robotPositionMt2: Pose3d
        get() = rawDataToPose3d(rawRobotPositionMt2)

    private val rawRobotPositionInBlueFieldSpace: DoubleArray
        get() = getDoubleArray(LimelightTableKeys.Get.robotPositionInBlueFieldSpace)

    private val rawRobotPositionInBlueFieldSpaceMt2: DoubleArray
        get() = getDoubleArray(LimelightTableKeys.Get.robotPositionInBlueFieldSpaceMt2)

    val botPoseEstimate_wpiBlue_MegaTag2: LimelightHelpers.PoseEstimate
        get() = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name)

    val botPoseEstimate_wpiRed_MegaTag2: LimelightHelpers.PoseEstimate
        get() = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(config.name)

    val robotPositionInBlueFieldSpace: Pose3d
        get() = rawDataToPose3d(rawRobotPositionInBlueFieldSpace)

    val robotPositionInBlueFieldSpaceMt2: Pose3d
        get() = rawDataToPose3d(rawRobotPositionInBlueFieldSpaceMt2)

    private val rawRobotPositionInRedFieldSpace: DoubleArray
        get() = getDoubleArray(LimelightTableKeys.Get.robotPositionInRedFieldSpace)

    private val rawRobotPositionInRedFieldSpaceMt2: DoubleArray
        get() = getDoubleArray(LimelightTableKeys.Get.robotPositionInRedFieldSpaceMt2)

    val robotPositionInRedFieldSpace: Pose3d
        get() = rawDataToPose3d(rawRobotPositionInRedFieldSpace)

    val robotPositionInRedFieldSpaceMt2: Pose3d
        get() = rawDataToPose3d(rawRobotPositionInRedFieldSpaceMt2)

    public val rawRobotPositionInTargetSpace: DoubleArray
        get() = getDoubleArray(LimelightTableKeys.Get.robotPositionInTargetSpace)

    val robotPositionInTargetSpace: Pose3d
        get() = rawDataToPose3d(rawRobotPositionInTargetSpace)

    private val rawCameraPositionInTargetSpace: DoubleArray
        get() = getDoubleArray(LimelightTableKeys.Get.cameraPositionInTargetSpace)

    val cameraPositionInTargetSpace: Pose3d
        get() = rawDataToPose3d(rawCameraPositionInTargetSpace)

    private val rawTargetPositionInCameraSpace: DoubleArray
        get() = getDoubleArray(LimelightTableKeys.Get.targetPositionInCameraSpace)

    val targetPositionInCameraSpace: Pose3d
        get() = rawDataToPose3d(rawTargetPositionInCameraSpace)

    private val rawTargetPositionInRobotSpace: DoubleArray
        get() = getDoubleArray(LimelightTableKeys.Get.targetPositionInRobotSpace)

    val targetPositionInRobotSpace: Pose3d
        get() = rawDataToPose3d(rawTargetPositionInRobotSpace)

    private val rawColorUnderCrosshair: IntArray
        get() = getDoubleArray(LimelightTableKeys.Get.hsvAtCrosshair).map { it.toInt() }.toIntArray()

    val colorUnderCrosshair: Color
        get() {
            val data = rawColorUnderCrosshair
            return Color.fromHSV(data[0], data[1], data[2])
        }

    val targetId: Int
        get() = getDouble(LimelightTableKeys.Get.targetId).toInt()

    private val neuralClassName: String
        get() = getString(LimelightTableKeys.Get.neuralClassName)

    private val rawBarcodeData: Array<String>
        get() = getStringArray(LimelightTableKeys.Get.Raw.barcodeData)

    val offsetFromTarget: Pose3d
        get() = LimelightHelpers.getTargetPose3d_RobotSpace(config.name)

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

data class LimelightTargetDataR2(
    val hasValidTarget: Boolean,
    val count: Int,
    val latency: Time,
    val captureLatency: Time,
    val horizontalAngularOffset: Angle,
    val verticalAngularOffset: Angle,
    val horizontalLinearOffset: Pixels,
    val verticalLinearOffset: Pixels,
    val areaOccupancy: Percentage,
    val id: Int,
    val classifierClassIndex: Int,
    val detectorClassIndex: Int,
    val longSidePixels: Pixels,
    val shortSidePixels: Pixels,
    val horizontalExtentPixels: Pixels,
    val verticalExtentPixels: Pixels,
    val skew: Angle
) {
    companion object {
        fun fromT2d(t2d: DoubleArray) =
            if (t2d.size != 17)
                LimelightTargetDataR2(
                    hasValidTarget = false,
                    count = 0,
                    latency = 0.0.milliseconds,
                    captureLatency = 0.0.milliseconds,
                    horizontalAngularOffset = 0.0.degrees,
                    verticalAngularOffset = 0.0.degrees,
                    horizontalLinearOffset = 0.pixels,
                    verticalLinearOffset = 0.pixels,
                    areaOccupancy = 0.0.percent,
                    id = 0,
                    classifierClassIndex = 0,
                    detectorClassIndex = 0,
                    longSidePixels = 0.pixels,
                    shortSidePixels = 0.pixels,
                    horizontalExtentPixels = 0.pixels,
                    verticalExtentPixels = 0.pixels,
                    skew = 0.0.degrees
                )
            else
                LimelightTargetDataR2(
                    hasValidTarget = t2d[LimelightTableKeys.T2dIndices.targetValid] == 1.0,
                    count = t2d[LimelightTableKeys.T2dIndices.targetCount].toInt(),
                    latency = t2d[LimelightTableKeys.T2dIndices.targetLatency].milliseconds,
                    captureLatency = t2d[LimelightTableKeys.T2dIndices.captureLatency].milliseconds,
                    horizontalAngularOffset = t2d[LimelightTableKeys.T2dIndices.tx].degrees,
                    verticalAngularOffset = t2d[LimelightTableKeys.T2dIndices.ty].degrees,
                    horizontalLinearOffset = t2d[LimelightTableKeys.T2dIndices.txnc].toInt().pixels,
                    verticalLinearOffset = t2d[LimelightTableKeys.T2dIndices.tync].toInt().pixels,
                    areaOccupancy = t2d[LimelightTableKeys.T2dIndices.ta].percent,
                    id = t2d[LimelightTableKeys.T2dIndices.tid].toInt(),
                    classifierClassIndex = t2d[LimelightTableKeys.T2dIndices.targetClassIndexClassifier].toInt(),
                    detectorClassIndex = t2d[LimelightTableKeys.T2dIndices.targetClassIndexDetector].toInt(),
                    longSidePixels = t2d[LimelightTableKeys.T2dIndices.targetLongSidePixels].toInt().pixels,
                    shortSidePixels = t2d[LimelightTableKeys.T2dIndices.targetShortSidePixels].toInt().pixels,
                    horizontalExtentPixels = t2d[LimelightTableKeys.T2dIndices.targetHorizontalExtentPixels].toInt().pixels,
                    verticalExtentPixels = t2d[LimelightTableKeys.T2dIndices.targetVerticalExtentPixels].toInt().pixels,
                    skew = t2d[LimelightTableKeys.T2dIndices.targetSkewDegrees].degrees
                )
    }
}

data class LimelightPoseEstimate(
    val pose2d: Pose2d,
    val pose3d: Pose3d,
    val timestamp: Time,
    val latency: Time,
    val targetCount: Int,
    val tagSpan: Double,
    val averageTagDistance: Double,
    val averageTagArea: Double
) {
}

data class LimelightHardwareMetrics(
    val fps: Frequency,
    val cpuTemperature: Temperature,
    val temperature: Temperature,
    val ramUsage: Percentage
)
