package net.tecdroid.vision.limelight

import edu.wpi.first.math.geometry.*
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Frequency
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Time
import net.tecdroid.util.units.*

data class LimelightConfig(
    val name: String,
    val offset: Translation3d,
)

class Limelight(private val config: LimelightConfig) {
    private val table = NetworkTableInstance.getDefault().getTable(config.name)

    private fun getTableEntry(name: String) = table.getEntry(name)

    private fun getDouble(name: String) = getTableEntry(name).getDouble(0.0)
    private fun setDouble(name: String, value: Double) = getTableEntry(name).setDouble(value)

    private fun getString(name: String) = getTableEntry(name).getString("")
    private fun setString(name: String, value: String) = getTableEntry(name).setString(value)

    private fun getDoubleArray(name: String) = getTableEntry(name).getDoubleArray(DoubleArray(0))
    private fun setDoubleArray(name: String, value: DoubleArray) = getTableEntry(name).setDoubleArray(value)

    private fun getStringArray(name: String) = getTableEntry(name).getStringArray(Array<String>(0) { "" })

    //
    // Target Validity
    //

    /**
     * Determines if a valid target is in sight
     */
    val hasTarget: Boolean
        get() = getDouble(LimelightTableKeys.Get.hasValidTarget) == 1.0

    /**
     * Determines if more than one valid target is in sight
     */
    val hasMultipleTargets: Boolean
        get() = targetCount > 1

    /**
     * Returns the amount of targets currently in the camera's view
     */
    val targetCount: Int
        get() = detectionMetrics.count

    //
    // Basic Vision Data
    //

    /**
     * Obtains the horizontal angular offset from the center of the target
     */
    val horizontalOffset: Angle
        get() = getDouble(LimelightTableKeys.Get.horizontalOffsetDegrees).degrees

    /**
     * Obtains the vertical angular offset from the center of the target
     */
    val verticalOffset: Angle
        get() = getDouble(LimelightTableKeys.Get.verticalOffsetDegrees).degrees

    /**
     * Obtains what percentage of the camera's vision is being occupied by the target
     */
    val targetAreaOccupancy: Percentage
        get() = Percentage(getDouble(LimelightTableKeys.Get.targetAreaOccupancyPercentage))

    //
    // Positions in Field Space
    //

    /**
     * Returns the robot's position with the origin at the middle of the field
     */
    val robotPosition: Pose3d
        get() = rawDataToPose3d(getDoubleArray(LimelightTableKeys.Get.robotPosition))

    /**
     * Returns the robot's position with the origin at the middle of the field. Uses MegaTag2
     */
    val robotPositionMt2: Pose3d
        get() = rawDataToPose3d(getDoubleArray(LimelightTableKeys.Get.robotPositionMt2))

    /**
     * Returns the robot's position with the origin at the blue alliance origin
     */
    val robotPositionInBlueFieldSpace: Pose3d
        get() = rawDataToPose3d(getDoubleArray(LimelightTableKeys.Get.robotPositionInBlueFieldSpace))

    /**
     * Returns the robot's position with the origin at the blue alliance origin. Uses MegaTag2
     */
    val robotPositionInBlueFieldSpaceMt2: Pose3d
        get() = rawDataToPose3d(getDoubleArray(LimelightTableKeys.Get.robotPositionInBlueFieldSpaceMt2))

    /**
     * Returns the robot's position with the origin at the red alliance origin and rotated 180 degrees
     */
    val robotPositionInRedFieldSpace: Pose3d
        get() = rawDataToPose3d(getDoubleArray(LimelightTableKeys.Get.robotPositionInRedFieldSpace))

    /**
     * Returns the robot's position with the origin at the red alliance origin and rotated 180 degrees. Uses MegaTag2
     */
    val robotPositionInRedFieldSpaceMt2: Pose3d
        get() = rawDataToPose3d(getDoubleArray(LimelightTableKeys.Get.robotPositionInRedFieldSpaceMt2))

    //
    // Positions in Target Space
    //

    val robotPositionInTargetSpace: Pose3d
        get() = rawDataToPose3d(getDoubleArray(LimelightTableKeys.Get.robotPositionInTargetSpace))

    val cameraPositionInTargetSpace: Pose3d
        get() = rawDataToPose3d(getDoubleArray(LimelightTableKeys.Get.cameraPositionInTargetSpace))

    //
    // Positions in Robot Space
    //

    val targetPositionInRobotSpace: Pose3d
        get() = rawDataToPose3d(getDoubleArray(LimelightTableKeys.Get.targetPositionInRobotSpace))

    val cameraPositionInRobotSpace: Pose3d
        get() = rawDataToPose3d(getDoubleArray(LimelightTableKeys.Get.cameraPositionInRobotSpace))

    //
    // Positions in Camera Space
    //

    val targetPositionInCameraSpace: Pose3d
        get() = rawDataToPose3d(getDoubleArray(LimelightTableKeys.Get.targetPositionInCameraSpace))

    /**
     * Provides a summary of the camera's current detection metrics
     */
    val detectionMetrics: LimelightDetectionMetrics2d
        get() = LimelightDetectionMetrics2d.fromT2d(getDoubleArray(LimelightTableKeys.Get.targetingDataR2))

    // Subset Member, work on this later
    val classifierClassName: String
        get() = getString(LimelightTableKeys.Get.classifierPipelineClassname)

    // Subset Member, work on this later
    val detectorClassName: String
        get() = getString(LimelightTableKeys.Get.detectorPipelineClassname)

    /**
     * The pipeline latency
     */
    val pipelineLatency: Time
        get() = getDouble(LimelightTableKeys.Get.pipelineLatency).milliseconds

    /**
     * The capture latency
     */
    val captureLatency: Time
        get() = getDouble(LimelightTableKeys.Get.captureLatency).milliseconds

    /**
     * The total latency
     */
    val latency: Time
        get() = pipelineLatency + captureLatency

    /**
     * The current pipeline index
     */
    var pipelineIndex: Int
        get() = getDouble(LimelightTableKeys.Get.pipelineIndex).toInt()
        set(value) { setDouble(LimelightTableKeys.Set.pipelineIndex, value.toDouble()) }

    // Make into enum later
    val pipelineTypeName: String
        get() = getString(LimelightTableKeys.Get.pipelineType)

    // Wrap in output later
    val json: String
        get() = getString(LimelightTableKeys.Get.jsonOutput)

    val colorUnderCrosshair: LimelightHsv
        get() = LimelightHsv.fromTc(getDoubleArray(LimelightTableKeys.Get.hsvAtCrosshair))

    val targetId: Int
        get() = getDouble(LimelightTableKeys.Get.targetId).toInt()

    private val neuralClassName: String
        get() = getString(LimelightTableKeys.Get.neuralClassName)

    private val rawBarcodeData: Array<String>
        get() = getStringArray(LimelightTableKeys.Get.Raw.barcodeData)

    val heartbeat: Double
        get() = getDouble(LimelightTableKeys.Get.heartbeat)

    val hardwareMetrics: LimelightHardwareMetrics
        get() = LimelightHardwareMetrics.fromHw(getDoubleArray(LimelightTableKeys.Get.hardwareMetrics))

    val crosshairPosition: Pair<Translation2d, Translation2d>
        get() {
            val data = getDoubleArray(LimelightTableKeys.Get.crosshairPosition)
            return Translation2d(data[0], data[1]) to Translation2d(data[2], data[3])
        }

    val megaTagStandardDeviations: Double
        get() = getDouble(LimelightTableKeys.Get.megaTagStandardDeviations)


    /**
     * Obtains the horizontal pixel offset from the center of the target
     */
    val horizontalPixelOffset: Pixels
        get() = getDouble(LimelightTableKeys.Get.horizontalOffsetPixels).toInt().pixels

    /**
     * Obtains the vertical pixel offset from the center of the target
     */
    val verticalPixelOffset: Pixels
        get() = getDouble(LimelightTableKeys.Get.verticalOffsetPixels).toInt().pixels

}

internal fun rawDataToPose2d(data: DoubleArray) =
    if (data.size < 6) Pose2d()
    else Pose2d(
        Translation2d(data[0].meters, data[1].meters),
        Rotation2d(data[5].degrees)
    )

internal fun rawDataToPose3d(data: DoubleArray) =
    if (data.size < 6) Pose3d()
    else Pose3d(
        Translation3d(data[0].meters, data[1].meters, data[2].meters),
        Rotation3d(data[3].degrees, data[4].degrees, data[5].degrees)
    )

internal fun pose3dToRawData(pose3d: Pose3d) = arrayOf(
    pose3d.x, pose3d.y, pose3d.z,
    pose3d.rotation.x, pose3d.rotation.y, pose3d.rotation.z
).toDoubleArray()

internal fun pose3dToPose2d(pose3d: Pose3d) = Pose2d(
    Translation2d(pose3d.measureX, pose3d.measureY),
    Rotation2d(pose3d.rotation.z)
)

data class LimelightHsv(
    val hue: Int = 0,
    val saturation: Int = 0,
    val value: Int = 0
) {
    companion object {
        fun fromTc(tc: DoubleArray): LimelightHsv = LimelightHsv(
            hue = tc[LimelightTableKeys.TcIndices.hue].toInt(),
            saturation = tc[LimelightTableKeys.TcIndices.saturation].toInt(),
            value = tc[LimelightTableKeys.TcIndices.value].toInt(),
        )
    }
}

data class LimelightDetectionMetrics2d(
    val isValidData: Boolean = false,
    val hasValidTarget: Boolean = false,
    val count: Int = 0,
    val latency: Time = 0.0.milliseconds,
    val captureLatency: Time = 0.0.milliseconds,
    val horizontalAngularOffset: Angle = 0.0.degrees,
    val verticalAngularOffset: Angle = 0.0.degrees,
    val horizontalLinearOffset: Pixels = 0.pixels,
    val verticalLinearOffset: Pixels = 0.pixels,
    val areaOccupancy: Percentage = 0.0.percent,
    val id: Int = 0,
    val classifierClassIndex: Int = 0,
    val detectorClassIndex: Int = 0,
    val longSidePixels: Pixels = 0.pixels,
    val shortSidePixels: Pixels = 0.pixels,
    val horizontalExtentPixels: Pixels = 0.pixels,
    val verticalExtentPixels: Pixels = 0.pixels,
    val skew: Angle = 0.0.degrees
) {
    companion object {
        fun fromT2d(t2d: DoubleArray) =
            if (t2d.size != 17) LimelightDetectionMetrics2d()
            else LimelightDetectionMetrics2d(
                isValidData = true,
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
    val isValidEstimate: Boolean,
    val targetCount: Int,
    val tagSpan: Double,
    val averageTagDistance: Double,
    val averageTagArea: Double
)

data class LimelightHardwareMetrics(
    val fps: Frequency,
    val cpuTemperature: Temperature,
    val temperature: Temperature,
    val ramUsage: Percentage
) {
    companion object {
        fun fromHw(value: DoubleArray): LimelightHardwareMetrics =
            LimelightHardwareMetrics(
                fps = value[0].hertz,
                cpuTemperature = value[1].degreesCelsius,
                ramUsage = value[2].percent,
                temperature = value[3].degreesCelsius
            )
    }
}
