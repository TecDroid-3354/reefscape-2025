package net.tecdroid.vision.limelight

import edu.wpi.first.math.geometry.*
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Frequency
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.util.Color
import net.tecdroid.util.units.*

data class LimelightConfig(
    val name: String,
    val offset: Translation3d,
)

abstract class LimelightBase(private val config: LimelightConfig) {
    private val table = NetworkTableInstance.getDefault().getTable(config.name)

    private fun getTableEntry(name: String) = table.getEntry(name)

    protected fun getDouble(name: String) = getTableEntry(name).getDouble(0.0)
    protected fun setDouble(name: String, value: Double) = getTableEntry(name).setDouble(value)

    protected fun getString(name: String) = getTableEntry(name).getString("")
    protected fun setString(name: String, value: String) = getTableEntry(name).setString(value)

    protected fun getDoubleArray(name: String) = getTableEntry(name).getDoubleArray(DoubleArray(0))
    protected fun setDoubleArray(name: String, value: DoubleArray) = getTableEntry(name).setDoubleArray(value)

    protected fun getStringArray(name: String) = getTableEntry(name).getStringArray(Array(0) { "" })
}

open class Limelight(config: LimelightConfig) : LimelightBase(config) {
    //
    // Target Validity
    //

    /**
     * Determines if a valid target is in sight
     */
    val hasTarget: Boolean
        get() = getDouble(LimelightTableKeys.Get.hasValidTarget) == 1.0

//    /**
//     * Determines if more than one valid target is in sight
//     */
//    val hasMultipleTargets: Boolean
//        get() = targetCount > 1

//    /**
//     * Returns the amount of targets currently in the camera's view
//     */
//    val targetCount: Int
//        get() = detectionState.count

    //
    // Basic Vision Data
    //

    /**
     * Obtains the horizontal pixel offset from the center of the target
     */
    val horizontalPixelOffset: Pixels
        get() = getDouble(LimelightTableKeys.Get.horizontalOffsetPixels).toInt().pixels

    /**
     * Obtains the horizontal angular offset from the center of the target
     */
    val horizontalOffset: Angle
        get() = getDouble(LimelightTableKeys.Get.horizontalOffsetDegrees).degrees

    /**
     * Obtains the vertical pixel offset from the center of the target
     */
    val verticalPixelOffset: Pixels
        get() = getDouble(LimelightTableKeys.Get.verticalOffsetPixels).toInt().pixels

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
    // Hardware data
    //

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
     * The device's heartbeat
     */
    val heartbeat: Double
        get() = getDouble(LimelightTableKeys.Get.heartbeat)

    /**
     * A snapshot of the current hardware state
     */
    val hardwareMetrics: LimelightHardwareMetrics
        get() = LimelightHardwareMetrics.fromHw(getDoubleArray(LimelightTableKeys.Get.hardwareMetrics))

    //
    // Pipeline Data
    //

    /**
     * The current pipeline index
     */
    var pipelineIndex: Int
        get() = getDouble(LimelightTableKeys.Get.pipelineIndex).toInt()
        set(value) { setDouble(LimelightTableKeys.Set.pipelineIndex, value.toDouble()) }

    // TODO: Convert to enumeration
    /**
     * Obtains the name of the current pipeline
     */
    val pipelineTypeName: String
        get() = getString(LimelightTableKeys.Get.pipelineType)

    //
    // Overlay data
    //

    /**
     * The position of both crosshairs
     */
    val crosshairs: LimelightCrosshairs
        get() = LimelightCrosshairs.fromRawCrosshairData(getDoubleArray(LimelightTableKeys.Get.crosshairPosition))

    /**
     * Returns the average color of the 3x3 pixel grid formed at the crosshair
     */
    val colorAtCrosshair: Color
        get() = rawDataToColor(getDoubleArray(LimelightTableKeys.Get.hsvAtCrosshair))

    //
    // Uncategorized
    //

    /**
     * JSON output of limelight readings
     */
    val json: String
        get() = getString(LimelightTableKeys.Get.jsonOutput)
}

class LimelightAprilTagDetector(config: LimelightConfig): Limelight(config) {
    //
    // Basic AprilTag Data
    //

    /**
     * Returns the id of the target in sight
     */
    val targetId: Int
        get() = getDouble(LimelightTableKeys.Get.targetId).toInt()

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

    var cameraPositionInRobotSpace: Pose3d
        get() = rawDataToPose3d(getDoubleArray(LimelightTableKeys.Get.cameraPositionInRobotSpace))
        set(value) { setDoubleArray(LimelightTableKeys.Set.cameraPositionInRobotSpace, pose3dToRawData(value)) }

    //
    // Positions in Camera Space
    //

    val targetPositionInCameraSpace: Pose3d
        get() = rawDataToPose3d(getDoubleArray(LimelightTableKeys.Get.targetPositionInCameraSpace))

    //
    // Miscellaneous
    //

    val megaTagStandardDeviations: Double
        get() = getDouble(LimelightTableKeys.Get.megaTagStandardDeviations)

    /**
     * Provides a summary of the camera's current detection metrics
     */
    val detectionState: LimelightDetectionState2d
        get() = LimelightDetectionState2d.fromT2d(getDoubleArray(LimelightTableKeys.Get.targetingDataR2))


}

abstract class LimelightNeuralBase<T>(config: LimelightConfig, protected val computingMethod: (String) -> T) : Limelight(config) {
    /**
     * Primary neural detector or classifier class name
     */
    val generalNeuralClassName: String
        get() = getString(LimelightTableKeys.Get.neuralClassName)

    /**
     * Transforms the targeted detection or classification into its corresponding object
     */
    abstract fun computeResult(classname: String): T
}

class LimelightNeuralClassifier<T>(config: LimelightConfig, computingMethod: (String) -> T) : LimelightNeuralBase<T>(config, computingMethod) {
    /**
     * Classifier's computed class name
     */
    private val classifierClassName: String
        get() = getString(LimelightTableKeys.Get.classifierPipelineClassname)

    override fun computeResult(classname: String): T = computingMethod(classifierClassName)
}

class LimelightNeuralDetector<T>(config: LimelightConfig, computingMethod: (String) -> T) : LimelightNeuralBase<T>(config, computingMethod) {
    /**
     * Detector's primary detection name
     */
    private val detectorClassName: String
        get() = getString(LimelightTableKeys.Get.detectorPipelineClassname)

    override fun computeResult(classname: String): T = computingMethod(detectorClassName)

}

class LimelightBarcodeDetector(config: LimelightConfig): Limelight(config) {
    /**
     * Raw barcode data
     */
    val barcodeData: Array<String>
        get() = getStringArray(LimelightTableKeys.Get.Raw.barcodeData)
}

//
// Helper functions
//

/**
 * Converts the pose [DoubleArray] returned by a [Limelight] to a [Pose3d]
 */
internal fun rawDataToPose3d(data: DoubleArray): Pose3d =
    if (data.size < 6) Pose3d()
    else Pose3d(
        Translation3d(data[0].meters, data[1].meters, data[2].meters),
        Rotation3d(data[3].degrees, data[4].degrees, data[5].degrees)
    )

/**
 * Converts the pose [DoubleArray] returned by a [Limelight] to a [Pose2d]
 */
internal fun rawDataToPose2d(data: DoubleArray) = pose3dToPose2d(rawDataToPose3d(data))

/**
 * Converts a [Pose3d] into a [DoubleArray] for use with a [Limelight]
 */
internal fun pose3dToRawData(pose3d: Pose3d) = arrayOf(
    pose3d.x, pose3d.y, pose3d.z,
    pose3d.rotation.x, pose3d.rotation.y, pose3d.rotation.z
).toDoubleArray()

/**
 * Truncates a [Pose3d] to a [Pose2d]
 */
internal fun pose3dToPose2d(pose3d: Pose3d) = Pose2d(
    Translation2d(pose3d.measureX, pose3d.measureY),
    Rotation2d(pose3d.rotation.z)
)

/**
 * Converts the [DoubleArray] of color under the crosshair of a [Limelight] to a [Color] object
 */
internal fun rawDataToColor(data: DoubleArray): Color =
    Color.fromHSV(data[LimelightTableKeys.TcIndices.hue].toInt(),
                  data[LimelightTableKeys.TcIndices.saturation].toInt(),
                  data[LimelightTableKeys.TcIndices.value].toInt()
    )

data class LimelightCrosshairs(
    val positionOne: Translation2d,
    val positionTwo: Translation2d
) {
    val hasSingleCrosshair = positionOne == positionTwo

    companion object {
        fun fromRawCrosshairData(data: DoubleArray) = LimelightCrosshairs(
            positionOne = Translation2d(data[0], data[1]),
            positionTwo = Translation2d(data[2], data[3])
        )
    }
}

data class LimelightDetectionState2d(
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
            if (t2d.size != 17) LimelightDetectionState2d()
            else LimelightDetectionState2d(
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
