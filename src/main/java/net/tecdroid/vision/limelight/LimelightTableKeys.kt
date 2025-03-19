package net.tecdroid.vision.limelight


object LimelightTableKeys {

    object Get {
        const val hasValidTarget = "tv"
        const val targetId = "tid"
        const val horizontalOffsetDegrees = "tx"
        const val verticalOffsetDegrees = "ty"
        const val targetAreaOccupancyPercentage = "ta"
        const val targetingDataR2 = "t2d"
        const val pipelineLatency = "tl"
        const val captureLatency = "cl"
        const val pipelineIndex = "getpipe"
        const val pipelineType = "getpipetype"
        const val jsonOutput = "json"
        const val neuralClassName = "tclass"
        const val hsvAtCrosshair = "tc"
        const val heartbeat = "hb"
        const val hardwareMetrics = "hw"
        const val crosshairPosition = "crosshairs"
        const val classifierPipelineClassname = "tcclass"
        const val detectorPipelineClassname = "tdclass"
        const val robotPosition = "botpose"
        const val robotPositionInBlueFieldSpace = "botpose_wpiblue"
        const val robotPositionInRedFieldSpace = "botpose_wpired"
        const val robotPositionMt2 = "botpose_orb"
        const val robotPositionInBlueFieldSpaceMt2 = "botpose_orb_wpiblue"
        const val robotPositionInRedFieldSpaceMt2 = "botpose_orb_wpired"
        const val megaTagStandardDeviations = "stddevs"
        const val cameraPositionInTargetSpace = "camerapose_targetspace"
        const val targetPositionInCameraSpace = "targetpose_cameraspace"
        const val targetPositionInRobotSpace = "targetpose_robotspace"
        const val robotPositionInTargetSpace = "botpose_targetspace"
        const val cameraPositionInRobotSpace = "camerapose_robotspace"

        object Raw {
            const val horizontalOffsetPixels = "txnc"
            const val verticalOffsetPixels = "tync"
            const val cornerData = "tcornxy"
            const val targetData = "rawtargets"
            const val fiducialData = "rawfiducials"
            const val detectionData = "rawdetections"
            const val barcodeData = "rawbarcodes"
        }
    }

    object Set {
        const val cameraPositionInRobotSpace = "camerapose_robotspace_set"
        const val priorityId = "priorityid"
        const val robotOrientation = "robot_orientation_set"
        const val idFilter = "fiducial_id_filters_set"
        const val pointOfInterestOffset = "fiducial_offset_set"
        const val ledMode = "ledMode"
        const val pipelineIndex = "pipeline"
        const val streamingMode = "stream"
        const val cropParameters = "crop"
        const val throttle = "throttle_set"
        const val imuMode = "imumode_set"
        const val imuAssistAlpha = "imuassistalpha_set"
    }

}

object LimelightIndices {
    object T2d {
        val targetValid = 0
        val targetCount = 1
        val targetLatency = 2
        val captureLatency = 3
        val tx = 4
        val ty = 5
        val txnc = 6
        val tync = 7
        val ta = 8
        val tid = 9
        val targetClassIndexDetector = 10
        val targetClassIndexClassifier = 11
        val targetLongSidePixels = 12
        val targetShortSidePixels = 13
        val targetHorizontalExtentPixels = 14
        val targetVerticalExtentPixels = 15
        val targetSkewDegrees = 16
    }

    object T3d {
        val x = 0
        val y = 1
        val z = 2
        val roll = 3
        val pitch = 4
        val yaw = 5
        val latency = 6
        val tagCount = 7
        val tagSpan = 8
        val averageTagDistanceFromCamera = 9
        val averageTagCoverage = 10
    }

    object Tc {
        val hue = 0
        val saturation = 1
        val value = 2
    }

    object TCornXY {
        val entriesPerCorner = 4
        val x0 = 0
        val y0 = 1
        val x1 = 2
        val y1 = 3
        val x2 = 4
        val y2 = 5
        val x3 = 6
        val y3 = 7
    }

    object RawTargets {
        val entriesPerDetection = 3
        val txnc = 0
        val tync = 1
        val ta = 2
    }

    object RawFiducials {
        val entriesPerDetection = 7
        val id = 0
        val txnc = 1
        val tync = 2
        val ta = 3
        val distToCamera = 4
        val distToRobot = 5
        val ambiguity = 6
    }
    object RawDetections {
        val entriesPerDetection = 12
        val id = 0
        val txnc = 1
        val tync = 2
        val ta = 3
        val x0 = 4
        val y0 = 5
        val x1 = 6
        val y1 = 7
        val x2 = 8
        val y2 = 9
        val x3 = 10
        val y3 = 11
    }
}
