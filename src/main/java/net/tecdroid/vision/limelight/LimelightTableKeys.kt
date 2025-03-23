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
        const val targetValid = 0
        const val targetCount = 1
        const val targetLatency = 2
        const val captureLatency = 3
        const val tx = 4
        const val ty = 5
        const val txnc = 6
        const val tync = 7
        const val ta = 8
        const val tid = 9
        const val targetClassIndexDetector = 10
        const val targetClassIndexClassifier = 11
        const val targetLongSidePixels = 12
        const val targetShortSidePixels = 13
        const val targetHorizontalExtentPixels = 14
        const val targetVerticalExtentPixels = 15
        const val targetSkewDegrees = 16
    }

    object T3d {
        const val x = 0
        const val y = 1
        const val z = 2
        const val roll = 3
        const val pitch = 4
        const val yaw = 5
        const val latency = 6
        const val tagCount = 7
        const val tagSpan = 8
        const val averageTagDistanceFromCamera = 9
        const val averageTagCoverage = 10
    }

    object Tc {
        const val hue = 0
        const val saturation = 1
        const val value = 2
    }

    object TCornXY {
        const val entriesPerCorner = 4
        const val x0 = 0
        const val y0 = 1
        const val x1 = 2
        const val y1 = 3
        const val x2 = 4
        const val y2 = 5
        const val x3 = 6
        const val y3 = 7
    }

    object RawTargets {
        const val entriesPerDetection = 3
        const val txnc = 0
        const val tync = 1
        const val ta = 2
    }

    object RawFiducials {
        const val entriesPerDetection = 7
        const val id = 0
        const val txnc = 1
        const val tync = 2
        const val ta = 3
        const val distToCamera = 4
        const val distToRobot = 5
        const val ambiguity = 6
    }

    object RawDetections {
        const val entriesPerDetection = 12
        const val id = 0
        const val txnc = 1
        const val tync = 2
        const val ta = 3
        const val x0 = 4
        const val y0 = 5
        const val x1 = 6
        const val y1 = 7
        const val x2 = 8
        const val y2 = 9
        const val x3 = 10
        const val y3 = 11
    }
}