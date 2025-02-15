package net.tecdroid.constants

import net.tecdroid.util.*

object RobotSpatialConvention: SpatialConvention(
    LongitudinalConvention(LongitudinalDirection.Front),
    TransversalConvention(TransversalDirection.Left),
    VerticalConvention(VerticalDirection.Up),
    RotationalConvention(RotationalDirection.Counterclockwise)
)
object WpiLibControllerSpatialConvention: SpatialConvention(
    LongitudinalConvention(LongitudinalDirection.Back),
    TransversalConvention(TransversalDirection.Right),
    VerticalConvention(VerticalDirection.Down),
    RotationalConvention(RotationalDirection.Clockwise)
)
