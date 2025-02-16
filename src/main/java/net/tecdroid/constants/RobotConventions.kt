package net.tecdroid.constants

import net.tecdroid.util.*

object StandardConvention: SpatialConvention(
    LongitudinalConvention(LongitudinalDirection.Front),
    TransversalConvention(TransversalDirection.Left),
    VerticalConvention(VerticalDirection.Up),
    RotationalConvention(RotationalDirection.Counterclockwise)
)
object WpiLibControllerConvention: SpatialConvention(
    LongitudinalConvention(LongitudinalDirection.Back),
    TransversalConvention(TransversalDirection.Right),
    VerticalConvention(VerticalDirection.Down),
    RotationalConvention(RotationalDirection.Clockwise)
)

var wpilibControllerToStandardConversion = SpatialConversion(WpiLibControllerConvention, StandardConvention)
