package net.tecdroid.util

data class NumericId(val id: Int) {

    init {
        require(id >= 0) { "Numeric ids must be positive ($id < 0)" }
    }
}

typealias CanId = NumericId
typealias DigitalPort = NumericId
typealias AnalogPort = NumericId
typealias PwmPort = NumericId
typealias RelayPort = NumericId
