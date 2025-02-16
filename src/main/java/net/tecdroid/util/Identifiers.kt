package net.tecdroid.util

open class NumericId(val id: Int) {

    init {
        require(id >= 0) { "Numeric ids must be positive ($id < 0)" }
    }

    operator fun plus(other: NumericId): NumericId {
        return NumericId(this.id + other.id)
    }

    operator fun minus(other: NumericId): NumericId {
        return NumericId(this.id - other.id)
    }

    fun toCanId() = CanId(id)
}

typealias CanId = NumericId

class DigitId(id: Int) : NumericId(id) {
    init {
        require(id in 0..9) { "Id digit must be a single numeric digit" }
    }
}

fun joinDigits(vararg digits: DigitId): NumericId {
    var base = 1
    var identifier = 0

    for (digit in digits.reversed()) {
        identifier += digit.id * base
        base *= 10
    }

    return NumericId(identifier)
}
