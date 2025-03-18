package net.tecdroid.util.units

data class Pixels(val count: Int) {
    companion object {
        fun of(count: Int) = Pixels(count)
    }
}

data class Percentage(val value: Double)
data class Factor(val value: Double)
