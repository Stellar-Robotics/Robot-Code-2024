package frc.utils;

public final class MiscUtils {
    
    // CLAMP functionality - Yay!
    public static double clamp(double min, double max, double value) {
        return Math.min(Math.max(min, value), max);
    }

    // Translate to non-linear response curve
    public static double transformRange(double linearInput, double exp) {
        return Math.abs(linearInput) * linearInput;
    }
}
