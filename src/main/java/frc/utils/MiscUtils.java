package frc.utils;

public final class MiscUtils {
    
    // CLAMP functionality - Yay!
    public static double clamp(double min, double max, double value) {
        return Math.min(Math.max(min, value), max);
    }
}
