package tileworld.agent.utils;

import tileworld.agent.classdefines.MemorySideCardEntry;
import tileworld.Parameters;

/**
 * Math helpers for online hazard learning and alive-probability estimation.
 */
public final class HazardLearningUtils {
    private HazardLearningUtils() {
    }

    public static double computeAliveProbability(
            MemorySideCardEntry entry,
            double now,
            double lambda,
            double epsilon) {

        if (entry.isSpawnObserved()) {
            // Hard constraint: once exact lifetime is exceeded, this object must be gone.
            return (now <= (entry.getFirstSeen() + Parameters.lifeTime)) ? 1.0 : 0.0;
        }

        double delta = Math.max(0.0, now - entry.getLastSeen());
        double p = Math.exp(-lambda * delta);
        return clipProbability(p, epsilon);
    }

    public static double updateMatched(double lambda, double eta, double delta) {
        // d(-log P)/d lambda = delta, where P = exp(-lambda * delta).
        return Math.max(0.0, lambda - (eta * delta));
    }

    public static double updateMismatch(double lambda, double eta, double delta, double epsilon) {
        // d(-log(1-exp(-lambda*delta)))/d lambda = delta / (exp(lambda*delta)-1).
        double denom = Math.expm1(Math.max(0.0, lambda * delta)) + epsilon;
        return Math.max(0.0, lambda + (eta * delta / denom));
    }

    /**
     * Stable matched update with lower/upper bounds and per-step clipping.
     */
    public static double updateMatchedStable(
            double lambda,
            double eta,
            double delta,
            double minLambda,
            double maxLambda,
            double maxAbsStep) {
        double rawStep = -eta * Math.max(0.0, delta);
        double boundedStep = clip(rawStep, -maxAbsStep, maxAbsStep);
        return clip(lambda + boundedStep, minLambda, maxLambda);
    }

    /**
     * Stable mismatch update with denominator stabilization and bounded step.
     */
    public static double updateMismatchStable(
            double lambda,
            double eta,
            double delta,
            double epsilon,
            double minLambda,
            double maxLambda,
            double maxAbsStep) {
        double denom = Math.expm1(Math.max(0.0, lambda * delta)) + epsilon;
        double rawStep = eta * (Math.max(0.0, delta) / denom);
        double boundedStep = clip(rawStep, -maxAbsStep, maxAbsStep);
        return clip(lambda + boundedStep, minLambda, maxLambda);
    }

    public static double clip(double value, double low, double high) {
        if (value < low) {
            return low;
        }
        if (value > high) {
            return high;
        }
        return value;
    }

    public static double clipProbability(double p, double epsilon) {
        if (p < epsilon) {
            return epsilon;
        }
        if (p > 1.0) {
            return 1.0;
        }
        return p;
    }
}
