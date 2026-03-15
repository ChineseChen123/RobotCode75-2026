package frc.lib.util;

import java.util.ArrayDeque;
import java.util.Deque;

import edu.wpi.first.wpilibj.Timer;

public class SG5PointFilter {
    // Savitzky-Golay filter using 5 past points
    // falls back to linear approx. when < 5 past points
    private final double dt;
    private final Deque<Double> velocities = new ArrayDeque<>();

    private double lastTimeGivenVelocity = 0.0;

    public SG5PointFilter(double dt) {
        this.dt = dt;
    }

    public double linearFallback() {
        // linear approximation using the last two points
        if (velocities.size() < 2) {
            return 0.0;
        }
        return (velocities.getLast() - velocities.getFirst()) / (dt * (velocities.size() - 1));
    }

    public double updateDeriv(double velocity) {
        // derivative of velocity filter
        
        lastTimeGivenVelocity = Timer.getFPGATimestamp(); 

        if (lastTimeGivenVelocity - Timer.getFPGATimestamp() > dt * 5) {
            velocities.clear();
        }

        velocities.addLast(velocity);
        if (velocities.size() > 5) {
            velocities.removeFirst();
        }

        if (velocities.size() < 5) {
            return linearFallback();
        }

        double[] v = new double[5];
        int idx = 0;
        for (double val : velocities) {
            v[idx++] = val;
        }

        double vk   = v[4];
        double vk1  = v[3];
        double vk2  = v[2];
        double vk3  = v[1];
        double vk4  = v[0];

        return (25.0 * vk - 48.0 * vk1 + 36.0 * vk2 - 16.0 * vk3 + 3.0 * vk4) / (12.0 * dt);
    }
}
