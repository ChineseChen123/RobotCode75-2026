package frc.lib.util;

import java.util.Arrays;
import java.util.Iterator;
import java.util.TreeMap;

public class LinearInterpolationMap {
	private final TreeMap<Double, Double> points = new TreeMap<>();

	private boolean cacheDirty = true;
	private double[] segmentStartX = new double[0];
	private double[] segmentEndX = new double[0];
	private double[] segmentStartY = new double[0];
	private double[] segmentEndY = new double[0];
	private double[] segmentSlope = new double[0];
	private double[] segmentIntercept = new double[0];

	/** Adds a point and returns this map to allow daisy chaining. */
	public LinearInterpolationMap add(double x, double y) {
		points.put(x, y);
		cacheDirty = true;
		return this;
	}

	/** Adds a point using standard non-chained call style. */
	public void put(double x, double y) {
		points.put(x, y);
		cacheDirty = true;
	}

	/**
	 * Returns interpolated/extrapolated y-value for a given x.
	 *
	 * <p>If x is below min or above max, uses the first two or last two points respectively.
	 */
	public double get(double x) {
		if (points.isEmpty()) {
			throw new IllegalStateException("LinearInterpolationMap has no points");
		}
		if (points.size() == 1) {
			return points.firstEntry().getValue();
		}

		Double exactValue = points.get(x);
		if (exactValue != null) {
			return exactValue;
		}

		rebuildCacheIfDirty();

		if (x < segmentStartX[0]) {
			return evaluateSegment(0, x);
		}

		int lastSegment = segmentSlope.length - 1;
		if (x > segmentEndX[lastSegment]) {
			return evaluateSegment(lastSegment, x);
		}

		int segmentIndex = Arrays.binarySearch(segmentEndX, x);
		if (segmentIndex < 0) {
			segmentIndex = -segmentIndex - 1;
		}

		return evaluateSegment(segmentIndex, x);
	}

	/**
	 * Returns the inverse-interpolated/extrapolated x-value for a given y.
	 *
	 * <p>If the map is not monotonic in y, this uses the first segment that contains y. If none contain y,
	 * it uses the closest segment by y-distance.
	 */
	public double getInverse(double y) {
		if (points.isEmpty()) {
			throw new IllegalStateException("LinearInterpolationMap has no points");
		}
		if (points.size() == 1) {
			return points.firstKey();
		}

		rebuildCacheIfDirty();

		int containingSegment = -1;
		for (int i = 0; i < segmentSlope.length; i++) {
			if (isBetweenInclusive(y, segmentStartY[i], segmentEndY[i])) {
				containingSegment = i;
				break;
			}
		}

		if (containingSegment >= 0) {
			return evaluateInverseSegment(containingSegment, y);
		}

		int closestSegment = 0;
		double bestDistance = Double.POSITIVE_INFINITY;
		for (int i = 0; i < segmentSlope.length; i++) {
			double minY = Math.min(segmentStartY[i], segmentEndY[i]);
			double maxY = Math.max(segmentStartY[i], segmentEndY[i]);
			double distance = y < minY ? (minY - y) : (y > maxY ? (y - maxY) : 0.0);

			if (distance < bestDistance) {
				bestDistance = distance;
				closestSegment = i;
			}
		}

		return evaluateInverseSegment(closestSegment, y);
	}

	private double evaluateSegment(int index, double x) {
		return segmentSlope[index] * x + segmentIntercept[index];
	}

	private double evaluateInverseSegment(int index, double y) {
		double slope = segmentSlope[index];
		if (slope == 0.0) {
			// Horizontal segment does not have a unique inverse; return the segment start x.
			return segmentStartX[index];
		}
		return (y - segmentIntercept[index]) / slope;
	}

	private boolean isBetweenInclusive(double value, double a, double b) {
		return value >= Math.min(a, b) && value <= Math.max(a, b);
	}

	private void rebuildCacheIfDirty() {
		if (!cacheDirty) {
			return;
		}

		int segmentCount = points.size() - 1;
		segmentStartX = new double[segmentCount];
		segmentEndX = new double[segmentCount];
		segmentStartY = new double[segmentCount];
		segmentEndY = new double[segmentCount];
		segmentSlope = new double[segmentCount];
		segmentIntercept = new double[segmentCount];

		Iterator<Double> xIterator = points.keySet().iterator();
		double x1 = xIterator.next();
		double y1 = points.get(x1);

		for (int i = 0; i < segmentCount; i++) {
			double x2 = xIterator.next();
			double y2 = points.get(x2);

			double slope = (y2 - y1) / (x2 - x1);

			segmentStartX[i] = x1;
			segmentEndX[i] = x2;
			segmentStartY[i] = y1;
			segmentEndY[i] = y2;
			segmentSlope[i] = slope;
			segmentIntercept[i] = y1 - slope * x1;

			x1 = x2;
			y1 = y2;
		}

		cacheDirty = false;
	}
}
