package frc.util.misc;

import java.util.*;
import edu.wpi.first.math.geometry.Translation2d;

public class Cluster {
	private final double density;
	private final int memberCount;
	private final List<Translation2d> members;
	private final List<Translation2d> hull;
	private final Translation2d center;
	private final Translation2d weightedCenter;

	public Cluster(List<Translation2d> members) {
		this.members = members;
		this.memberCount = members.size();
		this.hull = convexHull(members);
		this.density = computeDensity(hull, memberCount);

		this.center = center(hull);
		this.weightedCenter = weightedCenter(members);
	}

	public double getDensity() {
		return density;
	}

	public int getMemberCount() {
		return memberCount;
	}

	public List<Translation2d> getMembers() {
		return members;
	}

	public List<Translation2d> getHull() {
		return hull;
	}

	public Translation2d getClosest(Translation2d other) {
		Translation2d closest = members.get(0);
		double closestDist = other.getDistance(closest);
		for (int i = 1; i < memberCount; i++) {
			var candidate = members.get(i);
			var candidateDist = other.getDistance(candidate);
			if (candidateDist < closestDist) {
				closest = candidate;
				closestDist = candidateDist;
			}
		}
		return closest;
	}

	public Translation2d getFarthest(Translation2d other) {
		Translation2d farthest = members.get(0);
		double farthestDist = other.getDistance(farthest);
		for (int i = 1; i < memberCount; i++) {
			var candidate = members.get(i);
			var candidateDist = other.getDistance(candidate);
			if (candidateDist > farthestDist) {
				farthest = candidate;
				farthestDist = candidateDist;
			}
		}
		return farthest;
	}

	public Translation2d getCenter() {
		return center;
	}

	public Translation2d getWeightedCenter() {
		return weightedCenter;
	}


	public static List<Cluster> formClusters(List<Translation2d> points, double absorbRadius) {
		int n = points.size();
		if (n == 0) return Collections.emptyList();

		Map<Long, List<Integer>> grid = new HashMap<>();
		for (int i = 0; i < n; i++) {
			long key = cellKey(points.get(i), absorbRadius);
			grid.computeIfAbsent(key, k -> new ArrayList<>()).add(i);
		}

		UnionFind uf = new UnionFind(n);

		for (Map.Entry<Long, List<Integer>> entry : grid.entrySet()) {
			int gx = unpackX(entry.getKey());
			int gy = unpackY(entry.getKey());

			for (int dx = -1; dx <= 1; dx++) {
				for (int dy = -1; dy <= 1; dy++) {
					long neighborKey = pack(gx + dx, gy + dy);
					List<Integer> neighbor = grid.get(neighborKey);
					if (neighbor == null) continue;

					for (int i : entry.getValue()) {
						Translation2d a = points.get(i);
						for (int j : neighbor) {
							if (i >= j) continue;
							Translation2d b = points.get(j);
							if (a.getDistance(b) <= absorbRadius) {
								uf.union(i, j);
							}
						}
					}
				}
			}
		}

		Map<Integer, List<Translation2d>> rawClusters = new HashMap<>();
		for (int i = 0; i < n; i++) {
			int root = uf.find(i);
			rawClusters.computeIfAbsent(root, k -> new ArrayList<>()).add(points.get(i));
		}

		List<Cluster> clusters = new ArrayList<>();
		for (List<Translation2d> pts : rawClusters.values()) {
			if (!pts.isEmpty()) {
				clusters.add(new Cluster(pts));
			}
		}

		return clusters;
	}

	private static long cellKey(Translation2d p, double cellSize) {
		int gx = (int) Math.floor(p.getX() / cellSize);
		int gy = (int) Math.floor(p.getY() / cellSize);
		return pack(gx, gy);
	}

	private static long pack(int x, int y) {
		return (((long) x) << 32) | (y & 0xffffffffL);
	}

	private static int unpackX(long key) {
		return (int) (key >> 32);
	}

	private static int unpackY(long key) {
		return (int) key;
	}

	private static final class UnionFind {
		private final int[] parent;
		private final int[] rank;

		UnionFind(int n) {
			parent = new int[n];
			rank = new int[n];
			for (int i = 0; i < n; i++) parent[i] = i;
		}

		int find(int x) {
			if (parent[x] != x) parent[x] = find(parent[x]);
			return parent[x];
		}

		void union(int a, int b) {
			a = find(a);
			b = find(b);
			if (a == b) return;

			if (rank[a] < rank[b]) parent[a] = b;
			else if (rank[a] > rank[b]) parent[b] = a;
			else {
				parent[b] = a;
				rank[a]++;
			}
		}
	}

	private static double computeDensity(List<Translation2d> points) {
		if (points.size() < 3) {
			return Double.POSITIVE_INFINITY;
		}

		List<Translation2d> hull = convexHull(points);
		double area = polygonArea(hull);

		if (area <= 1e-6) {
			return Double.POSITIVE_INFINITY;
		}

		return points.size() / area;
	}

	private static double computeDensity(List<Translation2d> hull, int count) {
		double area = polygonArea(hull);

		if (area <= 1e-6) {
			return Double.POSITIVE_INFINITY;
		}

		return count / area;
	}

	private static List<Translation2d> convexHull(List<Translation2d> points) {
		List<Translation2d> sorted = new ArrayList<>(points);

		sorted.sort(Comparator
				.comparingDouble(Translation2d::getX)
				.thenComparingDouble(Translation2d::getY));

		List<Translation2d> lower = new ArrayList<>();
		for (Translation2d p : sorted) {
			while (lower.size() >= 2 &&
					cross(
						lower.get(lower.size() - 2),
						lower.get(lower.size() - 1),
						p
					) <= 0) {
				lower.remove(lower.size() - 1);
			}
			lower.add(p);
		}

		List<Translation2d> upper = new ArrayList<>();
		for (int i = sorted.size() - 1; i >= 0; i--) {
			Translation2d p = sorted.get(i);
			while (upper.size() >= 2 &&
					cross(
						upper.get(upper.size() - 2),
						upper.get(upper.size() - 1),
						p
					) <= 0) {
				upper.remove(upper.size() - 1);
			}
			upper.add(p);
		}

		lower.remove(lower.size() - 1);
		upper.remove(upper.size() - 1);
		lower.addAll(upper);

		return lower;
	}

	private static double cross(Translation2d o, Translation2d a, Translation2d b) {
		return (a.getX() - o.getX()) * (b.getY() - o.getY())
			- (a.getY() - o.getY()) * (b.getX() - o.getX());
	}

	private static double polygonArea(List<Translation2d> poly) {
		double area = 0.0;

		for (int i = 0; i < poly.size(); i++) {
			Translation2d p1 = poly.get(i);
			Translation2d p2 = poly.get((i + 1) % poly.size());

			area += p1.getX() * p2.getY()
				- p2.getX() * p1.getY();
		}

		return Math.abs(area) * 0.5;
	}

	private static Translation2d center(List<Translation2d> hull) {
		if (hull.size() < 3) {
			return new Translation2d();
		}

		double cx = 0.0;
		double cy = 0.0;
		double area = 0.0;

		for (int i = 0; i < hull.size(); i++) {
			Translation2d p0 = hull.get(i);
			Translation2d p1 = hull.get((i + 1) % hull.size());

			double cross = p0.getX() * p1.getY() - p1.getX() * p0.getY();

			area += cross;
			cx += (p0.getX() + p1.getX()) * cross;
			cy += (p0.getY() + p1.getY()) * cross;
		}

		area *= 0.5;

		if (Math.abs(area) < 1e-6) {
			return new Translation2d();
		}

		cx /= (6.0 * area);
		cy /= (6.0 * area);

		return new Translation2d(cx, cy);
	}

	private static Translation2d weightedCenter(List<Translation2d> members) {
		double sumX = 0.0;
		double sumY = 0.0;

		for (Translation2d p : members) {
			sumX += p.getX();
			sumY += p.getY();
		}

		int n = members.size();
		if (n == 0) {
			return new Translation2d();
		}

		return new Translation2d(sumX / n, sumY / n);
	}
}
