package frc.util.geometry;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.util.flipping.AllianceFlipUtil;
import frc.util.flipping.AllianceFlipUtil.FieldFlipType;
import frc.util.flipping.AllianceFlippable;

public class PoseBoundingBoxUtil {
	public static interface BoundingBox<Self extends BoundingBox<Self>> extends AllianceFlippable<Self> {
		public boolean withinBounds(Translation2d trans);
		public default boolean withinBounds(Pose2d pose) {
			return withinBounds(pose.getTranslation());
		}

		public static RectangularBoundingBox rectangle(Translation2d corner1, Translation2d corner2) {
			return new RectangularBoundingBox(corner1, corner2);
		}
		public static CircularBoundingBox circle(Translation2d center, Distance radius) {
			return new CircularBoundingBox(center, radius);
		}

		public default OrBox or(BoundingBox<?>... boxes) {
			return new OrBox(this).or(boxes);
		}
		public default AndBox and(BoundingBox<?>... boxes) {
			return new AndBox(this).and(boxes);
		}
		public default NotBox not() {
			return new NotBox(this);
		}
	}
	// Primitives
	public static class VerticalLine implements BoundingBox<VerticalLine> {
		private final double xPosMeters;
		private final int side;

		private VerticalLine(double xPosMeters, int side) {
			this.xPosMeters = xPosMeters;
			this.side = side;
		}
		public VerticalLine(double xPosMeters, boolean rightSide) {
			this(xPosMeters, (rightSide) ? (+1) : (-1));
		}

		@Override
		public boolean withinBounds(Translation2d trans) {
			return (trans.getX() - this.xPosMeters) * this.side >= 0.0;
		}

		@Override
		public VerticalLine flip(FieldFlipType flipType) {
			return new VerticalLine(
				AllianceFlipUtil.flipXPosMeters(this.xPosMeters, flipType),
				-this.side
			);
		}
	}
	public static class RectangularBoundingBox implements BoundingBox<RectangularBoundingBox> {
		private final Translation2d bottomLeftCorner;
		private final Translation2d topRightCorner;

		public RectangularBoundingBox(Translation2d corner1, Translation2d corner2) {
			this.bottomLeftCorner = new Translation2d(Math.min(corner1.getX(), corner2.getX()), Math.min(corner1.getY(), corner2.getY()));
			this.topRightCorner = new Translation2d(Math.max(corner1.getX(), corner2.getX()), Math.max(corner1.getY(), corner2.getY()));
		}

		@Override
		public boolean withinBounds(Translation2d trans) {
			return
				trans.getX() >= this.bottomLeftCorner.getX()
				&& trans.getX() <= this.topRightCorner.getX()
				&& trans.getY() >= this.bottomLeftCorner.getY()
				&& trans.getY() <= this.topRightCorner.getY()
			;
		}

		@Override
		public RectangularBoundingBox flip(FieldFlipType flipType) {
			return new RectangularBoundingBox(
				AllianceFlipUtil.flip(this.bottomLeftCorner, flipType),
				AllianceFlipUtil.flip(this.topRightCorner, flipType)
			);
		}
	}
	public static class CircularBoundingBox implements BoundingBox<CircularBoundingBox> {
		private final Translation2d center;
		private final double radiusMeters;

		public CircularBoundingBox(Translation2d center, double radiusMeters) {
			this.center = center;
			this.radiusMeters = radiusMeters;
		}
		public CircularBoundingBox(Translation2d center, Distance radius) {
			this(center, radius.in(Meters));
		}

		@Override
		public boolean withinBounds(Translation2d trans) {
			return this.center.getDistance(trans) <= this.radiusMeters;
		}

		@Override
		public CircularBoundingBox flip(FieldFlipType flipType) {
			return new CircularBoundingBox(
				AllianceFlipUtil.flip(this.center, flipType),
				this.radiusMeters
			);
		}
	}

	// Compound
	public static class OrBox implements BoundingBox<OrBox> {
		private final BoundingBox<?>[] boxes;

		public OrBox(BoundingBox<?>... boxes) {
			this.boxes = boxes;
		}

		@Override
		public boolean withinBounds(Translation2d trans) {
			for (var box : this.boxes) {
				if (box.withinBounds(trans)) {
					return true;
				}
			}
			return false;
		}

		@Override
		public OrBox or(BoundingBox<?>... boxes) {
			BoundingBox<?>[] newBoxes = new BoundingBox[this.boxes.length + boxes.length];
			System.arraycopy(this.boxes, 0, newBoxes, 0, this.boxes.length);
			System.arraycopy(boxes, 0, newBoxes, this.boxes.length, boxes.length);
			return new OrBox(newBoxes);
		}

		@Override
		public OrBox flip(FieldFlipType flipType) {
			BoundingBox<?>[] newBoxes = new BoundingBox[this.boxes.length];
			for (int i = 0; i < this.boxes.length; i++) {
				newBoxes[i] = this.boxes[i].flip(flipType);
			}
			return new OrBox(newBoxes);
		}
	}
	public static class AndBox implements BoundingBox<AndBox> {
		private final BoundingBox<?>[] boxes;

		public AndBox(BoundingBox<?>... boxes) {
			this.boxes = boxes;
		}

		@Override
		public boolean withinBounds(Translation2d trans) {
			for (var box : this.boxes) {
				if (!box.withinBounds(trans)) {
					return false;
				}
			}
			return true;
		}

		@Override
		public AndBox and(BoundingBox<?>... boxes) {
			BoundingBox<?>[] newBoxes = new BoundingBox[this.boxes.length + boxes.length];
			System.arraycopy(this.boxes, 0, newBoxes, 0, this.boxes.length);
			System.arraycopy(boxes, 0, newBoxes, this.boxes.length, boxes.length);
			return new AndBox(newBoxes);
		}

		@Override
		public AndBox flip(FieldFlipType flipType) {
			BoundingBox<?>[] newBoxes = new BoundingBox[this.boxes.length];
			for (int i = 0; i < this.boxes.length; i++) {
				newBoxes[i] = this.boxes[i].flip(flipType);
			}
			return new AndBox(newBoxes);
		}
	}
	public static class NotBox implements BoundingBox<NotBox> {
		private final BoundingBox<?> box;

		public NotBox(BoundingBox<?> box) {
			this.box = box;
		}

		@Override
		public boolean withinBounds(Translation2d trans) {
			return !this.box.withinBounds(trans);
		}

		@Override
		public NotBox flip(FieldFlipType flipType) {
			return new NotBox(
				this.box.flip(flipType)
			);
		}
	}
}
