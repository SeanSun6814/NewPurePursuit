package frc.robot;

public class Vector {

	public double dx;
	public double dy;

	public Vector() {
		dx = dy = 0.0;
	}

	public Vector(double dx, double dy) {
		this.dx = dx;
		this.dy = dy;
	}

	public Vector(Point p) {
		this.dx = p.x;
		this.dy = p.y;
	}

	public Vector(Point p1, Point p2) {
		this.dx = p2.x - p1.x;
		this.dy = p2.y - p1.y;
	}

	public String toString() {
		return "Vector(" + dx + ", " + dy + ")";
	}

	public double length() {
		return Math.sqrt(dx * dx + dy * dy);
	}

	public Vector add(Vector v) {
		Vector v2 = new Vector(this.dx + v.dx, this.dy + v.dy);
		return v2;
	}

	public Vector subtract(Vector v) {
		Vector v2 = new Vector(this.dx - v.dx, this.dy - v.dy);
		return v2;
	}

	public Vector scale(double scaleFactor) {
		return new Vector(this.dx * scaleFactor, this.dy * scaleFactor);
	}

	public Vector normalize() {
		double length = length();
		if (length != 0) {
			return new Vector(this.dx / length, this.dy / length);
		}
		return new Vector();
	}

	public double dotProduct(Vector v) {
		return this.dx * v.dx + this.dy * v.dy;
	}

	@Deprecated
	public double getdX() {
		return dx;
	}

	@Deprecated
	public void setdX(double dx) {
		this.dx = dx;
	}

	@Deprecated
	public double getdY() {
		return dy;
	}

	@Deprecated
	public void setdY(double dy) {
		this.dy = dy;
	}

	public static void main(String args[]) {
		Vector vA = new Vector(20, 30);
		Vector vB = new Vector(2.0, 2.0);

		System.out.println("Vector vA =" + vA.toString());
		System.out.println("Vector vB =" + vB.toString());

		System.out.println("Vector vA-vB =" + vA.subtract(vB).toString());
		System.out.println("Vector vB-vA =" + vB.subtract(vA).toString());

		System.out.println("vA.normalize() =" + vA.normalize().toString());
		System.out.println("vB.normalize() =" + vB.normalize().toString());

		System.out.println("Dot product vA.vB =" + vA.dotProduct(vB));
		System.out.println("Dot product vB.vA =" + vB.dotProduct(vA));

	}
}
