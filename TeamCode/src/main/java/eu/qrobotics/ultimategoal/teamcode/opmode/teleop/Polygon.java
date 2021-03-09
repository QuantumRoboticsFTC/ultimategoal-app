package eu.qrobotics.ultimategoal.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Polygon {
    public Vector2d[] vertices;

    public Polygon(Vector2d[] vertices) {
        this.vertices = vertices;
    }

    private boolean onSegment(Vector2d p, Vector2d q, Vector2d r) {
        return q.getX() <= Math.max(p.getX(), r.getX()) && q.getX() >= Math.min(p.getX(), r.getX()) &&
                q.getY() <= Math.max(p.getY(), r.getY()) && q.getY() >= Math.min(p.getY(), r.getY());
    }

    private int orientation(Vector2d p, Vector2d q, Vector2d r) {
        double val = (q.getY() - p.getY()) * (r.getX() - q.getX()) -
                (q.getX() - p.getX()) * (r.getY() - q.getY());

        return (int) Math.signum(val);
    }

    private boolean doIntersect(Vector2d p1, Vector2d q1, Vector2d p2, Vector2d q2) {
        // Find the four orientations needed for general and
        // special cases
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);

        // General case
        if (o1 != o2 && o3 != o4)
            return true;

        // Special Cases
        // p1, q1 and p2 are colinear and p2 lies on segment p1q1
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;

        // p1, q1 and p2 are colinear and q2 lies on segment p1q1
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;

        // p2, q2 and p1 are colinear and p1 lies on segment p2q2
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;

        // p2, q2 and q1 are colinear and q1 lies on segment p2q2
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;

        return false; // Doesn't fall in any of the above cases
    }

    public boolean isInsidePolygon(Vector2d point) {
        if (vertices.length < 3) return false;

        // Create a point for line segment from p to infinite
        Vector2d extreme = new Vector2d(1000, point.getY());

        // Count intersections of the above line with sides of polygon
        int count = 0, i = 0;

        do {
            int next = (i + 1) % vertices.length;

            if (doIntersect(vertices[i], vertices[next], point, extreme)) {
                if (orientation(vertices[i], point, vertices[next]) == 0)
                    return onSegment(vertices[i], point, vertices[next]);
                if(vertices[i].getY() != extreme.getY())
                    count++;
            }
            i = next;
        } while (i != 0);

        return count % 2 == 1;
    }

    public Vector2d closestPointOnPolygon(Vector2d point) {
        double bestDistance = Double.POSITIVE_INFINITY;
        Vector2d bestPoint = null;

        int i = 0;
        do {
            int next = (i + 1) % vertices.length;

            double t = -((vertices[next].minus(vertices[i])).dot(vertices[i].minus(point)))
                    / ((vertices[next].minus(vertices[i])).dot(vertices[next].minus(vertices[i])));
            t = Math.max(0, Math.min(1, t));
            Vector2d currentPoint = vertices[i].times(1 - t).plus(vertices[next].times(t));

            double distance = currentPoint.minus(point).norm();
            if(distance < bestDistance) {
                bestDistance = distance;
                bestPoint = currentPoint;
            }

            i = next;
        } while (i != 0);

        return bestPoint;
    }
}
