package org.team5712.lib.pathfind;

public class Obstacle {
    PolygonFloat polygon;

    public Obstacle(float[] xPoints, float[] yPoints, int nPoints) {
        this.polygon = new PolygonFloat(xPoints, yPoints, nPoints);
    }
}
