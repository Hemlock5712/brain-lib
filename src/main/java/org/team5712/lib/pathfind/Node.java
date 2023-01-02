package org.team5712.lib.pathfind;

import java.util.*;

public class Node {
    double x, y, holonomicRotation;
    List<Node> neighbors;

    public Node(double x, double y) {
        this.x = x;
        this.y = y;
        holonomicRotation = -1;
        this.neighbors = new ArrayList<>();
    }

    public Node(double x, double y, double holonomicRotation) {
        this.x = x;
        this.y = y;
        this.holonomicRotation = holonomicRotation;
        this.neighbors = new ArrayList<>();
    }

    public void addNeighbor(Node neighbor) {
        this.neighbors.add(neighbor);
    }
}
