package org.team5712.lib.pathfind;

import java.util.*;

public class Pathfinder {
    List<Obstacle> obstacles = new ArrayList<>();
    NavigationMesh navMesh = new NavigationMesh();

    public Pathfinder(int xNodes, int yNodes) {
        for(int i = 0; i < xNodes; i++) {
            for (int j = 0; j < yNodes; j++) {
                navMesh.addNode(new Node(i, j));
            }
        }
    }

    public Pathfinder(int xNodes, int yNodes, List<Obstacle> obstacles) {
        this(xNodes, yNodes);
        this.obstacles = obstacles;
    }

    public void addObstacle(Obstacle obstacle) {
        this.obstacles.add(obstacle);
    }

    /**
     * Finds a path between the start point and end point.
     * End point should have a node created for it earlier on, probably
     * at robot startup, since we'll know all the points we have presets for
     *
     * @param startPoint Current robot position
     * @param endPoint Target position to find a path to
     * @return List of nodes to create a trajectory through, or null if no path is found
     */
    public List<Node> findPath(Node startPoint, Node endPoint) {
        // Add edges from current position to all other positions
        addNode(startPoint);

        return navMesh.findPath(startPoint, endPoint);
    }

    public void addNode(Node node) {
        navMesh.addNode(node);
        for(int i = 0; i < navMesh.getNodeSize(); i++) {
            Node endNode = navMesh.getNode(i);
            navMesh.addEdge(new Edge(node, endNode), obstacles);
        }
    }

    /**
     * Add edges between all nodes. Do this after all obstacles are added to the field
     */
    public void generateNodeEdges() {
        for(int i = 0; i < navMesh.getNodeSize(); i++) {
            Node startNode = navMesh.getNode(i);
            for(int j = i + 1; j < navMesh.getNodeSize(); j++) {
                Node endNode = navMesh.getNode(j);
                navMesh.addEdge(new Edge(startNode, endNode), obstacles);
            }
        }
    }
}
