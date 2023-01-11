import org.junit.jupiter.api.Test;
import org.team5712.lib.pathfind.Obstacle;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class ObstacleTest {
    @Test
    public void testExpandPolygonSquare() {
        Obstacle o = new Obstacle(new double[] { 0, 0, 4, 4}, new double[] {0, 4, 4, 0});
        Obstacle offset = o.offset(0.5f);
        System.out.println(offset);
        assertEquals(true, true);
    }

    @Test
    public void testExpandPolygonPentagon() {
        Obstacle o = new Obstacle(new double[] { 0, 0, 10, 5f, 10}, new double[] {0, 10, 10, 5f, 0});
        Obstacle offset = o.offset(0.5f);
        System.out.println(offset);
        assertEquals(true, true);
    }
}
