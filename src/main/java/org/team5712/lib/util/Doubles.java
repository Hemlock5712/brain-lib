package org.team5712.lib.util;

import java.util.List;

public class Doubles {
    public static double[] toArray(List<Double> arr) {
        double[] floats = new double[arr.size()];
        for(int i = 0; i < arr.size(); i++) {
            floats[i] = arr.get(i);
        }
        return floats;
    }
}
