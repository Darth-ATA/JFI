package jfi.shape.relevantspoints;

import jfi.shape.Contour;
import jfi.utils.JFIMath;

import java.awt.geom.Point2D;
import java.util.Arrays;
import java.util.List;

/**
 * Provides differents functions that are commons in differents operations with
 * contours.
 * @author Darth-ATA
 */
public class ContourOp {
    /**
     * Translate the contour.
     * @param contour to be translated.
     * @param translation we want to do.
     */
    public static void translation(Contour contour, Point2D translation){
        Point2D point;
        double x,y;

        for (int i = 0; i < contour.size(); i++){
            point = contour.get(i);
            x = point.getX() - translation.getX();
            y = point.getY() - translation.getY();
            contour.set(i, new Point2D.Double(x,y));
        }
    }

    /**
     * Calculates the difference range between two contours centrated both in 
     * the same barycenter.
     * @param contour1 for the diff.
     * @param contour2 for the diff.
     * @return the diff between the two contours.
     */
    public static double contourDiff(Contour contour1, Contour contour2){
        Point2D barycenter = barycenter(contour1);
        Point2D reconstructedBarycenter = barycenter(contour2);

        double x = reconstructedBarycenter.getX() - barycenter.getX();
        double y = reconstructedBarycenter.getY() - barycenter.getY();
        Point2D translation = new Point2D.Double(x, y);

        // Translates the reconstructed shape for make their barycenter in the 
        // same position.
        translation(contour2, translation);

        List<Double> curvature1 = Arrays.asList(contour1.curvature().toArray());
        List<Double> curvature2 = Arrays.asList(contour2.curvature().toArray());

        int maxCurvature1Index;
        int maxCurvature2Index;

        Point2D contour2Point;
        Point2D contour1Point;
        int minIndex = Math.min(curvature1.size(), curvature2.size());

        maxCurvature1Index = criticPointIndex(curvature1);
        maxCurvature2Index = criticPointIndex(curvature2);

        double sum = 0;
        double distance;
        double maxDistance = Double.MAX_VALUE;
        for (int i = 0; i < minIndex - 1; i++){
            contour1Point = contour1.get((i + maxCurvature1Index) % curvature1.size());
            contour2Point = contour2.get((i + maxCurvature2Index) % curvature2.size());
            distance = JFIMath.distance(contour1Point, barycenter);
            if (maxDistance < distance)
                maxDistance = distance;
            sum += Math.abs(JFIMath.distance(contour1Point, barycenter) - JFIMath.distance(contour2Point, barycenter));;
        }
        return sum / (curvature2.size() * maxDistance);
    }
    
    /**
     * Calculate the barycenter of the contour.
     * @param contour where we want to calculate the barycenter
     * @return barycenter position.
     */
    public static Point2D barycenter(List<Point2D> contour){
        double x = 0;
        double y = 0;

        for(Point2D point : contour){
            x += point.getX();
            y += point.getY();
        }
        x = x/contour.size();
        y = y/contour.size();

        return new Point2D.Double(x,y);
    }
    
    /**
     * Finds the critic point index.
     * @param zone
     * @return critic value index.
     */
    public static int criticPointIndex(List<Double> zone){
        int maxIndex = getMaxIndex(zone);
        int minIndex = getMinIndex(zone);
        int criticIndex;

        if(Math.abs(zone.get(maxIndex)) > Math.abs(zone.get(minIndex))){
            criticIndex = maxIndex;
        }
        else{
            criticIndex = minIndex;
        }

        return criticIndex;
    }
    
    /**
     * Provides the index of the max value in the list.
     * @param values list where find.
     * @return the index of the max value.
     */
    public static int getMaxIndex(List<Double> values){
        int start = 1;
        int end = values.size();
        int max = start;
        for(int i = start; i < end; i++){
            if(Math.abs(values.get(max)) < Math.abs(values.get(i))){
                max = i;
            }
        }
        return max;
    }
    
    /**
     * Provides the index of the min value in the list.
     * @param values list where find.
     * @return the index of the min value.
     */
    public static int getMinIndex(List<Double> values){
        int start = 1;
        int end = values.size();
        int min = start;
        for(int i = start; i < end; i++){
            if(Math.abs(values.get(min)) > Math.abs(values.get(i))){
                min = i;
            }
        }
        return min;
    }
}
