package jfi.shape.relevantspoints;

import jfi.shape.Contour;
import jfi.shape.fuzzy.FuzzyContourFactory;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;


/**
 * Provides all the functionalities for calculate the linearity and curves of 
 * the contour.
 * @author Darth-ATA
 */
public class Linearizator {

    private final Contour contour;
    private final double alpha;
    private final double q;
    
    /**
     * Build the Curvizator with the Contour as parameter
     * @param contour contour.
     * @param alpha alpha.
     */
    public Linearizator(Contour contour, double alpha){
        this.contour = contour;
        this.alpha = alpha;
        this.q = 1 - (0.37 / Math.PI) * this.alpha;
    }

    /**
     * Provides the linearity degree of the segment.
     * @param start of the segment.
     * @param end of the segment.
     * @param alpha for calculate the linearity.
     * @return linearity value.
     */
    public Double symLinearityDegree(int start, int end, double alpha){
        Point2D startPoint = contour.get(start);
        Point2D endPoint = contour.get(end);

        double linearityDegree;
        List<Point2D> segmentInterestPoints = contour.getSegment(startPoint, endPoint);
        if (segmentInterestPoints.isEmpty())
            linearityDegree = 1;
        else
            linearityDegree = FuzzyContourFactory.linearityDegree((ArrayList)segmentInterestPoints, q);

        return linearityDegree;
    }

    /**
     * Provides the linearity degree of the segment.
     * @param startPoint of the segment.
     * @param endPoint of the segment.
     * @param alpha for calculate the linearity.
     * @return linearity value.
     */
    public Double symLinearityDegree(Point2D startPoint, Point2D endPoint, double alpha){
        double linearityDegree;
        List<Point2D> segmentInterestPoints = contour.getSegment(startPoint, endPoint);
        if (segmentInterestPoints.isEmpty())
            linearityDegree = 1;
        else {
            linearityDegree = FuzzyContourFactory.linearityDegree((ArrayList)segmentInterestPoints, alpha);
            double distanceToTheLine = distanceWithSignBetweenLineAndPoint(
                    startPoint, endPoint, segmentInterestPoints.get(segmentInterestPoints.size()/2));
            if (distanceToTheLine < 0){
                linearityDegree = -linearityDegree;
            }
        }
        return linearityDegree;
    }

    /**
     * Calculate the distance between the line defined by two points and a third point.
     * @param lineStart one point of the line.
     * @param lineEnd other point of the line.
     * @param point the third point.
     * @return the distance between the line and the point.
     */
    public static double distanceWithSignBetweenLineAndPoint(Point2D lineStart, Point2D lineEnd, Point2D point){
        double x1 = lineStart.getX();
        double y1 = lineStart.getY();

        double x2 = lineEnd.getX();
        double y2 = lineEnd.getY();

        double x0 = point.getX();
        double y0 = point.getY();

        return ((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1)/(Math.sqrt(Math.pow(y2 - y1,2) + Math.pow(x2 - x1,2)));
    }    
}