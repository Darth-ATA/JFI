package jfi.shape.relevantspoints;

import javafx.util.Pair;
import jfi.shape.Contour;

import java.awt.geom.Point2D;
import java.util.*;
import java.util.stream.IntStream;

/**
 * Provides all the functionalities for detect the relevant points of a contour
 * using the space scale.
 * @author Darth-ATA
 */
public class ScaleSpaceDetector {
    //Minimun segment size.
    static int MINIMUN_DISTANCE = 10;
    static double ZEROS_THRESHOLD = 1;
    static double CRITICS_THRESHOLD = 8;
    
    private final double segmentSizeRatio;
    private final double segmentLocalRatio;

    /**
     * Build the ScaleSpaceDetector with the segmentSizeRatio as parameter.
     * @param segmentSizeRatio 
     * @param segmentLocalRatio
     */
    public ScaleSpaceDetector(double segmentSizeRatio, double segmentLocalRatio){
        this.segmentSizeRatio = segmentSizeRatio;
        this.segmentLocalRatio = segmentLocalRatio;
    }

    /**
     * Takes all the scales relevantPoints and apply the multiescale detection.
     * @param startContour
     * @param start
     * @param end
     * @param sigmaScalesRelevantIndex
     * @param threshold
     * @return 
     */
    public List<Integer> getRelevantMultiScaleIndex(Contour startContour, int start, int end,
                                                    List<List<Integer>> sigmaScalesRelevantIndex, double threshold){
        // If we have scales...
        if (sigmaScalesRelevantIndex.size() > 0){
            List<Integer> relevantScaleIndex = new ArrayList<>();
            Set<Integer>  relevantMultiScaleIndexSet = new HashSet<>();

            List<Integer> relevantIndex = sigmaScalesRelevantIndex.get(sigmaScalesRelevantIndex.size() - 1);
            relevantIndex.stream()
                    .filter((relevant) -> ((relevant >= start && relevant <= end) || (start > end && (relevant >= end || relevant <= start))))
                    .forEachOrdered((relevant) -> {
                        relevantScaleIndex.add(relevant);
                    });
            if (relevantScaleIndex.size() > 1) {
                IntStream.range(0, relevantScaleIndex.size())
                    .parallel()
                    .forEach(i -> {
                    Integer iStartIndex = relevantScaleIndex.get(i);
                    Integer iEndIndex = relevantScaleIndex.get((i + 1) % relevantScaleIndex.size());
                    Pair<Integer, Integer> pairPointIndex = new Pair(iStartIndex, iEndIndex);
                    pairPointIndex = interestPointScaleTrajectory(sigmaScalesRelevantIndex, pairPointIndex);
                    iStartIndex = pairPointIndex.getKey();
                    iEndIndex = pairPointIndex.getValue();
                    Point2D iStartPoint = startContour.get(iStartIndex);
                    Point2D iEndPoint = startContour.get(iEndIndex);

                    List<List<Integer>> iSigmaScalesIndex = new ArrayList(sigmaScalesRelevantIndex);

                    List<Point2D> originalSegment = startContour.getSegment(iStartPoint, iEndPoint);
                    double segmentError = errorSegments(originalSegment, iStartPoint, iEndPoint);

                    relevantMultiScaleIndexSet.add(iStartIndex);
                    if (segmentError > threshold) {
                        iSigmaScalesIndex.remove(sigmaScalesRelevantIndex.size() - 1);
                        relevantMultiScaleIndexSet.addAll(getRelevantMultiScaleIndex(startContour, iStartIndex, iEndIndex,
                                iSigmaScalesIndex, threshold));
                    }
                    relevantMultiScaleIndexSet.add(iEndIndex);
                });
            }
            else if (relevantScaleIndex.size() > 0) {
                Integer iStartIndex = relevantScaleIndex.get(0);
                relevantMultiScaleIndexSet.add(interestPointScaleTrajectory(sigmaScalesRelevantIndex, iStartIndex));
            }
            else if (sigmaScalesRelevantIndex.size() > 1){
                List<List<Integer>> iSigmaScalesIndex = new ArrayList(sigmaScalesRelevantIndex);
                iSigmaScalesIndex.remove(sigmaScalesRelevantIndex.size() - 1);
                relevantMultiScaleIndexSet.addAll(getRelevantMultiScaleIndex(startContour, start, end,
                        iSigmaScalesIndex, threshold));
            }
            List<Integer> sortedRelevantMultiScaleIndex = new ArrayList(relevantMultiScaleIndexSet);
            Collections.sort(sortedRelevantMultiScaleIndex);
            return sortedRelevantMultiScaleIndex;
        }
        else{
            return new ArrayList<>();
        }
    }
    
    /**
     * Follow the relevantPoints pair for discover their index in the original contour.
     * @param sigmaScalesInterestIndex all the relevant points scales.
     * @param pairPointIndex points whose find their trajectory.
     * @return the index of the points.
     */
    private Pair<Integer, Integer> interestPointScaleTrajectory(List<List<Integer>> sigmaScalesInterestIndex,
            Pair<Integer, Integer> pairPointIndex){
        int nearestStartIndex = pairPointIndex.getKey();
        int nearestEndIndex = pairPointIndex.getValue();
        int indexPointMinDistance = Integer.MAX_VALUE;
        int iPointIndex;
        int iIndexPointDistance;
        List<Integer> pointsIndex;
        boolean inverted = (pairPointIndex.getValue() - pairPointIndex.getKey()) < 0;
        for (int j = sigmaScalesInterestIndex.size() - 1; j >= 0; j--){
            pointsIndex = sigmaScalesInterestIndex.get(j);
            for (int i = 0; i < pointsIndex.size(); i++) {
                iPointIndex = pointsIndex.get(i);
                iIndexPointDistance = Math.abs(pairPointIndex.getKey() - iPointIndex);
                if (iIndexPointDistance < indexPointMinDistance) {
                    nearestStartIndex = iPointIndex;
                    indexPointMinDistance = iIndexPointDistance;
                }
            }
            indexPointMinDistance = Integer.MAX_VALUE;
            for (int i = 0; i < pointsIndex.size(); i++) {
                iPointIndex = pointsIndex.get(i);
                if ((inverted && (iPointIndex - nearestStartIndex) < 0) 
                        || (!inverted && iPointIndex != nearestStartIndex)){
                    iIndexPointDistance = Math.abs(pairPointIndex.getValue() - iPointIndex);
                    if (iIndexPointDistance < indexPointMinDistance) {
                        nearestEndIndex = iPointIndex;
                        indexPointMinDistance = iIndexPointDistance;
                    }
                }
            }
            pairPointIndex = new Pair(nearestStartIndex, nearestEndIndex);
            indexPointMinDistance = Integer.MAX_VALUE;
        }
        return pairPointIndex;
    }
    
       /**
     * Follow the relevantPoints pair for discover their index in the original contour.
     * @param sigmaScalesInterestIndex all the relevant points scales.
     * @param pairPointIndex points whose find their trajectory.
     * @return the index of the points.
     */
    private Integer interestPointScaleTrajectory(List<List<Integer>> sigmaScalesInterestIndex,
            Integer pointIndex){
        int nearestStartIndex = pointIndex;
        int indexPointMinDistance = Integer.MAX_VALUE;
        int iPointIndex;
        int iIndexPointDistance;
        List<Integer> pointsIndex;
        for (int j = sigmaScalesInterestIndex.size() - 1; j >= 0; j--){
            pointsIndex = sigmaScalesInterestIndex.get(j);
            for (int i = 0; i < pointsIndex.size(); i++) {
                iPointIndex = pointsIndex.get(i);
                iIndexPointDistance = Math.abs(pointIndex - iPointIndex);
                if (iIndexPointDistance < indexPointMinDistance) {
                    nearestStartIndex = iPointIndex;
                    indexPointMinDistance = iIndexPointDistance;
                }
            }
            indexPointMinDistance = Integer.MAX_VALUE;
        }
        return nearestStartIndex;
    }

    /**
     * Calculate the min critic error index point.
     * @param originalContour
     * @param relevantMultiScaleIndex
     * @return 
     */
    private static Pair<Integer, Double> minCriticsIndexError(
            Contour originalContour, List<Integer> relevantMultiScaleIndex){

        Pair<Integer, Double> minError = new Pair<>(0, Double.MAX_VALUE);
        double iError;
        List<Point2D> originalSegment;
        Integer iStartIndex;
        Integer iEndIndex;
        Point2D iStartPoint;
        Point2D iEndPoint;
        for(int i = 0; i < relevantMultiScaleIndex.size(); i++){
            iStartIndex = relevantMultiScaleIndex.get((i-1) < 0 ? relevantMultiScaleIndex.size() - 1 : (i-1));
            iStartPoint = originalContour.get(iStartIndex);
            iEndIndex = relevantMultiScaleIndex.get((i+2) % relevantMultiScaleIndex.size());
            iEndPoint = originalContour.get(iEndIndex);
            originalSegment = originalContour.getSegment(iStartPoint, iEndPoint);
            iError = errorSegments(originalSegment, iStartPoint, iEndPoint);
            if (iError < minError.getValue()){
                minError = new Pair<>(i, iError);
            }
        }
        return minError;
    }

    /**
     * Calculate the min zero error index point.
     * @param originalContour
     * @param relevantMultiScaleIndex
     * @return 
     */
    private static Pair<Integer, Double> minChangesIndexError(
            Contour originalContour, List<Integer> relevantMultiScaleIndex){

        Pair<Integer, Double> minError = new Pair<>(0, Double.MAX_VALUE);
        double iFirstError;
        List<Point2D> firstSegment;
        Integer iStartIndex;
        Integer iEndIndex;
        Point2D iStartPoint;
        Point2D iEndPoint;
        for(int i = 0; i < relevantMultiScaleIndex.size(); i++){
            iStartIndex = relevantMultiScaleIndex.get((i-1) < 0 ? relevantMultiScaleIndex.size() - 1 : (i-1));
            iStartPoint = originalContour.get(iStartIndex);
            iEndIndex = relevantMultiScaleIndex.get(i);
            iEndPoint = originalContour.get(iEndIndex);
            firstSegment = originalContour.getSegment(iStartPoint, iEndPoint);
            iFirstError = errorSegments(firstSegment, iStartPoint, iEndPoint);
            if (iFirstError < minError.getValue()){
                minError = new Pair<>(i, iFirstError);
            }
        }
        return minError;
    }

    /**
     * Minimization algorithm for critic points.
     * @param originalContour
     * @param relevantMultiScaleIndex
     * @return 
     */
    public List<Integer> minCriticsPoints(Contour originalContour, List<Integer> relevantMultiScaleIndex, double threshold){
        List<Integer> minRelevantMultiScaleIndex = new ArrayList<>(relevantMultiScaleIndex);
        Pair<Integer, Double> iMinIndexError = minCriticsIndexError(originalContour, minRelevantMultiScaleIndex);
        int removableIndex;
        while(iMinIndexError.getValue() <= threshold && minRelevantMultiScaleIndex.size() > 2){
            removableIndex = iMinIndexError.getKey();
            minRelevantMultiScaleIndex.remove(removableIndex);
            iMinIndexError = minCriticsIndexError(originalContour, minRelevantMultiScaleIndex);
        }
        for (int i = 0; i < minRelevantMultiScaleIndex.size() && minRelevantMultiScaleIndex.size() > 2; i++){
            if (Math.abs(minRelevantMultiScaleIndex.get(i) - minRelevantMultiScaleIndex.get(((i+1) % minRelevantMultiScaleIndex.size()))) < MINIMUN_DISTANCE){
                minRelevantMultiScaleIndex.remove(i);
                i--;
            }
        }
        return  minRelevantMultiScaleIndex;
    }

    
    
    /**
     * Multiscale segmentation using zero points.
     * @param contour
     * @param maxSigma
     * @param sigma
     * @param threshold
     * @return 
     */
    public List<Integer> zerosSegmentation(Contour contour, double maxSigma, 
            double sigma, double threshold, double alpha){
        Contour iContour;
        Detector iDetector;
        List<List<Integer>> sigmaScalesZerosIndex = new ArrayList<>();
        List<Integer> iScaleZerosIndex;
        for (double iSigma = sigma; iSigma < maxSigma; iSigma = iSigma * sigma){
            iContour = contour.gaussianFiltering(iSigma);
            iDetector = new Detector(iContour, this.segmentSizeRatio, 
                    this.segmentLocalRatio, alpha);
            iScaleZerosIndex = iDetector.getZeroCurvaturePointsIndex();
            sigmaScalesZerosIndex.add(iScaleZerosIndex);
        }
        Contour contourFiltrated = contour.gaussianFiltering(sigma);
        /*List<Integer> zerosMultiScaleIndex = this.getRelevantMultiScaleIndex(contourFiltrated, 0,
                contourFiltrated.size(), sigmaScalesZerosIndex, threshold);
        zerosMultiScaleIndex = this.minCriticsPoints(contour, zerosMultiScaleIndex, threshold);*/
        iDetector = new Detector(contour, this.segmentSizeRatio, 
                this.segmentLocalRatio, alpha);
        return iDetector.getZeroCurvaturePointsIndex();
    }
    
    /**
     * Multiscale segmentation using critic points.
     * @param contour
     * @param maxSigma
     * @param sigma
     * @param threshold
     * @return 
     */
    public List<Integer> criticsSegmentation(Contour contour, double maxSigma, 
            double sigma, double threshold, double alpha){
        Contour iContour;
        Detector iDetector;
        List<List<Integer>> sigmaScalesCriticsIndex = new ArrayList<>();
        List<Integer> iScaleCriticsIndex;
        for (double iSigma = sigma; iSigma < maxSigma; iSigma = iSigma * sigma) {
            iContour = contour.gaussianFiltering(iSigma);
            iDetector = new Detector(iContour, this.segmentSizeRatio, 
                    this.segmentLocalRatio, alpha);
            iScaleCriticsIndex = iDetector.getCriticCurvaturePointsIndex();
            sigmaScalesCriticsIndex.add(iScaleCriticsIndex);
        }
        Contour contourFiltrated = contour.gaussianFiltering(sigma);
        List<Integer> criticsMultiScaleIndex = this.getRelevantMultiScaleIndex(contourFiltrated, 0,
                contourFiltrated.size(), sigmaScalesCriticsIndex, threshold);
        return this.minCriticsPoints(contour, criticsMultiScaleIndex, threshold);
    }
    
    /**
     * Multiscale segmentation using relevant points.
     * @param contour to be segmentated.
     * @param maxSigma for the scale space.
     * @param sigma for the scale space.
     * @param alpha for the contour characteristics.
     * @return a list with relevant points index.
     */
    public List<Integer> bothSegmentation(Contour contour, double maxSigma, 
            double sigma, double alpha){
        Contour iContour;
        Detector iDetector;
        List<List<Integer>> sigmaScalesZerosIndex = new ArrayList<>();
        List<Integer> iScaleZerosIndex;
        List<List<Integer>> sigmaScalesCriticsIndex = new ArrayList<>();
        List<Integer> iScaleCriticsIndex;
        for (double iSigma = sigma; iSigma < maxSigma; iSigma = iSigma * sigma) {
            iContour = contour.gaussianFiltering(iSigma);
            iDetector = new Detector(iContour, this.segmentSizeRatio, 
                    this.segmentLocalRatio, alpha);
            iScaleZerosIndex = iDetector.getZeroCurvaturePointsIndex();
            sigmaScalesZerosIndex.add(iScaleZerosIndex);
            iScaleCriticsIndex = iDetector.getCriticCurvaturePointsIndex();
            sigmaScalesCriticsIndex.add(iScaleCriticsIndex);
        }
        Contour contourFiltrated = contour.gaussianFiltering(sigma);
        double threshold = ZEROS_THRESHOLD;
        List<Integer> zerosMultiScaleIndex = this.getRelevantMultiScaleIndex(contourFiltrated, 0,
                contourFiltrated.size(), sigmaScalesZerosIndex, threshold);
        zerosMultiScaleIndex = this.minCriticsPoints(contourFiltrated, zerosMultiScaleIndex, threshold);
        threshold = CRITICS_THRESHOLD;
        List<Integer> criticsMultiScaleIndex = this.getRelevantMultiScaleIndex(contourFiltrated, 0,
                contourFiltrated.size(), sigmaScalesCriticsIndex, threshold);
        criticsMultiScaleIndex = this.minCriticsPoints(contourFiltrated, criticsMultiScaleIndex, threshold);
        
        Set<Integer> setRelevantPoints = new HashSet<>(zerosMultiScaleIndex);
        setRelevantPoints.addAll(criticsMultiScaleIndex);
        List<Integer> relevantsIndex = new ArrayList<>(setRelevantPoints);
        Collections.sort(relevantsIndex);
        return relevantsIndex;
    }
    
    /**
     * Calculate the error between the original and the relevant points.
     * @param segment contour contour segment.
     * @param startPoint 
     * @param endPoint
     * @return 
     */
    public static double errorSegments(List<Point2D> segment, Point2D startPoint, Point2D endPoint){
        Point2D iPoint;
        double maxDistance = -Double.MAX_VALUE;
        double iDistance;
        for (int i = 0; i < segment.size(); i++){
            iPoint = segment.get(i);
            iDistance = absDistanceBetweenLineAndPoint(startPoint, endPoint, iPoint);
            if (iDistance > maxDistance) {
                maxDistance = iDistance;
            }
        }
        return maxDistance;
    }
    
    /**
     * Calculate the absolute distance between the line defined by two points and
     * a third.
     * @param lineStart
     * @param lineEnd
     * @param point
     * @return 
     */
    public static double absDistanceBetweenLineAndPoint(Point2D lineStart, Point2D lineEnd, Point2D point){
        double x1 = lineStart.getX();
        double y1 = lineStart.getY();

        double x2 = lineEnd.getX();
        double y2 = lineEnd.getY();

        double x0 = point.getX();
        double y0 = point.getY();

        return Math.abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1)/(Math.sqrt(Math.pow(y2 - y1,2) + Math.pow(x2 - x1,2)));
    }
}
