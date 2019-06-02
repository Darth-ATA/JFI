package jfi.shape.relevantspoints;

import java.awt.geom.Point2D;
import jfi.shape.Contour;
import jfi.shape.CurvatureFunction;
import jfi.shape.CurvatureOp;

import java.util.*;
import jfi.shape.fuzzy.FuzzyContour;
import jfi.shape.fuzzy.FuzzyContourFactory;

/**
 * Provides all the functionalities for detect the relevant points of a contour.
 * @author Darth-ATA
 */
public class Detector {
    private static boolean DEBUG = false;
    private static CurvatureOp.Approach curvatureApproach = CurvatureOp.Approach.STANDAR_APPROACH;

    //Minimun segment size.
    static int MINIMUN_SIZE = 10;
    
    // Truncation values.
    static int BASE = 10;
    static int DECIMALS_QUANTITY = 5;

    public static final int CRITIC_POINT = 1 ;
    public static final int CHANGE_POINT = 2;

    private final Contour contour;
    private double segmentSizeRatio;
    private double segmentLocalRatio;
    private CurvatureFunction contourCurvature;
    private double alpha;

    /**
     * Build the Detector with a Contour as default.
     */
    public Detector(){
        this.contour = new Contour();
    }

    /**
     * Build the Detector with the Contour as parameter
     * @param contour contour.
     * @param segmentSizeRatio segment ratio.
     * @param segmentLocalRatio segment ratio for the local ranges characteristic.
     * @param alpha needed for the fuzzy contour.
     */
    public Detector(Contour contour, double segmentSizeRatio, 
            double segmentLocalRatio, double alpha){
        this.contour = contour;
        this.segmentSizeRatio = segmentSizeRatio;
        this.segmentLocalRatio = segmentLocalRatio;
        CurvatureOp curvatureOp = new CurvatureOp(this.segmentSizeRatio);
        curvatureOp.setEstimationMethod(curvatureApproach);
        this.contourCurvature = curvatureOp.apply(this.contour);
        this.alpha = alpha;
    }

    /**
     * Take the contour of the Detector and depending on the segmentLocalRatio
     * finds the indexes of points with min or max curvature value of the contour.
     * @return list of indexes of the min or max curvature points.
     */
    public List<Integer> getCriticCurvaturePointsIndex(){
        int segmentSize = (int)(this.segmentLocalRatio * this.contour.size());
        Double[] curvatureArray = this.contourCurvature.toArray();
        List<Double> curvatureList = Arrays.asList(curvatureArray);
        List<Integer> maxPoints = listMinValuesIndexes(segmentSize, curvatureList);
        List<Integer> minPoints = listMaxValuesIndexes(segmentSize, curvatureList);
        Set<Integer> setExtremePoints = new HashSet<>();

        if (DEBUG) {
            System.out.println("MaxPoints = " + maxPoints.toString());
            System.out.println("MinPoints = " + minPoints.toString());
        }
        setExtremePoints.addAll(maxPoints);
        setExtremePoints.addAll(minPoints);

        List<Integer> extremePoints = new ArrayList<>(setExtremePoints);
        Collections.sort(extremePoints);
        return extremePoints;
    }
    
    /**
     * Take the contour of the Detector and finds sign change of the curvature.
     * @return list of indexes of the sign change curvature points.
     */
    public List<Integer> getZeroCurvaturePointsIndex(){
        List<Double> values = Arrays.asList(contourCurvature.toArray());
        List<Integer> zeros = signChangePointsIndex(values);
        if (DEBUG){
            System.out.println("Zeros = " + zeros.toString());
        }
        return zeros;
    }
    
    /**
     * Take the contour of the Detector and depending on the segmentLocalRatio
     * finds the indexes of points with min or max linearity value of the contour.
     * @return list of indexes of the min or max linearity points.
     */
    public List<Integer> getMinLinearityPointsIndex(){
        FuzzyContour fcontour = FuzzyContourFactory.getLinearityInstance(this.contour,
                this.alpha, 
                (int)(this.segmentSizeRatio * this.contour.size()));
        int segmentSize = (int)(this.segmentLocalRatio * this.contour.size());
        List<Double> contourLinearity = new ArrayList();
        for (jfi.fuzzy.Iterable.FuzzyItem<Point2D> e : fcontour) {
            contourLinearity.add(e.getDegree());
        }
        List<Integer> minPoints = listMinValuesIndexes(segmentSize, contourLinearity);
        Set<Integer> setExtremePoints = new HashSet<>();

        if (DEBUG) {
            System.out.println("MinPoints = " + minPoints.toString());
        }
        setExtremePoints.addAll(minPoints);

        List<Integer> extremePoints = new ArrayList<>(setExtremePoints);
        Collections.sort(extremePoints);
        return extremePoints;
    }
    
    /**
     * Take the contour of the Detector and depending on the segmentLocalRatio
     * finds the indexes of points with min or max linearity value of the contour.
     * @return list of indexes of the min or max linearity points.
     */
    public List<Integer> getMaxLinearityPointsIndex(){
        FuzzyContour fcontour = FuzzyContourFactory.getLinearityInstance(this.contour, 
                this.alpha, 
                (int)(this.segmentSizeRatio * this.contour.size()));
        int segmentSize = (int)(this.segmentLocalRatio * this.contour.size());
        List<Double> contourLinearity = new ArrayList();
        for (jfi.fuzzy.Iterable.FuzzyItem<Point2D> e : fcontour) {
            contourLinearity.add(e.getDegree());
        }
        List<Integer> maxPoints = listMaxValuesIndexes(segmentSize, contourLinearity);
        Set<Integer> setExtremePoints = new HashSet<>();

        if (DEBUG) {
            System.out.println("MaxPoints = " + maxPoints.toString());
        }
        setExtremePoints.addAll(maxPoints);

        List<Integer> extremePoints = new ArrayList<>(setExtremePoints);
        Collections.sort(extremePoints);
        return extremePoints;
    }

    /**
    * Take the relevant points and removes the points marked as relevante in a
    * plane zone.
    * @param relevantPoints the list of the relevant points.
    * @param values the contour list values.
    */
    private static List<Integer> plainZonesEraser(List<Integer> relevantPoints, List<Double> values){
        double current;
        double next;
        double middle;
        int iCurrent;
        int iNext;
        int iMiddle;
        List<Integer> newRelevantPoints = new ArrayList();
        for (int index = 0; index < relevantPoints.size(); index++){
            iCurrent = relevantPoints.get(index);
            iNext = relevantPoints.get((index + 1) % relevantPoints.size());
            if (iNext < iCurrent){
                iMiddle = ((iCurrent + values.size() + iNext)/2) % values.size();
            }
            else{
                iMiddle = (iCurrent + iNext)/2;
            }
            
            current = values.get(iCurrent);
            next = values.get(iNext);
            middle = values.get(iMiddle);
            if (current == middle && middle == next){
                newRelevantPoints.add(iMiddle);
                index++;
            }
            else{
                newRelevantPoints.add(iCurrent);
            }
        }
        return newRelevantPoints;
    }
    
    /**
     * Finds all the min values in the collection.
     * @param segmentSize the size of the interval to check if a value is min.
     * @param values set of values.
     * @return the list of index of the points where the collection item is min.
     */
    private static List<Integer> listMinValuesIndexes(int segmentSize, List<Double> values){
        List<Integer> minPoints = new ArrayList<>();
        boolean add = true;
        double current;
        double next;
        int indexS;
        int modIndex;

        for (int index = 0; index < values.size(); index++){
            current = values.get(index);
            indexS = index - segmentSize/2;
            while (add && indexS < index + segmentSize/2){
                modIndex = Math.floorMod(indexS, values.size());
                if (index != indexS){
                    next = values.get(modIndex);
                    if (current > next){
                        add = false;
                    }
                }
                else{
                    double previous = values.get(Math.floorMod(modIndex - 1, values.size()));
                    next = values.get(Math.floorMod(modIndex + 1, values.size()));
                    if (current >= previous && current >= next){
                        add = false;
                    }
                }
                indexS++;
            }
            if (add){
                minPoints.add(index);
            }
            add = true;
        }        
        
        return plainZonesEraser(minPoints, values);
    }
    
    /**
     * Finds all the max values in the collection.
     * @param segmentSize the size of the interval to check if a value is max.
     * @param values set of values.
     * @return the list of index of the points where the collection item is max.
     */
    private static List<Integer> listMaxValuesIndexes(int segmentSize, List<Double> values){
        List<Integer> maxPoints = new ArrayList<>();
        boolean add = true;
        double current;
        double next;
        int indexS;
        int modIndex;

        for (int index = 0; index < values.size(); index++){
            current = values.get(index);
            indexS = index  - segmentSize/2;
            while (add && indexS < index + segmentSize/2){
                modIndex = Math.floorMod(indexS, values.size());
                if (index != indexS){
                    next = values.get(modIndex);
                    if (current < next){
                        add = false;
                    }
                }
                else{
                    double previous = values.get(Math.floorMod(modIndex - 1, values.size()));
                    next = values.get(Math.floorMod(modIndex + 1, values.size()));
                    if (current <= previous && current <= next){
                        add = false;
                    }
                }
                indexS++;
            }
            if (add){
                maxPoints.add(index);
            }
            add = true;
        }

        return plainZonesEraser(maxPoints, values);
    }
    
    /**
     * Finds all the zero values in the collection.
     * @param values
     * @return the list of index of the points where the collection item is zero.
     */
    private static List<Integer> signChangePointsIndex(List<Double> values){
        List<Integer> signChangePoints = new ArrayList<>();
        double current;
        double previous;
        double next;

        boolean signChange;
        boolean plainZoneEntry;
        boolean plainZoneExit;
        
        int length = 0;
        for (int index = 1; index < values.size(); index++){
            previous = truncate(values.get(index - 1), BASE, DECIMALS_QUANTITY);
            current = truncate(values.get(index), BASE, DECIMALS_QUANTITY);
            next = truncate(values.get((index + 1) % values.size()), BASE, DECIMALS_QUANTITY);
            length++;
            signChange = previous * next < 0;
            plainZoneEntry = current == 0 && previous != 0;
            plainZoneExit = current == 0 && next != 0;

            boolean isPseudoInflection = signChange || plainZoneEntry || plainZoneExit;
            if (isPseudoInflection && length > MINIMUN_SIZE){
                signChangePoints.add(index);
                length = 0;
            }
        }
        return signChangePoints;
    }
    
    /**
     * Truncate the value for have an specific number of decimals.
     * It is a workaround because the zero values have unespected decimals.
     * @param value to be truncated.
     * @param base base for the exponent (always 10).
     * @param exp numer of decimals that we want.
     * @return 
     */
    private static double truncate(double value, int base, double exp){
        return (value * Math.pow(base, exp))/Math.pow(base, exp);
    }
}
