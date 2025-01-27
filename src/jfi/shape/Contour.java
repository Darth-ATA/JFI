/*
  Contour represented as a collection of 2D points

  @author Jesús Chamorro Martínez (jesus@decsai.ugr.es)
  @author Luis Suárez Lloréns
*/
package jfi.shape;

import java.awt.Point;
import java.awt.Rectangle;
import java.awt.geom.Point2D;
import java.awt.image.BufferedImage;
import java.awt.image.WritableRaster;
import java.security.InvalidParameterException;
import java.util.ArrayList;
import java.util.Collection;

public class Contour extends ArrayList<Point2D> implements Segmentable{ 
    
    /**
     *  Default ratio for window creation.
     */
    public static final double DEFAULT_WINDOW_RATIO_SIZE = 1.0/15;
    
    /**
     *  Default offset.
     */
    public final static int DEFAULT_OFFSET = 0;
    /**
     * Flag to know if the clockwise direcction is used in the contour round
     */
    private final boolean clockwise = true;
    
    /**
     * Constructs an empty contour.
     */
    public Contour(){
        super();
    }
    /**
     * Constructs a contour containing the points of the specified
     * collection, in the order they are returned by the collection's
     * iterator.
     *
     * @param points the collection whose elements are to be placed into this contour
     * @throws NullPointerException if the specified collection is null
     */
    public Contour(Collection<Point> points){
        super(points);
    }
    
    /**
     * Constructs a contour from an mask image. 
     * 
     * It is assumed that the mask contains a single connected component. If not,
     * only the contour of the first shape (starting from the top-left) is created
     * 
     * @param mask the mask image
     */
    public Contour(ImageMask mask) {
        super();
        maskToContour(mask);
    }
    
    /**
     * Returns <tt>true</tt> if the clockwise direcction is used in the
     * contour iteration.
     * 
     * @return the clockwise direcction state
     */
    public boolean isClockwise(){
        return clockwise;
    }
    
    /**
     * Add points to the contour from an mask image. 
     * 
     * It is assumed that the mask contains a single connected component. If not,
     * only the contour of the first shape (starting from the top-left) is created
     * 
     * @param mask the mask image
     */
    private void maskToContour(ImageMask mask){
        if(mask!=null){                       
            int r, c, S = 6, iter; 
            Point currentPoint;
            boolean pointFound, firstIteration = true;
            Point initialPoint = null;
            //Get the first point of the contour
            for (r = 0, pointFound = false; r < mask.getHeight() && !pointFound; r++) {
                for (c = 0; c < mask.getWidth() && !pointFound; c++) {
                    if (mask.getRaster().getSample(c, r, 0) != 0) {
                        initialPoint = new Point(c, r);
                        pointFound = true;
                    }
                }
            }
            if (pointFound) {
                currentPoint = new Point(initialPoint);
                while (((currentPoint.x != initialPoint.x) || (currentPoint.y != initialPoint.y)) || (firstIteration)) {
                    this.add(new Point(currentPoint));
                    pointFound = false;
                    iter = 0;
                    while ((!pointFound) && (iter < 3)) {
                        if (pointFound = isBorderPoint((8 + (S - 1)) % 8, currentPoint, mask)) {
                            currentPoint = freemanStep((8 + (S - 1)) % 8, currentPoint);
                            S = (8 + (S - 2)) % 8;  //Change direction
                        } else if (pointFound = isBorderPoint(S, currentPoint, mask)) {
                            currentPoint = freemanStep(S, currentPoint);
                        } else if (pointFound = isBorderPoint((S + 1) % 8, currentPoint, mask)) {
                            currentPoint = freemanStep((S + 1) % 8, currentPoint);
                        } else {
                            S = (S + 2) % 8;  //Change direction
                        }
                        iter++;
                    }
                    firstIteration = false;
                }
            }
        }      
    }
    
    /**
     * Draws the contour points into an image
     * 
     * @param bounded if <tt>true</tt>, the image size will be set to the 
     * rectangle that bounds the contour (in that case, the countour point 
     * locations in the image will not necessarily match with the actual 
     * coordinates values); if <tt>false</tt>, the actual coordinates 
     * values will be used.
     * @param transparency if <tt>true</tt>, alpha-component is used to set 
     * transparent the background and opaque the contour points (both background 
     * and foreground have rgb values [0,0,0]); if <tt>false</tt>, a grey-level 
     * image is returned with black background and white values for the contour 
     * points 
     * 
     * @return an image with the contour drawn
     */
    public BufferedImage toImage(boolean bounded, boolean transparency) {
        BufferedImage img = null;
        if (!isEmpty()) {
            Rectangle bounds = this.getBounds();
            int width = bounded ? bounds.width : bounds.width + bounds.x;
            int height = bounded ? bounds.height : bounds.height + bounds.y;
            int type = transparency ? BufferedImage.TYPE_INT_ARGB : BufferedImage.TYPE_BYTE_GRAY;
            img = new BufferedImage(width, height, type);
            
            int x, y, band = transparency?3:0;
            Point offset = bounded ? bounds.getLocation() : new Point(0,0);
            WritableRaster imgRaster = img.getRaster();
            for (Point2D point : this) {
                x = (int) Math.round(point.getX()) - offset.x;
                y = (int) Math.round(point.getY()) - offset.y;
                imgRaster.setSample(x, y, band, 255);
            }
        }
        return img;
    }
    
    /**
     * Draws the contour points into an image using the actual coordinates 
     * values (without bounded fit) 
     *       
     * @return a grey-level image with black background and the contour drawn 
     * in white  
     */
    public BufferedImage toImage() {
        return toImage(false,false);
    }
    
    /**
     * Returns an integer Rectangle that completely encloses the contour
     *
     * @return an integer Rectangle that completely encloses the contour.
     */
    public Rectangle getBounds() {
        int maxX = 0, maxY = 0;
        int minX = Integer.MAX_VALUE;
        int minY = Integer.MAX_VALUE;
        for (Point2D point : this) {
            if (maxX < Math.round(point.getX())) {
                maxX = (int) Math.round(point.getX());
            }
            if (minX > Math.round(point.getX())) {
                minX = (int) Math.round(point.getX());
            }
            if (maxY < Math.round(point.getY())) {
                maxY = (int) Math.round(point.getY());
            }
            if (minY > Math.round(point.getY())) {
                minY = (int) Math.round(point.getY());
            }
        }
        return new Rectangle(minX, minY, maxX - minX + 1, maxY - minY + 1);
    }
    
    
    /**
     * Applies a Gaussian filter to the contour
     * 
     * @param sigma standard deviation of the gaussian function
     * 
     * @return the filtered contour 
     */
    public Contour gaussianFiltering(double sigma){
        ArrayList<Double> kernel = ContourFilteringOp.gaussianKernel(sigma);
        ContourFilteringOp filteringOp = new ContourFilteringOp(kernel);
        return filteringOp.apply(this);     
    }
    
    /**
     * Calculates the curvature of the contour using the default values
     * 
     * @return Curvature of the contour
     */
    public CurvatureFunction curvature(){
        CurvatureOp curvatureOp = new CurvatureOp(DEFAULT_WINDOW_RATIO_SIZE);
        return curvatureOp.apply(this);
    }
    
    
    /**
     * Checks if the point at a given direction from a point is part of the figure
     * 
     * The direction uses Freeman's chain code.
     * 
     * @param direction
     * @param actualPoint
     * @param image
     * @return true if the point is part of the figure. false in other case. 
     */
    private boolean isBorderPoint(int direction, Point actualPoint, BufferedImage image){
        Point nextStep = freemanStep(direction,actualPoint);
        if(0 <= nextStep.x && nextStep.x < image.getWidth() && 
           0 <= nextStep.y && nextStep.y < image.getHeight())
            if (image.getRaster().getSample(nextStep.x, nextStep.y, 0) !=0 )
                return true;
        return false;
    }
    
    /**
     * Calculates the point at given direction from a point
     * 
     * The direction uses Freeman's chain code.
     * 
     * @param direction 
     * @param actualPoint
     * 
     * @return Point at given direction from actualPoint
     */
    private Point freemanStep(int direction, Point actualPoint){
        Point nextStep = new Point(actualPoint);     
        switch (direction){
            case 0:
                nextStep.x++;
                break;
            case 1: 
                nextStep.x++;
                nextStep.y++;
                break;
            case 2:
                nextStep.y++;
                break;
            case 3: 
                nextStep.x--;
                nextStep.y++;
                break;
            case 4:
                nextStep.x--;
                break;
            case 5: 
                nextStep.x--;
                nextStep.y--;
                break;
            case 6:
                nextStep.y--;
                break;
            case 7: 
                nextStep.x++;
                nextStep.y--;
                break;   
        }      
        return nextStep;
    }
       
    /**
     * Return the contour segment connecting the points <code>start</code> and 
     * <code>end</code> (both included). If <code>start==end</code>, a segment 
     * with a single point is returned.
     * 
     * @param start the starting point
     * @param end the ending point
     * @param clockwise reaching <code>end</code> from <code>start</code> in 
     * clockwise direcction  
     * @return the contour segment 
     */
    @Override
    public ArrayList<Point2D> getSegment(Point2D start, Point2D end, boolean clockwise){
        if(!contains(start) || !contains(end)){
            throw new InvalidParameterException("Points must be contained in the contour.");
        }
        ArrayList<Point2D> segment = new ArrayList<>();
        ContourIterator it = new ContourIterator(this,start,clockwise);
        while(!it.isPrevious(end)){
            segment.add(it.next());         
        }
        return segment;
    }
    
    public ArrayList<Point2D> getSegment(int start, int end, boolean clockwise){
         ArrayList<Point2D> segment = new ArrayList<>(); 
         int i=start;
         while(i!=end){
             //TODO
             //segment.add(get(i));
             //i= clockwise ? (i+1)%size() : (i-1+size())%size();
         }
         return segment;
    }
    
    /**
     * Return the contour segment of size <code>segment_size</code> starting 
     * from the point <code>start</code> 
     * 
     * @param start the starting point
     * @param segment_size the segment size
     * @param clockwise reaching the end point of the segment from <code>start</code> 
     * in clockwise direcction
     * @return the contour segment
     */
    @Override
    public ArrayList<Point2D> getSegment(Point2D start, int segment_size, boolean clockwise){
        if(!contains(start)){
            throw new InvalidParameterException("Start point must be contained in the contour.");
        }
        if(segment_size > this.size()){
            throw new InvalidParameterException("Segment size bigger than contour size.");
        }        
        int index_end = clockwise ? (indexOf(start)+segment_size-1)%size() : (indexOf(start)-segment_size+1+size())%size(); 
        return this.getSegment(start,get(index_end),clockwise);
    }

    /**
     * Return the point located an <code>offset</code> distance from the element 
     * at the specified position <code>index</code> in the contour. The flag 
     * <code>clockwise</code>  will be taked into acount in order to locate the point.
     * 
     * If <code>offset</code> is negative, the new point will be reached following 
     * the backward way
     *
     * @param index index of the element 
     * @param offset distance from the element
     * @return the point located an <code>offset</code> distance from the element
     * at the specified position
     */
    public Point2D getPointBeside(int index, int offset) {
        if (index >= this.size())
            throw new IndexOutOfBoundsException("Index: "+index+", Size: "+size());
        offset = offset % this.size();
        if (!clockwise) {
            offset = -offset;
        }
        int new_index = (index + offset + this.size()) % this.size();
        return this.get(new_index);
    }

    /**
     * Return the point located an <code>offset</code> distance from the point
     * <code>point</code> in the contour. The flag <code>clockwise</code>  will 
     * be taked into acount in order to locate the point.
     * 
     * If <code>offset</code> is negative, the new point will be reached following 
     * the backward way
     *
     * @param point contour point 
     * @param offset distance from the element
     * @return the point located an <code>offset</code> distance from the element
     * at the specified position
     */
    public Point2D getPointBeside(Point2D point, int offset){
        return getPointBeside(indexOf(point),offset);
    }
     
}
