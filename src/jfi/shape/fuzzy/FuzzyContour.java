package jfi.shape.fuzzy;

import java.awt.Rectangle;
import java.awt.geom.Point2D;
import java.awt.image.BufferedImage;
import java.awt.image.WritableRaster;
import java.util.Set;
import java.util.Map.Entry;
import jfi.fuzzy.DiscreteFuzzySet;
import jfi.shape.Contour;
import jfi.shape.ImageMask;

/**
 * A <code>FuzzyContour</code> represents a fuzzy set over the countour-points
 * domain. It has two main uses:
 * <ul>
 * <li>To represent a fuzzy property associated to the contour-points (for example, 
 *     the verticity degree of each point in the contour)
 * </li>
 * <li>To model a contour in a fuzzy way (i.e, in which degree a given point 
 *     belongs to a contour shape)
 * </li>
 * </ul>
 *
 *
 * @author Jesús Chamorro Martínez (jesus@decsai.ugr.es)
 */
public final class FuzzyContour extends DiscreteFuzzySet<Point2D> {

    /**
     * Constructs an empty fuzzy contour with an empty label.
     */
    public FuzzyContour() {
        super();
    }

    /**
     * Constructs an empty fuzzy contour.
     *
     * @param label label of the fuzzy set
     */
    public FuzzyContour(String label) {
        super(label);
    }

    /**
     * Constructs a fuzzy contour from a mask image setting all the membership
     * values to 1.0.
     *
     * It is assumed that the mask contains a single connected component. If
     * not, only the contour of the first shape (starting from the top-left) is
     * created
     *
     * @param label label of the fuzzy set
     * @param mask the mask image
     */
    public FuzzyContour(String label, ImageMask mask) {
        this(label, new Contour(mask));
    }

    /**
     * Constructs a fuzzy contour from a mask image with an empty label and all 
     * the membership values to 1.0.
     *
     * It is assumed that the mask contains a single connected component. If
     * not, only the contour of the first shape (starting from the top-left) is
     * created
     *
     * @param mask the mask image
     */
    public FuzzyContour(ImageMask mask) {
        this("",mask);
    }
    
    /**
     * Constructs a fuzzy contour from a crisp contour setting all the
     * membership values to 1.0.
     *
     * @param label label of the fuzzy se
     * @param contour the crisp contour
     */
    public FuzzyContour(String label, Contour contour) {
        this(label);
        for(Point2D point:contour){
            this.add(point);  //By default, membership degree is set to 1.0
        }
    }
    
    /**
     * Constructs a fuzzy contour from a crisp contour with an empty label and 
     * setting all the membership values to 1.0.
     *
     * @param contour the crisp contour
     */
    public FuzzyContour(Contour contour) {
        this("",contour);
    }

    /**
     * Appends the specified point to fuzzy contour with membership degree 1.0.
     *
     * @param point point to be appended to this list
     * @return <tt>true</tt>
     */
    public boolean add(Point2D point) {
        return this.add(point, 1.0);

    }

    /**
     * Draws the contour points into an image, using the membership degrees for
     * greylevel estimation.
     * 
     * @return an image with the contour drawn
     */
    public BufferedImage toImage() {
        BufferedImage img = null;
        if (this.size() > 0) {
            Rectangle bounds = this.getBounds();
            img = new BufferedImage(bounds.width, bounds.height, BufferedImage.TYPE_BYTE_GRAY);
            int x, y, grey_level;

            WritableRaster imgRaster = img.getRaster();
            for (Entry<Point2D, Double> e : this) {
                x = (int) e.getKey().getX() - bounds.x;
                y = (int) e.getKey().getY() - bounds.y;
                grey_level = (int) (255.0 * e.getValue());
                imgRaster.setSample(x, y, 0, grey_level);
            }
        }
        return img;
    }

    /**
     * Returns an integer Rectangle that completely encloses the contour
     *
     * @return an integer Rectangle that completely encloses the contour.
     */
    private Rectangle getBounds() {
        int maxX = 0, maxY = 0;
        int minX = Integer.MAX_VALUE;
        int minY = Integer.MAX_VALUE;

        for (Entry<Point2D,Double> e : this) {
            Point2D point = e.getKey();
            if (maxX < point.getX()) {
                maxX = (int) point.getX();
            }
            if (minX > point.getX()) {
                minX = (int) point.getX();
            }
            if (maxY < point.getY()) {
                maxY = (int) point.getY();
            }
            if (minY > point.getY()) {
                minY = (int) point.getY();
            }
        }
        return new Rectangle(minX, minY, maxX-minX+1, maxY-minY+1);
    }

    /**
     * Return the crisp contour corresponding to the reference set 
     * @return the crisp contour corresponding to the reference set 
     */
    public Contour getContourReferenceSet(){
        Set reference_set = super.getReferenceSet();
        return new Contour(reference_set);
    }
}
