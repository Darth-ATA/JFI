/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * VentanaPrincipalJCM.java
 *
 * Created on 02-feb-2011, 17:26:17
 */

package jfi.test;

import jfi.shape.ImageMaskIO;
import jfi.shape.ImageMask;
import java.awt.Point;
import java.awt.geom.Point2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.imageio.ImageIO;
import javax.swing.JFileChooser;
import jfi.fuzzy.DiscreteFuzzySet;
import jfi.fuzzy.LevelSet;
import jfi.fuzzy.cardinal.EDCardinal;
import jfi.fuzzy.cardinal.SigmaCount;
import jfi.fuzzy.membershipfunction.PiecewiseFunction;
import jfi.fuzzy.membershipfunction.TrapezoidalFunction;
import jfi.shape.Contour;
import jfi.shape.fuzzy.FuzzyContour;
import jfi.shape.fuzzy.FuzzyContourFactory;
import jfi.texture.fuzzy.FuzzyTexture;
import jfi.texture.fuzzy.FuzzyTextureFactory;



/**
 *
 * @author Jesús Chamorro
 */
public class VentanaPrincipalJCM extends javax.swing.JFrame {

    /** Creates new form VentanaPrincipal */
    public VentanaPrincipalJCM() {
        initComponents();
        this.setSize(1340, 800);
        setLocationRelativeTo(null);
        
        //Pruebas
        //testFuzzySets();
        //testFuzzyContour();
        //pruebas_cardinales();
        
        try {
            FuzzyTexture ft = FuzzyTextureFactory.getInstance(FuzzyTextureFactory.TYPE_DIRECTIONALITY_TAMURA_ABBADENI);
            BufferedImage bi;
            bi = ImageIO.read(new File("C:\\Texturas\\Direccionalidad\\BD\\encuesta\\30.jpg"));
            double md = ft.membershipDegree(bi);
            System.out.println(md);
            
        } catch (IOException ex) {
            Logger.getLogger(VentanaPrincipalJCM.class.getName()).log(Level.SEVERE, null, ex);
        }
        
        
        
    }
    
    
    /** This method is called from within the constructor to
     * initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is
     * always regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        escritorio = new javax.swing.JDesktopPane();
        jMenuBar1 = new javax.swing.JMenuBar();
        jMenu1 = new javax.swing.JMenu();
        menuAbrir = new javax.swing.JMenuItem();
        menuGuardar = new javax.swing.JMenuItem();
        menuMascara = new javax.swing.JMenuItem();

        setDefaultCloseOperation(javax.swing.WindowConstants.EXIT_ON_CLOSE);
        setTitle("Pruebas Jesús");
        getContentPane().add(escritorio, java.awt.BorderLayout.CENTER);

        jMenu1.setText("Archivo");

        menuAbrir.setText("Abrir");
        menuAbrir.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                menuAbrirActionPerformed(evt);
            }
        });
        jMenu1.add(menuAbrir);

        menuGuardar.setText("Guardar");
        menuGuardar.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                menuGuardarActionPerformed(evt);
            }
        });
        jMenu1.add(menuGuardar);

        menuMascara.setText("Máscara");
        menuMascara.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                menuMascaraActionPerformed(evt);
            }
        });
        jMenu1.add(menuMascara);

        jMenuBar1.add(jMenu1);

        setJMenuBar(jMenuBar1);

        pack();
    }// </editor-fold>//GEN-END:initComponents

    private void menuAbrirActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_menuAbrirActionPerformed
        BufferedImage img;
        JFileChooser dlg = new JFileChooser();

        int resp = dlg.showOpenDialog(this);
        if( resp == JFileChooser.APPROVE_OPTION) {
           try{
              File f = dlg.getSelectedFile(); 
              img = ImageIO.read(f);
              VentanaImagen vi = new VentanaImagen();
              vi.lienzoImagen.setImage(img);
              this.escritorio.add(vi);
              vi.setVisible(true);
           }catch(Exception ex){
             System.err.println("Error al leer la imagen");
           }
        }
    }//GEN-LAST:event_menuAbrirActionPerformed

    private void menuMascaraActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_menuMascaraActionPerformed
        ImageMask mask=null;
        JFileChooser dlg = new JFileChooser();

        int resp = dlg.showOpenDialog(this);
        if( resp == JFileChooser.APPROVE_OPTION) {
           try{
              File f = dlg.getSelectedFile(); 
              mask = ImageMaskIO.read(f);
              VentanaImagen vi = new VentanaImagen();
              vi.setTitle("Mascara");
              vi.lienzoImagen.setImage(mask);
              this.escritorio.add(vi);
              vi.setVisible(true);
              
              //Pruebas
              testVerticity(mask);
              //prueba_contornoDifuso(mask);
              
           }catch(Exception ex){
             System.err.println("Error al leer la imagen ("+ex.getMessage()+")");
           }
        }
    }//GEN-LAST:event_menuMascaraActionPerformed

    private void menuGuardarActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_menuGuardarActionPerformed
        VentanaImagen vi = (VentanaImagen) escritorio.getSelectedFrame();
        if (vi != null) {
            JFileChooser dlg = new JFileChooser();
            int resp = dlg.showSaveDialog(this);
            if (resp == JFileChooser.APPROVE_OPTION) {
                File f = dlg.getSelectedFile();

                try {
                    LienzoImagen lienzo = (LienzoImagen) vi.lienzoImagen;
                    BufferedImage img = lienzo.getImage();
                    if (img != null) {
                        ImageIO.write(img, "png", f);
                        vi.setTitle(f.getName());
                    }
                } catch (Exception ex) {
                    System.err.println("Error al guardar la imagen");
                }
            }
        }
    }//GEN-LAST:event_menuGuardarActionPerformed

    
    
    //// Pruebas
    //// -------
    
    private void pruebas_cardinales(){
        DiscreteFuzzySet fset = new DiscreteFuzzySet();   
        
        fset.add(new Point(1,1),0.0);
        fset.add(new Point(2,1),0.3);
        fset.add(new Point(3,1),0.5);
        fset.add(new Point(4,1),0.9);
        fset.add(new Point(5,1),0.9);        
        fset.add(new Point(6,1),0.9);
        fset.add(new Point(7,1),0.9);
        fset.add(new Point(8,1),1.0);
        fset.add(new Point(9,1),1.0);
        fset.add(new Point(10,1),1.0);
        
        SigmaCount sigma = new SigmaCount(fset);
        System.out.println(sigma.getValue());
        
        ArrayList<Entry> al = new ArrayList(fset.entrySet());
        Comparator comp = Map.Entry.comparingByValue();
        Collections.sort(al, Collections.reverseOrder(comp));
        System.out.println(al.toString());
        
        EDCardinal ed = new EDCardinal(fset);
        System.out.println(ed.toString());
        
         System.out.print("Más probable: ");
         for(int i=0;i<ed.mostLikely().length;i++)
             System.out.print(ed.mostLikely()[i]+" ");
        
    }
    
    
    
    private void testFuzzySets(){
        DiscreteFuzzySet<Point2D> fcontorno;
        fcontorno = new DiscreteFuzzySet<>();
        Point2D p;
        
        for(int i=0; i<10; i++){
            p = new Point(i,i);
            fcontorno.add(p,(double)i/10);
        }
        
        fcontorno.add(new Point(7,7),1.0);
        fcontorno.remove(new Point(8,8));
        
        System.out.println();
        Iterator<Entry<Point2D,Double>> it = fcontorno.iterator();
        Entry<Point2D,Double> e;
        while (it.hasNext()) {
            e = it.next();
            p =e.getKey();
            System.out.println(p.toString()+ e.getValue());
        }
        
        System.out.println(fcontorno.getReferenceSet().toString());
        System.out.println(fcontorno.alphaCut(0.2).toString());
        System.out.println(fcontorno.kernel().toString());
        System.out.println(fcontorno.support().toString());
    } 
    
    
    private void testVerticity(ImageMask mask){
        
        Contour contour = new Contour(mask);
        FuzzyContour fcontour = FuzzyContourFactory.getInstance(contour, FuzzyContourFactory.TYPE_VERTICITY);
         
        System.out.println("\nTamaño del contorno: "+fcontour.size());     
        SigmaCount sigma = new SigmaCount(fcontour);
        System.out.println("Sigma-count sobre original: "+sigma.getValue());
        EDCardinal ed = new EDCardinal(fcontour);
        System.out.println("ED sobre original : "+ed.toString());  
        System.out.println("Más probable: "+ed.mostLikely()[0]+" con probablididad "+ed.probability(ed.mostLikely()[0]));  
        
        BufferedImage img = fcontour.toImage();
        VentanaImagen vim = new VentanaImagen();
        vim.lienzoImagen.setImage(img);
        vim.setTitle("Verticidad");
        this.escritorio.add(vim);
        vim.setVisible(true);
        
        DiscreteFuzzySet<Point2D> maximos = new DiscreteFuzzySet<>();;
        ArrayList<Entry> entry_list = new ArrayList(fcontour.entrySet());
        double id,idw,ipd,isd;
        int wsize,idw_index,w;
        boolean es_maximo;
                
        wsize=(int) (Contour.DEFAULT_WINDOW_RATIO_SIZE * entry_list.size()); 
        
        System.out.println("Wsize: " +wsize);
        
        for(int i=0; i<entry_list.size(); i++){
            
            id = (Double)entry_list.get(i).getValue();
            for(w=-wsize, es_maximo=true; w<=wsize && es_maximo; w++){
                idw_index = (i + w + entry_list.size()) % entry_list.size();
                idw = (Double)entry_list.get(idw_index).getValue();
                if(w!=0 && id<=idw) es_maximo = false;
            }
            if(es_maximo){
                maximos.add((Point2D)entry_list.get(i).getKey(), id);
            }
        }
        
        System.out.println("Tamaño del conjunto de máximos: "+maximos.size());
        sigma = new SigmaCount(maximos);
        System.out.println("Sigma-count sobre máximos: "+sigma.getValue());
        ed = new EDCardinal(maximos);
        System.out.println("ED sobre máximos : "+ed.toString());  
        System.out.println("Más probable: "+ed.mostLikely()[0]+" con probablididad "+ed.probability(ed.mostLikely()[0]));  
        
    }
    
    private void testFuzzyContour(){
        FuzzyContour fcontorno = new FuzzyContour();
        
        Point2D p;
        
        for(int i=0; i<10; i++){
            p = new Point(i,i);
            fcontorno.add(p,(double)i/10);
        }
     
        System.out.println();
        Iterator<Entry<Point2D,Double>> it = fcontorno.iterator();
        Entry<Point2D,Double> e;
        while (it.hasNext()) {
            e = it.next();
            System.out.println(e.getKey().toString()+ e.getValue());
        }
        
        
    }
    
    void prueba_contornoDifuso(ImageMask mask) {
        FuzzyContour c = new FuzzyContour("",mask);
        Point2D p;

        int i = 0;
        Iterator<Entry<Point2D,Double>> it = c.iterator();
        Entry<Point2D,Double> e;
        while (it.hasNext()) {
            e = it.next();
            p = e.getKey();
            c.setMembershipDegree(p, (double)(i++%11)/10.0 );            
            System.out.println("["+i+"] "+p.toString()+ c.membershipDegree(p));
        }

        BufferedImage img = c.toImage();
        VentanaImagen vim = new VentanaImagen();
        vim.lienzoImagen.setImage(img);
        vim.setTitle("Contorno");
        this.escritorio.add(vim);
        vim.setVisible(true);

        Set alpha_cut = c.alphaCut(0.4f);
        Contour alpha_cut_contour = new Contour(alpha_cut);
        img = alpha_cut_contour.toImage();
        vim = new VentanaImagen();
        vim.lienzoImagen.setImage(img);
        vim.setTitle("Alfa-corte");
        this.escritorio.add(vim);
        vim.setVisible(true);

        Set kernel = c.kernel();
        Contour kernel_contour = new Contour(kernel);
        img = kernel_contour.toImage();
        vim = new VentanaImagen();
        vim.lienzoImagen.setImage(img);
        vim.setTitle("Kernel");
        this.escritorio.add(vim);
        vim.setVisible(true);

        Set soporte = c.support();
        Contour soporte_contour = new Contour(soporte);
        img = soporte_contour.toImage();
        vim = new VentanaImagen();
        vim.lienzoImagen.setImage(img);
        vim.setTitle("Soporte");
        this.escritorio.add(vim);
        vim.setVisible(true);
        
        Set reference_set = c.getReferenceSet();
        Contour reference_set_contour = new Contour(reference_set);
        img = reference_set_contour.toImage();
        vim = new VentanaImagen();
        vim.lienzoImagen.setImage(img);
        vim.setTitle("Conjutno referente");
        this.escritorio.add(vim);
        vim.setVisible(true);
    }

    public static void pruebaLevelSet(){
        DiscreteFuzzySet fset = new DiscreteFuzzySet();   
        
        fset.add(1,0.0);
        fset.add(2,0.3);
        fset.add(3,0.5);
        fset.add(4,0.9);
        fset.add(5,0.9);        
        fset.add(6,0.9);
        fset.add(7,0.9);
        fset.add(8,1.0);
        fset.add(9,1.0);
        fset.add(10,1.0);
        
        fset.add(11,0.0);
        fset.add(12,0.3);
        fset.add(13,0.5);
        fset.add(14,0.9);
        fset.add(15,0.9);        
        fset.add(16,0.9);
        fset.add(17,0.9);
        fset.add(18,1.0);
        fset.add(19,1.0);
        fset.add(20,1.0);
        
        LevelSet l = new LevelSet(fset);
        
        System.out.println("LevelSet: " + l.toString());
        
        
    }
    
    public static void pruebaMembershipFuctions(){
        TrapezoidalFunction f1 = new TrapezoidalFunction(340,350,360,360);
        double x[] = {339,340,345,350,352,355,357,360,361,359,0,5,10,15,20,21};
        System.out.println(f1.toString());
        for(int i=0;i<x.length;i++){
            System.out.println(x[i]+": "+f1.apply(x[i]));
        }
        TrapezoidalFunction f2 = new TrapezoidalFunction(0,0,10,20);
        System.out.println(f2.toString());
        for(int i=0;i<x.length;i++){
            System.out.println(x[i]+": "+f2.apply(x[i]));
        }
        PiecewiseFunction f3 = new PiecewiseFunction();
        f3.add(f1);
        f3.add(f2);
        System.out.println(f3.toString());
        for(int i=0;i<x.length;i++){
            System.out.println(x[i]+": "+f3.apply(x[i]));
        }
        
    }

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        
        
        /*
         * Set the Nimbus look and feel
         */
        //<editor-fold defaultstate="collapsed" desc=" Look and feel setting code (optional) ">
        /*
         * If Nimbus (introduced in Java SE 6) is not available, stay with the
         * default look and feel. For details see
         * http://download.oracle.com/javase/tutorial/uiswing/lookandfeel/plaf.html
         */
        try {
            for (javax.swing.UIManager.LookAndFeelInfo info : javax.swing.UIManager.getInstalledLookAndFeels()) {
                if ("Nimbus".equals(info.getName())) {
                    javax.swing.UIManager.setLookAndFeel(info.getClassName());
                    break;
                }
            }
        } catch (ClassNotFoundException ex) {
            java.util.logging.Logger.getLogger(VentanaPrincipalJCM.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (InstantiationException ex) {
            java.util.logging.Logger.getLogger(VentanaPrincipalJCM.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (IllegalAccessException ex) {
            java.util.logging.Logger.getLogger(VentanaPrincipalJCM.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (javax.swing.UnsupportedLookAndFeelException ex) {
            java.util.logging.Logger.getLogger(VentanaPrincipalJCM.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        }
        //</editor-fold>
        //</editor-fold>
        //</editor-fold>
        //</editor-fold>

        //pruebaMembershipFuctions();
        
       
        
        new VentanaPrincipalJCM().setVisible(true);
    }
    
    
    
    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JDesktopPane escritorio;
    private javax.swing.JMenu jMenu1;
    private javax.swing.JMenuBar jMenuBar1;
    private javax.swing.JMenuItem menuAbrir;
    private javax.swing.JMenuItem menuGuardar;
    private javax.swing.JMenuItem menuMascara;
    // End of variables declaration//GEN-END:variables

}
