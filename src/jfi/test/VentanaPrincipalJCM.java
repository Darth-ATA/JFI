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
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.Set;
import javax.imageio.ImageIO;
import javax.swing.JFileChooser;
import jfi.fuzzy.DiscreteFuzzySet;
import jfi.shape.Contour;
import jfi.shape.fuzzy.FuzzyContour;



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
              prueba_contornoDifuso(mask);
              
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
        Iterator<Point2D> it = fcontorno.iterator();
        while (it.hasNext()) {
            p = it.next();
            System.out.println(p.toString()+ fcontorno.getMembershipValue(p));
        }
        
        System.out.println(fcontorno.getReferenceSet().toString());
        System.out.println(fcontorno.getAlphaCut(0.2).toString());
        System.out.println(fcontorno.getKernel().toString());
        System.out.println(fcontorno.getSupport().toString());
    } 
    
    
    private void testFuzzyContour(){
        FuzzyContour fcontorno = new FuzzyContour();
        
        Point2D p;
        
        for(int i=0; i<10; i++){
            p = new Point(i,i);
            fcontorno.add(p,(double)i/10);
        }
     
        System.out.println();
        Iterator<Point2D> it = fcontorno.iterator();
        while (it.hasNext()) {
            p = it.next();
            System.out.println(p.toString()+ fcontorno.getMembershipValue(p));
        }
        
        
    }

    
    void prueba_contornoDifuso(ImageMask mask) {
        FuzzyContour c = new FuzzyContour("",mask);
        Point2D p;

        int i = 0;
        Iterator<Point2D> it = c.iterator();
        while (it.hasNext()) {
            p = it.next();
            c.setMembershipValue(p, (double)(i++%11)/10.0 );            
            System.out.println("["+i+"] "+p.toString()+ c.getMembershipValue(p));
        }

        BufferedImage img = c.toImage();
        VentanaImagen vim = new VentanaImagen();
        vim.lienzoImagen.setImage(img);
        vim.setTitle("Contorno");
        this.escritorio.add(vim);
        vim.setVisible(true);

        Set alpha_cut = c.getAlphaCut(0.4f);
        Contour alpha_cut_contour = new Contour(alpha_cut);
        img = alpha_cut_contour.toImage();
        vim = new VentanaImagen();
        vim.lienzoImagen.setImage(img);
        vim.setTitle("Alfa-corte");
        this.escritorio.add(vim);
        vim.setVisible(true);

        Set kernel = c.getKernel();
        Contour kernel_contour = new Contour(kernel);
        img = kernel_contour.toImage();
        vim = new VentanaImagen();
        vim.lienzoImagen.setImage(img);
        vim.setTitle("Kernel");
        this.escritorio.add(vim);
        vim.setVisible(true);

        Set soporte = c.getSupport();
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
