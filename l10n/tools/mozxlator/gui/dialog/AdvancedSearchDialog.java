/*
 * AdvancedSearchDialog.java
 *
 * Created on 6. september 2000, 21:11
 */

package org.mozilla.translator.gui.dialog;

/**
 *
 * @author  Henrik Lynggaard
 * @version 
 */
public class AdvancedSearchDialog extends javax.swing.JDialog {

    /** Creates new form AdvancedSearchDialog */
    public AdvancedSearchDialog(java.awt.Frame parent,boolean modal) {
        super (parent, modal);
        initComponents ();
        pack ();
    }

    /** This method is called from within the constructor to
     * initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is
     * always regenerated by the FormEditor.
     */
private void initComponents() {//GEN-BEGIN:initComponents
jLabel1 = new javax.swing.JLabel();
jLabel2 = new javax.swing.JLabel();
jLabel3 = new javax.swing.JLabel();
jLabel4 = new javax.swing.JLabel();
jComboBox1 = new javax.swing.JComboBox();
jComboBox2 = new javax.swing.JComboBox();
jTextField1 = new javax.swing.JTextField();
jTextField2 = new javax.swing.JTextField();
jCheckBox1 = new javax.swing.JCheckBox();
jCheckBox2 = new javax.swing.JCheckBox();
jComboBox3 = new javax.swing.JComboBox();
jComboBox4 = new javax.swing.JComboBox();
jTextField3 = new javax.swing.JTextField();
jTextField4 = new javax.swing.JTextField();
jCheckBox3 = new javax.swing.JCheckBox();
jComboBox5 = new javax.swing.JComboBox();
jComboBox6 = new javax.swing.JComboBox();
jTextField5 = new javax.swing.JTextField();
jTextField6 = new javax.swing.JTextField();
jPanel1 = new javax.swing.JPanel();
jButton1 = new javax.swing.JButton();
jButton2 = new javax.swing.JButton();
getContentPane().setLayout(new java.awt.GridBagLayout());
java.awt.GridBagConstraints gridBagConstraints1;
addWindowListener(new java.awt.event.WindowAdapter() {
public void windowClosing(java.awt.event.WindowEvent evt) {
closeDialog(evt);
}
}
);

jLabel1.setText("Field");

gridBagConstraints1 = new java.awt.GridBagConstraints();
gridBagConstraints1.gridx = 1;
gridBagConstraints1.gridy = 0;
gridBagConstraints1.fill = java.awt.GridBagConstraints.HORIZONTAL;
getContentPane().add(jLabel1, gridBagConstraints1);


jLabel2.setText("Rule ");

gridBagConstraints1 = new java.awt.GridBagConstraints();
gridBagConstraints1.gridx = 2;
gridBagConstraints1.gridy = 0;
gridBagConstraints1.fill = java.awt.GridBagConstraints.HORIZONTAL;
getContentPane().add(jLabel2, gridBagConstraints1);


jLabel3.setText("text");

gridBagConstraints1 = new java.awt.GridBagConstraints();
gridBagConstraints1.gridx = 3;
gridBagConstraints1.gridy = 0;
gridBagConstraints1.fill = java.awt.GridBagConstraints.HORIZONTAL;
getContentPane().add(jLabel3, gridBagConstraints1);


jLabel4.setText("locale");

gridBagConstraints1 = new java.awt.GridBagConstraints();
gridBagConstraints1.gridx = 4;
gridBagConstraints1.gridy = 0;
gridBagConstraints1.gridwidth = 0;
gridBagConstraints1.fill = java.awt.GridBagConstraints.HORIZONTAL;
getContentPane().add(jLabel4, gridBagConstraints1);



gridBagConstraints1 = new java.awt.GridBagConstraints();
gridBagConstraints1.gridx = 1;
gridBagConstraints1.gridy = 1;
getContentPane().add(jComboBox1, gridBagConstraints1);



gridBagConstraints1 = new java.awt.GridBagConstraints();
gridBagConstraints1.gridx = 2;
gridBagConstraints1.gridy = 1;
getContentPane().add(jComboBox2, gridBagConstraints1);


jTextField1.setText("jTextField1");

gridBagConstraints1 = new java.awt.GridBagConstraints();
gridBagConstraints1.gridx = 3;
gridBagConstraints1.gridy = 1;
getContentPane().add(jTextField1, gridBagConstraints1);


jTextField2.setText("jTextField2");

gridBagConstraints1 = new java.awt.GridBagConstraints();
gridBagConstraints1.gridx = 4;
gridBagConstraints1.gridy = 1;
gridBagConstraints1.gridwidth = 0;
getContentPane().add(jTextField2, gridBagConstraints1);


jCheckBox1.setText("jCheckBox1");

gridBagConstraints1 = new java.awt.GridBagConstraints();
gridBagConstraints1.gridx = 0;
gridBagConstraints1.gridy = 1;
getContentPane().add(jCheckBox1, gridBagConstraints1);


jCheckBox2.setText("jCheckBox2");

gridBagConstraints1 = new java.awt.GridBagConstraints();
gridBagConstraints1.gridx = 0;
gridBagConstraints1.gridy = 2;
getContentPane().add(jCheckBox2, gridBagConstraints1);



gridBagConstraints1 = new java.awt.GridBagConstraints();
gridBagConstraints1.gridx = 1;
gridBagConstraints1.gridy = 2;
getContentPane().add(jComboBox3, gridBagConstraints1);



gridBagConstraints1 = new java.awt.GridBagConstraints();
gridBagConstraints1.gridx = 2;
gridBagConstraints1.gridy = 2;
getContentPane().add(jComboBox4, gridBagConstraints1);


jTextField3.setText("jTextField3");

gridBagConstraints1 = new java.awt.GridBagConstraints();
gridBagConstraints1.gridx = 3;
gridBagConstraints1.gridy = 2;
getContentPane().add(jTextField3, gridBagConstraints1);


jTextField4.setText("jTextField4");

gridBagConstraints1 = new java.awt.GridBagConstraints();
gridBagConstraints1.gridx = 4;
gridBagConstraints1.gridy = 2;
gridBagConstraints1.gridwidth = 0;
getContentPane().add(jTextField4, gridBagConstraints1);


jCheckBox3.setText("jCheckBox3");

gridBagConstraints1 = new java.awt.GridBagConstraints();
getContentPane().add(jCheckBox3, gridBagConstraints1);



gridBagConstraints1 = new java.awt.GridBagConstraints();
getContentPane().add(jComboBox5, gridBagConstraints1);



gridBagConstraints1 = new java.awt.GridBagConstraints();
getContentPane().add(jComboBox6, gridBagConstraints1);


jTextField5.setText("jTextField5");

gridBagConstraints1 = new java.awt.GridBagConstraints();
getContentPane().add(jTextField5, gridBagConstraints1);


jTextField6.setText("jTextField6");

gridBagConstraints1 = new java.awt.GridBagConstraints();
gridBagConstraints1.gridwidth = 0;
getContentPane().add(jTextField6, gridBagConstraints1);



jButton1.setText("jButton1");
  jPanel1.add(jButton1);
  
  
jButton2.setText("jButton2");
  jPanel1.add(jButton2);
  
  
gridBagConstraints1 = new java.awt.GridBagConstraints();
gridBagConstraints1.gridwidth = 0;
gridBagConstraints1.fill = java.awt.GridBagConstraints.HORIZONTAL;
getContentPane().add(jPanel1, gridBagConstraints1);

}//GEN-END:initComponents

    /** Closes the dialog */
    private void closeDialog(java.awt.event.WindowEvent evt) {//GEN-FIRST:event_closeDialog
        setVisible (false);
        dispose ();
    }//GEN-LAST:event_closeDialog

    /**
    * @param args the command line arguments
    */
    public static void main (String args[]) {
        new AdvancedSearchDialog (new javax.swing.JFrame (), true).show ();
    }


// Variables declaration - do not modify//GEN-BEGIN:variables
private javax.swing.JLabel jLabel1;
private javax.swing.JLabel jLabel2;
private javax.swing.JLabel jLabel3;
private javax.swing.JLabel jLabel4;
private javax.swing.JComboBox jComboBox1;
private javax.swing.JComboBox jComboBox2;
private javax.swing.JTextField jTextField1;
private javax.swing.JTextField jTextField2;
private javax.swing.JCheckBox jCheckBox1;
private javax.swing.JCheckBox jCheckBox2;
private javax.swing.JComboBox jComboBox3;
private javax.swing.JComboBox jComboBox4;
private javax.swing.JTextField jTextField3;
private javax.swing.JTextField jTextField4;
private javax.swing.JCheckBox jCheckBox3;
private javax.swing.JComboBox jComboBox5;
private javax.swing.JComboBox jComboBox6;
private javax.swing.JTextField jTextField5;
private javax.swing.JTextField jTextField6;
private javax.swing.JPanel jPanel1;
private javax.swing.JButton jButton1;
private javax.swing.JButton jButton2;
// End of variables declaration//GEN-END:variables

}
