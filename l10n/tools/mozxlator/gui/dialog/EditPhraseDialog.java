/*
 * The contents of this file are subject to the Mozilla Public
 * License Version 1.1 (the "License"); you may not use this file
 *  except in compliance with the License. You may obtain a copy of
 * the License at http://www.mozilla.org/MPL/

 * Software distributed under the License is distributed on an "AS
 * IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
 * implied. See the License for the specific language governing
 * rights and limitations under the License.
 *
 * The Original Code is MozillaTranslator (Mozilla Localization Tool)
 *
 * The Initial Developer of the Original Code is Henrik Lynggaard Hansen
 *
 * Portions created by Henrik Lynggard Hansen are
 * Copyright (C) Henrik Lynggaard Hansen.
 * All Rights Reserved.
 *
 * Contributor(s):
 * Henrik Lynggaard Hansen (Initial Code)
 *
 */
package org.mozilla.translator.gui.dialog;

import javax.swing.*;
import org.mozilla.translator.kernel.*;
import org.mozilla.translator.datamodel.*;
import org.mozilla.translator.gui.*;
/**
 *
 * @author  Henrik Lynggaard Hansen
 * @version
 */
public class EditPhraseDialog extends javax.swing.JDialog {

    /** Creates new form PhrasePropertyWindow */
    public EditPhraseDialog() 
    {
        super (MainWindow.getDefaultInstance(),"Edit Phrase",true);
        initComponents ();
        getRootPane().setDefaultButton(okayButton);
        pack();
        Utils.placeFrameAtCenter(this);
    }

    /** This method is called from within the constructor to
     * initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is
     * always regenerated by the FormEditor.
     */
private void initComponents() {//GEN-BEGIN:initComponents
basicPanel = new javax.swing.JPanel();
InstallLabel = new javax.swing.JLabel();
installField = new javax.swing.JTextField();
componentLabel = new javax.swing.JLabel();
componentField = new javax.swing.JTextField();
fileLabel = new javax.swing.JLabel();
fileField = new javax.swing.JTextField();
keyLabel = new javax.swing.JLabel();
keyField = new javax.swing.JTextField();
subcomponentLabel = new javax.swing.JLabel();
subcomponentField = new javax.swing.JTextField();
accessConnectLabel = new javax.swing.JLabel();
accessConnectField = new javax.swing.JTextField();
commandConnectLabel = new javax.swing.JLabel();
commandConnectField = new javax.swing.JTextField();
orignalPanel = new javax.swing.JPanel();
textLabel = new javax.swing.JLabel();
keepLabel = new javax.swing.JLabel();
keepCheck = new javax.swing.JCheckBox();
noteLabel = new javax.swing.JLabel();
originalScroll = new javax.swing.JScrollPane();
originalArea = new javax.swing.JTextArea();
noteScroll = new javax.swing.JScrollPane();
noteArea = new javax.swing.JTextArea();
accessOriginalLabel = new javax.swing.JLabel();
acessOriginalField = new javax.swing.JTextField();
commandOriginalLabel = new javax.swing.JLabel();
commandOriginalField = new javax.swing.JTextField();
translatedPanel = new javax.swing.JPanel();
translatedLabel = new javax.swing.JLabel();
accessTranslatedLabel = new javax.swing.JLabel();
accessTranslatedField = new javax.swing.JTextField();
commandTranslatedLabel = new javax.swing.JLabel();
commandTranslatedField = new javax.swing.JTextField();
statusLabel = new javax.swing.JLabel();
String[] modes = { "Not Seen", "Skipped", "Error" ,"Change" , "Accepted" , "Perfect" ,"Other"};
statusCombo = statusCombo = new JComboBox(modes);
commentLabel = new javax.swing.JLabel();
commentField = new javax.swing.JTextField();
translatedScroll = new javax.swing.JScrollPane();
translatedArea = new javax.swing.JTextArea();
buttonPanel = new javax.swing.JPanel();
okayButton = new javax.swing.JButton();
cancelButton = new javax.swing.JButton();
getContentPane().setLayout(new java.awt.GridBagLayout());
java.awt.GridBagConstraints gridBagConstraints1;
setDefaultCloseOperation(javax.swing.WindowConstants.DO_NOTHING_ON_CLOSE);

basicPanel.setLayout(new java.awt.GridBagLayout());
java.awt.GridBagConstraints gridBagConstraints2;
basicPanel.setBorder(new javax.swing.border.TitledBorder(
  new javax.swing.border.EtchedBorder(), "Basic information", 4, 2, 
  new java.awt.Font ("Dialog", 0, 10)));

InstallLabel.setText("Install");
  gridBagConstraints2 = new java.awt.GridBagConstraints();
  gridBagConstraints2.insets = new java.awt.Insets(3, 0, 0, 0);
  gridBagConstraints2.anchor = java.awt.GridBagConstraints.WEST;
  basicPanel.add(InstallLabel, gridBagConstraints2);
  
  
installField.setEditable(false);
  installField.setColumns(15);
  installField.setText("The Install");
  gridBagConstraints2 = new java.awt.GridBagConstraints();
  gridBagConstraints2.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints2.anchor = java.awt.GridBagConstraints.WEST;
  gridBagConstraints2.weightx = 1.0;
  basicPanel.add(installField, gridBagConstraints2);
  
  
componentLabel.setText("Component");
  gridBagConstraints2 = new java.awt.GridBagConstraints();
  gridBagConstraints2.gridx = 2;
  gridBagConstraints2.gridy = 0;
  gridBagConstraints2.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints2.anchor = java.awt.GridBagConstraints.WEST;
  basicPanel.add(componentLabel, gridBagConstraints2);
  
  
componentField.setEditable(false);
  componentField.setColumns(15);
  componentField.setText("The Component");
  gridBagConstraints2 = new java.awt.GridBagConstraints();
  gridBagConstraints2.gridx = 3;
  gridBagConstraints2.gridy = 0;
  gridBagConstraints2.gridwidth = 0;
  gridBagConstraints2.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints2.anchor = java.awt.GridBagConstraints.WEST;
  gridBagConstraints2.weightx = 1.0;
  basicPanel.add(componentField, gridBagConstraints2);
  
  
fileLabel.setText("File");
  gridBagConstraints2 = new java.awt.GridBagConstraints();
  gridBagConstraints2.gridx = 2;
  gridBagConstraints2.gridy = 1;
  gridBagConstraints2.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints2.anchor = java.awt.GridBagConstraints.WEST;
  basicPanel.add(fileLabel, gridBagConstraints2);
  
  
fileField.setEditable(false);
  fileField.setColumns(15);
  fileField.setText("The File");
  gridBagConstraints2 = new java.awt.GridBagConstraints();
  gridBagConstraints2.gridx = 3;
  gridBagConstraints2.gridy = 1;
  gridBagConstraints2.gridwidth = 0;
  gridBagConstraints2.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints2.anchor = java.awt.GridBagConstraints.WEST;
  gridBagConstraints2.weightx = 1.0;
  basicPanel.add(fileField, gridBagConstraints2);
  
  
keyLabel.setText("Key");
  gridBagConstraints2 = new java.awt.GridBagConstraints();
  gridBagConstraints2.gridx = 0;
  gridBagConstraints2.gridy = 2;
  gridBagConstraints2.insets = new java.awt.Insets(3, 0, 0, 0);
  gridBagConstraints2.anchor = java.awt.GridBagConstraints.WEST;
  basicPanel.add(keyLabel, gridBagConstraints2);
  
  
keyField.setEditable(false);
  keyField.setColumns(15);
  keyField.setText("The key");
  gridBagConstraints2 = new java.awt.GridBagConstraints();
  gridBagConstraints2.gridx = 1;
  gridBagConstraints2.gridy = 2;
  gridBagConstraints2.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints2.anchor = java.awt.GridBagConstraints.WEST;
  gridBagConstraints2.weightx = 1.0;
  basicPanel.add(keyField, gridBagConstraints2);
  
  
subcomponentLabel.setText("Subcomponent");
  gridBagConstraints2 = new java.awt.GridBagConstraints();
  gridBagConstraints2.gridx = 0;
  gridBagConstraints2.gridy = 1;
  gridBagConstraints2.insets = new java.awt.Insets(3, 0, 0, 0);
  gridBagConstraints2.anchor = java.awt.GridBagConstraints.WEST;
  basicPanel.add(subcomponentLabel, gridBagConstraints2);
  
  
subcomponentField.setEditable(false);
  subcomponentField.setColumns(15);
  subcomponentField.setText("the subcomponent");
  gridBagConstraints2 = new java.awt.GridBagConstraints();
  gridBagConstraints2.gridx = 1;
  gridBagConstraints2.gridy = 1;
  gridBagConstraints2.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints2.anchor = java.awt.GridBagConstraints.WEST;
  gridBagConstraints2.weightx = 1.0;
  basicPanel.add(subcomponentField, gridBagConstraints2);
  
  
accessConnectLabel.setText("Accesskey");
  gridBagConstraints2 = new java.awt.GridBagConstraints();
  gridBagConstraints2.gridx = 2;
  gridBagConstraints2.gridy = 3;
  gridBagConstraints2.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints2.anchor = java.awt.GridBagConstraints.WEST;
  basicPanel.add(accessConnectLabel, gridBagConstraints2);
  
  
accessConnectField.setEditable(false);
  accessConnectField.setColumns(15);
  accessConnectField.setText("somenameAccesskey");
  gridBagConstraints2 = new java.awt.GridBagConstraints();
  gridBagConstraints2.gridx = 3;
  gridBagConstraints2.gridy = 3;
  gridBagConstraints2.gridwidth = 0;
  gridBagConstraints2.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints2.anchor = java.awt.GridBagConstraints.WEST;
  basicPanel.add(accessConnectField, gridBagConstraints2);
  
  
commandConnectLabel.setText("Commandkey");
  gridBagConstraints2 = new java.awt.GridBagConstraints();
  gridBagConstraints2.gridx = 0;
  gridBagConstraints2.gridy = 3;
  gridBagConstraints2.insets = new java.awt.Insets(3, 0, 0, 0);
  gridBagConstraints2.anchor = java.awt.GridBagConstraints.WEST;
  basicPanel.add(commandConnectLabel, gridBagConstraints2);
  
  
commandConnectField.setEditable(false);
  commandConnectField.setColumns(15);
  commandConnectField.setText("yada yadaComandKey");
  gridBagConstraints2 = new java.awt.GridBagConstraints();
  gridBagConstraints2.gridx = 1;
  gridBagConstraints2.gridy = 3;
  gridBagConstraints2.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints2.anchor = java.awt.GridBagConstraints.WEST;
  basicPanel.add(commandConnectField, gridBagConstraints2);
  
  
gridBagConstraints1 = new java.awt.GridBagConstraints();
gridBagConstraints1.gridx = 0;
gridBagConstraints1.gridy = 0;
gridBagConstraints1.gridwidth = 0;
gridBagConstraints1.fill = java.awt.GridBagConstraints.HORIZONTAL;
gridBagConstraints1.anchor = java.awt.GridBagConstraints.NORTHWEST;
getContentPane().add(basicPanel, gridBagConstraints1);


orignalPanel.setLayout(new java.awt.GridBagLayout());
java.awt.GridBagConstraints gridBagConstraints3;
orignalPanel.setBorder(new javax.swing.border.TitledBorder(
  new javax.swing.border.EtchedBorder(), "Original", 4, 2, 
  new java.awt.Font ("Dialog", 0, 10)));

textLabel.setText("Text");
  gridBagConstraints3 = new java.awt.GridBagConstraints();
  gridBagConstraints3.insets = new java.awt.Insets(5, 0, 0, 0);
  gridBagConstraints3.anchor = java.awt.GridBagConstraints.NORTHEAST;
  orignalPanel.add(textLabel, gridBagConstraints3);
  
  
keepLabel.setText("Filter");
  gridBagConstraints3 = new java.awt.GridBagConstraints();
  gridBagConstraints3.gridx = 0;
  gridBagConstraints3.gridy = 2;
  gridBagConstraints3.insets = new java.awt.Insets(3, 0, 0, 0);
  gridBagConstraints3.anchor = java.awt.GridBagConstraints.EAST;
  orignalPanel.add(keepLabel, gridBagConstraints3);
  
  
keepCheck.setText("Keep original text");
  gridBagConstraints3 = new java.awt.GridBagConstraints();
  gridBagConstraints3.gridx = 1;
  gridBagConstraints3.gridy = 2;
  gridBagConstraints3.fill = java.awt.GridBagConstraints.HORIZONTAL;
  gridBagConstraints3.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints3.anchor = java.awt.GridBagConstraints.WEST;
  orignalPanel.add(keepCheck, gridBagConstraints3);
  
  
noteLabel.setText("Loc. note");
  gridBagConstraints3 = new java.awt.GridBagConstraints();
  gridBagConstraints3.gridx = 0;
  gridBagConstraints3.gridy = 1;
  gridBagConstraints3.insets = new java.awt.Insets(5, 0, 0, 0);
  gridBagConstraints3.anchor = java.awt.GridBagConstraints.NORTHEAST;
  orignalPanel.add(noteLabel, gridBagConstraints3);
  
  

  originalArea.setEditable(false);
    originalArea.setColumns(25);
    originalArea.setRows(3);
    originalArea.setText("This is bogus text\nShould newer show\ncheers\nHenrik\n");
    originalScroll.setViewportView(originalArea);
    
    gridBagConstraints3 = new java.awt.GridBagConstraints();
  gridBagConstraints3.gridx = 1;
  gridBagConstraints3.gridy = 0;
  gridBagConstraints3.fill = java.awt.GridBagConstraints.BOTH;
  gridBagConstraints3.insets = new java.awt.Insets(3, 3, 0, 0);
  orignalPanel.add(originalScroll, gridBagConstraints3);
  
  

  noteArea.setEditable(false);
    noteArea.setColumns(25);
    noteArea.setRows(3);
    noteArea.setText("this is bogus text\nshould newr show\ncheers \nhenrik");
    noteScroll.setViewportView(noteArea);
    
    gridBagConstraints3 = new java.awt.GridBagConstraints();
  gridBagConstraints3.gridx = 1;
  gridBagConstraints3.gridy = 1;
  gridBagConstraints3.fill = java.awt.GridBagConstraints.BOTH;
  gridBagConstraints3.insets = new java.awt.Insets(3, 3, 0, 0);
  orignalPanel.add(noteScroll, gridBagConstraints3);
  
  
accessOriginalLabel.setText("Accesskey");
  gridBagConstraints3 = new java.awt.GridBagConstraints();
  gridBagConstraints3.gridx = 0;
  gridBagConstraints3.gridy = 3;
  gridBagConstraints3.insets = new java.awt.Insets(3, 0, 0, 0);
  gridBagConstraints3.anchor = java.awt.GridBagConstraints.EAST;
  orignalPanel.add(accessOriginalLabel, gridBagConstraints3);
  
  
acessOriginalField.setEditable(false);
  acessOriginalField.setColumns(10);
  acessOriginalField.setText("jTextField1");
  gridBagConstraints3 = new java.awt.GridBagConstraints();
  gridBagConstraints3.gridx = 1;
  gridBagConstraints3.gridy = 3;
  gridBagConstraints3.fill = java.awt.GridBagConstraints.HORIZONTAL;
  gridBagConstraints3.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints3.anchor = java.awt.GridBagConstraints.NORTHWEST;
  orignalPanel.add(acessOriginalField, gridBagConstraints3);
  
  
commandOriginalLabel.setText("commandkey");
  gridBagConstraints3 = new java.awt.GridBagConstraints();
  gridBagConstraints3.gridx = 0;
  gridBagConstraints3.gridy = 4;
  gridBagConstraints3.insets = new java.awt.Insets(3, 0, 0, 0);
  gridBagConstraints3.anchor = java.awt.GridBagConstraints.EAST;
  orignalPanel.add(commandOriginalLabel, gridBagConstraints3);
  
  
commandOriginalField.setEditable(false);
  commandOriginalField.setColumns(10);
  commandOriginalField.setText("jTextField2");
  gridBagConstraints3 = new java.awt.GridBagConstraints();
  gridBagConstraints3.gridx = 1;
  gridBagConstraints3.gridy = 4;
  gridBagConstraints3.fill = java.awt.GridBagConstraints.HORIZONTAL;
  gridBagConstraints3.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints3.anchor = java.awt.GridBagConstraints.WEST;
  orignalPanel.add(commandOriginalField, gridBagConstraints3);
  
  
gridBagConstraints1 = new java.awt.GridBagConstraints();
gridBagConstraints1.gridx = 0;
gridBagConstraints1.gridy = 1;
gridBagConstraints1.anchor = java.awt.GridBagConstraints.NORTHWEST;
getContentPane().add(orignalPanel, gridBagConstraints1);


translatedPanel.setLayout(new java.awt.GridBagLayout());
java.awt.GridBagConstraints gridBagConstraints4;
translatedPanel.setBorder(new javax.swing.border.TitledBorder(
  new javax.swing.border.EtchedBorder(), "Translated", 4, 2, 
  new java.awt.Font ("Dialog", 0, 10)));

translatedLabel.setText("Text");
  gridBagConstraints4 = new java.awt.GridBagConstraints();
  gridBagConstraints4.gridx = 0;
  gridBagConstraints4.gridy = 0;
  gridBagConstraints4.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints4.anchor = java.awt.GridBagConstraints.NORTHEAST;
  translatedPanel.add(translatedLabel, gridBagConstraints4);
  
  
accessTranslatedLabel.setText("Accesskey");
  gridBagConstraints4 = new java.awt.GridBagConstraints();
  gridBagConstraints4.gridx = 0;
  gridBagConstraints4.gridy = 1;
  gridBagConstraints4.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints4.anchor = java.awt.GridBagConstraints.WEST;
  translatedPanel.add(accessTranslatedLabel, gridBagConstraints4);
  
  
accessTranslatedField.setColumns(10);
  accessTranslatedField.setText("jTextField1");
  gridBagConstraints4 = new java.awt.GridBagConstraints();
  gridBagConstraints4.gridx = 1;
  gridBagConstraints4.gridy = 1;
  gridBagConstraints4.gridwidth = 0;
  gridBagConstraints4.fill = java.awt.GridBagConstraints.HORIZONTAL;
  gridBagConstraints4.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints4.anchor = java.awt.GridBagConstraints.WEST;
  translatedPanel.add(accessTranslatedField, gridBagConstraints4);
  
  
commandTranslatedLabel.setText("Commandkey");
  gridBagConstraints4 = new java.awt.GridBagConstraints();
  gridBagConstraints4.gridx = 0;
  gridBagConstraints4.gridy = 2;
  gridBagConstraints4.anchor = java.awt.GridBagConstraints.WEST;
  translatedPanel.add(commandTranslatedLabel, gridBagConstraints4);
  
  
commandTranslatedField.setColumns(10);
  commandTranslatedField.setText("jTextField2");
  gridBagConstraints4 = new java.awt.GridBagConstraints();
  gridBagConstraints4.gridx = 1;
  gridBagConstraints4.gridy = 2;
  gridBagConstraints4.gridwidth = 0;
  gridBagConstraints4.fill = java.awt.GridBagConstraints.HORIZONTAL;
  gridBagConstraints4.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints4.anchor = java.awt.GridBagConstraints.WEST;
  translatedPanel.add(commandTranslatedField, gridBagConstraints4);
  
  
statusLabel.setText("Status");
  gridBagConstraints4 = new java.awt.GridBagConstraints();
  gridBagConstraints4.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints4.anchor = java.awt.GridBagConstraints.WEST;
  translatedPanel.add(statusLabel, gridBagConstraints4);
  
  
gridBagConstraints4 = new java.awt.GridBagConstraints();
  gridBagConstraints4.gridwidth = 0;
  gridBagConstraints4.fill = java.awt.GridBagConstraints.HORIZONTAL;
  gridBagConstraints4.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints4.anchor = java.awt.GridBagConstraints.WEST;
  translatedPanel.add(statusCombo, gridBagConstraints4);
  
  
commentLabel.setText("Comment");
  gridBagConstraints4 = new java.awt.GridBagConstraints();
  gridBagConstraints4.gridx = 0;
  gridBagConstraints4.gridy = 4;
  gridBagConstraints4.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints4.anchor = java.awt.GridBagConstraints.WEST;
  translatedPanel.add(commentLabel, gridBagConstraints4);
  
  
commentField.setColumns(15);
  commentField.setText("some comment");
  gridBagConstraints4 = new java.awt.GridBagConstraints();
  gridBagConstraints4.gridx = 1;
  gridBagConstraints4.gridy = 4;
  gridBagConstraints4.gridwidth = 0;
  gridBagConstraints4.fill = java.awt.GridBagConstraints.HORIZONTAL;
  gridBagConstraints4.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints4.anchor = java.awt.GridBagConstraints.WEST;
  translatedPanel.add(commentField, gridBagConstraints4);
  
  

  translatedArea.setColumns(25);
    translatedArea.setRows(3);
    translatedArea.setText("this is bogus text\nshould newer show\ncheers\nhenrik\n");
    translatedScroll.setViewportView(translatedArea);
    
    gridBagConstraints4 = new java.awt.GridBagConstraints();
  gridBagConstraints4.gridx = 1;
  gridBagConstraints4.gridy = 0;
  gridBagConstraints4.gridwidth = 0;
  gridBagConstraints4.fill = java.awt.GridBagConstraints.BOTH;
  gridBagConstraints4.insets = new java.awt.Insets(3, 3, 0, 0);
  translatedPanel.add(translatedScroll, gridBagConstraints4);
  
  
gridBagConstraints1 = new java.awt.GridBagConstraints();
gridBagConstraints1.gridx = 1;
gridBagConstraints1.gridy = 1;
gridBagConstraints1.gridwidth = 0;
gridBagConstraints1.fill = java.awt.GridBagConstraints.BOTH;
gridBagConstraints1.anchor = java.awt.GridBagConstraints.NORTHWEST;
getContentPane().add(translatedPanel, gridBagConstraints1);


buttonPanel.setLayout(new java.awt.GridBagLayout());
java.awt.GridBagConstraints gridBagConstraints5;

okayButton.setMnemonic('O');
  okayButton.setText("Okay");
  okayButton.addActionListener(new java.awt.event.ActionListener() {
  public void actionPerformed(java.awt.event.ActionEvent evt) {
  okayButtonPressed(evt);
  }
  }
  );
  gridBagConstraints5 = new java.awt.GridBagConstraints();
  gridBagConstraints5.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints5.anchor = java.awt.GridBagConstraints.EAST;
  buttonPanel.add(okayButton, gridBagConstraints5);
  
  
cancelButton.setMnemonic('C');
  cancelButton.setText("Cancel");
  cancelButton.addActionListener(new java.awt.event.ActionListener() {
  public void actionPerformed(java.awt.event.ActionEvent evt) {
  cancelButtonPressed(evt);
  }
  }
  );
  gridBagConstraints5 = new java.awt.GridBagConstraints();
  gridBagConstraints5.gridx = 1;
  gridBagConstraints5.gridy = 0;
  gridBagConstraints5.gridwidth = 0;
  gridBagConstraints5.insets = new java.awt.Insets(3, 3, 0, 0);
  gridBagConstraints5.anchor = java.awt.GridBagConstraints.EAST;
  buttonPanel.add(cancelButton, gridBagConstraints5);
  
  
gridBagConstraints1 = new java.awt.GridBagConstraints();
gridBagConstraints1.gridx = 0;
gridBagConstraints1.gridy = 2;
gridBagConstraints1.gridwidth = 0;
gridBagConstraints1.fill = java.awt.GridBagConstraints.HORIZONTAL;
gridBagConstraints1.anchor = java.awt.GridBagConstraints.NORTHWEST;
getContentPane().add(buttonPanel, gridBagConstraints1);

}//GEN-END:initComponents

  private void cancelButtonPressed (java.awt.event.ActionEvent evt) {//GEN-FIRST:event_cancelButtonPressed
    okay=false;
    setVisible(false);
  }//GEN-LAST:event_cancelButtonPressed

  private void okayButtonPressed (java.awt.event.ActionEvent evt) {//GEN-FIRST:event_okayButtonPressed
    okay=true;
    setVisible(false);
    
  }//GEN-LAST:event_okayButtonPressed

public void visDialog(Phrase phrase,String localeName)
{
    MozInstall install;
    MozComponent component;
    MozComponent subcomponent;
    MozFile file;
    Translation translation;
    Phrase accessPhrase,commandPhrase;
    Translation accessTranslation,commandTranslation;

    
    // fetch the needed objects
    file = (MozFile) phrase.getParent();
    subcomponent = (MozComponent) file.getParent();
    component =  (MozComponent) subcomponent.getParent();
    install = (MozInstall) component.getParent();
    translation = (Translation) phrase.getChildByName(localeName);
    accessPhrase = phrase.getAccessConnection();
    commandPhrase = phrase.getCommandConnection();
    accessTranslation=null;
    commandTranslation=null;
    
    // set up the connections
    if (accessPhrase!=null)
    {
        accessTranslation = (Translation) accessPhrase.getChildByName(localeName);

        accessConnectField.setText(accessPhrase.getName());
        
        if (accessTranslation!=null)
        {
            accessTranslatedField.setText(accessTranslation.getText());
        }
        else
        {
            accessTranslatedField.setText("");
        }
        acessOriginalField.setText(accessPhrase.getText());
    }
    else
    {
        accessConnectField.setText("Nothing found");
        accessTranslatedField.setText("");
        accessTranslatedLabel.setEnabled(false);
        accessTranslatedField.setEnabled(false);
        acessOriginalField.setEnabled(false);
        acessOriginalField.setText("");
    }
    
    if (commandPhrase!=null)
    {
        commandTranslation = (Translation) commandPhrase.getChildByName(localeName);
        
        commandConnectField.setText(commandPhrase.getName());
        
        if (commandTranslation!=null)
        {
            commandTranslatedField.setText(commandTranslation.getText());
        }
        else
        {
            commandTranslatedField.setText("");
        }
        commandOriginalField.setText(commandPhrase.getText());
    }
    else
    {
        commandConnectField.setText("Nothing found");
        commandTranslatedField.setText("");
        commandTranslatedLabel.setEnabled(false);
        commandTranslatedField.setEnabled(false);
        commandOriginalField.setText("");
        commandOriginalField.setEnabled(false);
    }

    
    
    // basic information
    installField.setText(install.getName());
    componentField.setText(component.getName());
    subcomponentField.setText(subcomponent.getName());
    fileField.setText(file.getName());
    keyField.setText(phrase.getName());
    
    //original information
    originalArea.setText(phrase.getText());
    noteArea.setText(phrase.getNote());
    keepCheck.setSelected(phrase.getKeepOriginal());
    originalArea.setCaretPosition(0);
    noteArea.setCaretPosition(0);
    
    // translated information
    if (translation!=null)
    {
        translatedArea.setText(translation.getText());
        statusCombo.setSelectedIndex(translation.getStatus());
        commentField.setText(translation.getComment());

    }
    else
    {
        translatedArea.setText("");
        statusCombo.setSelectedIndex(0);
        commentField.setText("");
    }
    okay=false;

    translatedArea.setCaretPosition(0);
    
    setVisible(true);
    
    
    // set the new information if neededd
    if (okay)
    {
        if (translation!=null)
        {
            translation.setText(translatedArea.getText());
            translation.setStatus(statusCombo.getSelectedIndex());
            translation.setComment(commentField.getText());
        }
        else
        {
            translation = new Translation(localeName,phrase,translatedArea.getText(),statusCombo.getSelectedIndex(),commentField.getText());
            phrase.addChild(translation);
        }
        // set the access connection if needed
        
        if (accessTranslatedField.isEnabled())
        {
            if (accessTranslation!=null)
            {
                accessTranslation.setText(accessTranslatedField.getText());
            }
            else
            {
                accessTranslation = new Translation(localeName,accessPhrase,accessTranslatedField.getText());
                accessPhrase.addChild(accessTranslation);
            }
        }
        
        //set the command connection if needed
        if (commandTranslatedField.isEnabled())
        {
            if (commandTranslation!=null)
            {
                commandTranslation.setText(commandTranslatedField.getText());
            }
            else
            {
                commandTranslation = new Translation(localeName,commandPhrase,commandTranslatedField.getText());
                commandPhrase.addChild(commandTranslation);
            }
        }  
    }
    dispose();
}


// Variables declaration - do not modify//GEN-BEGIN:variables
private javax.swing.JPanel basicPanel;
private javax.swing.JLabel InstallLabel;
private javax.swing.JTextField installField;
private javax.swing.JLabel componentLabel;
private javax.swing.JTextField componentField;
private javax.swing.JLabel fileLabel;
private javax.swing.JTextField fileField;
private javax.swing.JLabel keyLabel;
private javax.swing.JTextField keyField;
private javax.swing.JLabel subcomponentLabel;
private javax.swing.JTextField subcomponentField;
private javax.swing.JLabel accessConnectLabel;
private javax.swing.JTextField accessConnectField;
private javax.swing.JLabel commandConnectLabel;
private javax.swing.JTextField commandConnectField;
private javax.swing.JPanel orignalPanel;
private javax.swing.JLabel textLabel;
private javax.swing.JLabel keepLabel;
private javax.swing.JCheckBox keepCheck;
private javax.swing.JLabel noteLabel;
private javax.swing.JScrollPane originalScroll;
private javax.swing.JTextArea originalArea;
private javax.swing.JScrollPane noteScroll;
private javax.swing.JTextArea noteArea;
private javax.swing.JLabel accessOriginalLabel;
private javax.swing.JTextField acessOriginalField;
private javax.swing.JLabel commandOriginalLabel;
private javax.swing.JTextField commandOriginalField;
private javax.swing.JPanel translatedPanel;
private javax.swing.JLabel translatedLabel;
private javax.swing.JLabel accessTranslatedLabel;
private javax.swing.JTextField accessTranslatedField;
private javax.swing.JLabel commandTranslatedLabel;
private javax.swing.JTextField commandTranslatedField;
private javax.swing.JLabel statusLabel;
private javax.swing.JComboBox statusCombo;
private javax.swing.JLabel commentLabel;
private javax.swing.JTextField commentField;
private javax.swing.JScrollPane translatedScroll;
private javax.swing.JTextArea translatedArea;
private javax.swing.JPanel buttonPanel;
private javax.swing.JButton okayButton;
private javax.swing.JButton cancelButton;
// End of variables declaration//GEN-END:variables
private boolean okay;

}