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
package org.mozilla.translator.actions;

import java.awt.event.*;
import javax.swing.*;

import org.mozilla.translator.gui.dialog.*;
import org.mozilla.translator.datamodel.*;
import org.mozilla.translator.kernel.*;
import org.mozilla.translator.runners.*;
/** 
 *
 * @author  Henrik Lynggaard Hansen
 * @version 
 */
public class ExportPartialGlossaryAction extends AbstractAction {

  /** Creates new InstallManagerAction */
  public ExportPartialGlossaryAction() 
  {
    super("Export Partial glossary",null);    
    
  }
  
  public void actionPerformed(ActionEvent evt)
  {
    boolean result;
    MozInstall install;
    Object[] subcomponents;
    String fileName;
    ExportPartialGlossaryDialog epg;
    ExportPartialGlossaryRunner epr;
    
    epg = new ExportPartialGlossaryDialog();
    
    result = epg.visDialog();
    
    if (result)
    {
        subcomponents = epg.getSubcomponents();
        fileName = epg.getFileName();
        install = epg.getInstall();
        epr = new ExportPartialGlossaryRunner(install,subcomponents,fileName);
        epr.start();
    }
  }
}