// Copyright 2008 Isis Innovation Limited
#include "OpenGL.h"
#include "GLWindowMenu.h"
#include <gvars3/instances.h>
#include <gvars3/GStringUtil.h>
#include <sstream>

using namespace GVars3;
using namespace CVD;
using namespace std;

GLWindowMenu::GLWindowMenu(string sName, string sTitle)
{
  msName = sName;
  msTitle = sTitle;
  
  GUI.RegisterCommand(msName+".AddMenuButton", GUICommandCallBack, this);
  GUI.RegisterCommand(msName+".AddMenuToggle", GUICommandCallBack, this);
  GUI.RegisterCommand(msName+".AddMenuSlider", GUICommandCallBack, this);
  GUI.RegisterCommand(msName+".AddMenuMonitor", GUICommandCallBack, this);
  GUI.RegisterCommand(msName+".ShowMenu", GUICommandCallBack, this);
  GV2.Register(mgvnMenuItemWidth, msName+".MenuItemWidth", 90, HIDDEN | SILENT);
  GV2.Register(mgvnMenuTextOffset, msName+".MenuTextOffset", 20, HIDDEN | SILENT);
  GV2.Register(mgvnEnabled, msName+".Enabled", 1, HIDDEN | SILENT);
  
  mmSubMenus.clear();
  msCurrentSubMenu="";
}

GLWindowMenu::~GLWindowMenu()
{
  GUI.UnRegisterCommand(msName+".AddMenuButton");
  GUI.UnRegisterCommand(msName+".AddMenuToggle");
  GUI.UnRegisterCommand(msName+".AddMenuSlider");
  GUI.UnRegisterCommand(msName+".AddMenuMonitor");
  GUI.UnRegisterCommand(msName+".ShowMenu");
};


void GLWindowMenu::GUICommandCallBack(void *ptr, string sCommand, string sParams)
{
  ((GLWindowMenu*) ptr)->GUICommandHandler(sCommand, sParams);
}

void GLWindowMenu::GUICommandHandler(string sCommand, string sParams)
{
  vector<string> vs = ChopAndUnquoteString(sParams);  

  if(sCommand==msName+".AddMenuButton")
    {
      if(vs.size()<3)
	{
	  cout<< "? GLWindowMenu.AddMenuButton: Need 3/4 params: Target Menu, Name, Command , NextMenu=\"\"" << endl;
	  return;
	};
      MenuItem m;
      m.type = Button;
      m.sName = vs[1];
      m.sParam = UncommentString(vs[2]);
      m.sNextMenu = (vs.size()>3)?(vs[3]):("");
      mmSubMenus[vs[0]].mvItems.push_back(m);
      return;
    }

  if(sCommand==msName+".AddMenuToggle")
    {
      if(vs.size()<3)
	{
	  cout<< "? GLWindowMenu.AddMenuToggle: Need 3/4 params: Target Menu, Name, gvar_int name , NextMenu=\"\"" << endl;
	  return;
	};
      MenuItem m;
      m.type = Toggle;
      m.sName = vs[1];
      GV2.Register(m.gvnIntValue, vs[2]);
      m.sNextMenu = (vs.size()>3)?(vs[3]):("");
      mmSubMenus[vs[0]].mvItems.push_back(m);
      return;
    }

  if(sCommand==msName+".AddMenuMonitor")
    {
      if(vs.size()<3)
	{
	  cout<< "? GLWindowMenu.AddMenuMonitor: Need 3/4 params: Target Menu, Name, gvar name , NextMenu=\"\"" << endl;
	  return;
	};
      MenuItem m;
      m.type = Monitor;
      m.sName = vs[1];
      m.sParam = vs[2];
      m.sNextMenu = (vs.size()>3)?(vs[3]):("");
      mmSubMenus[vs[0]].mvItems.push_back(m);
      return;
    }

  if(sCommand==msName+".AddMenuSlider")
    {
      if(vs.size()<5)
	{
	  cout<< "? GLWindowMenu.AddMenuSlider: Need 5/6 params: Target Menu, Name, gvar_int name, min, max, NextMenu=\"\"" << endl;
	  return;
	};
      MenuItem m;
      m.type = Slider;
      m.sName = vs[1];
      GV2.Register(m.gvnIntValue, vs[2]);
      int *a;
      a = ParseAndAllocate<int>(vs[3]);
      if(a)
	{
	  m.min = *a;
	  delete a;
	}
      a = ParseAndAllocate<int>(vs[4]);
      if(a)
	{
	  m.max = *a;
	  delete a;
	}
      m.sNextMenu = (vs.size()>5)?(vs[5]):("");
      mmSubMenus[vs[0]].mvItems.push_back(m);
      return;
    }
  
  if(sCommand==msName+".ShowMenu")
    {
      if(vs.size()==0)
	msCurrentSubMenu = "";
      else
	msCurrentSubMenu = vs[0];
    };
  
};

void GLWindowMenu::LineBox(int l, int r, int t, int b)
{
  glBegin(GL_LINE_STRIP);
  glVertex2i(l,t);
  glVertex2i(l,b);
  glVertex2i(r,b);
  glVertex2i(r,t);
  glVertex2i(l,t);
  glEnd();
}

void GLWindowMenu::FillBox(int l, int r, int t, int b)
{
  glBegin(GL_QUADS);
  glVertex2i(l,t);
  glVertex2i(l,b);
  glVertex2i(r,b);
  glVertex2i(r,t);
  glEnd();
}

void GLWindowMenu::Render(int nTop, int nHeight, int nWidth, GLWindow2 &glw)
{
  if(!*mgvnEnabled)
    return;

  mnWidth = nWidth;
  mnMenuTop = nTop;
  mnMenuHeight = nHeight;

  double dAlpha = 0.8;
  if(msCurrentSubMenu=="")  // No Menu selected  - draw little arrow.
    {
      glColor4d(0,0.5,0,0.5);
      FillBox(mnWidth - 30, mnWidth - 1, mnMenuTop, mnMenuTop + mnMenuHeight);
      glColor4d(0,1,0,0.5);
      LineBox(mnWidth - 30, mnWidth - 1, mnMenuTop, mnMenuTop + mnMenuHeight);
      mnLeftMostCoord = mnWidth - 30;
      return;
    };
  
  SubMenu &m = mmSubMenus[msCurrentSubMenu];
  
  mnLeftMostCoord = mnWidth - (1 + m.mvItems.size()) * *mgvnMenuItemWidth;
  int nBase = mnLeftMostCoord;
  for(vector<MenuItem>::reverse_iterator i = m.mvItems.rbegin(); i!= m.mvItems.rend(); i++)
    {
      switch(i->type)
	{
	case Button:
	  glColor4d(0,0.5,0,dAlpha);
	  FillBox(nBase, nBase + *mgvnMenuItemWidth +1, mnMenuTop, mnMenuTop + mnMenuHeight);
	  glColor4d(0,1,0,dAlpha);
	  LineBox(nBase, nBase + *mgvnMenuItemWidth +1, mnMenuTop, mnMenuTop + mnMenuHeight);
	  glw.PrintString(ImageRef( nBase + 3, mnMenuTop + *mgvnMenuTextOffset),
			  i->sName);
	  break;
	  
	case Toggle:
	  if(*(i->gvnIntValue))
	    glColor4d(0,0.5,0.5,dAlpha);
	  else
	    glColor4d(0.5,0,0,dAlpha);
	  FillBox(nBase, nBase + *mgvnMenuItemWidth +1, mnMenuTop, mnMenuTop + mnMenuHeight);
	  if(*(i->gvnIntValue))
	    glColor4d(0,1,0.5,dAlpha);
	  else
	    glColor4d(1,0,0,dAlpha);
	  LineBox(nBase, nBase + *mgvnMenuItemWidth +1, mnMenuTop, mnMenuTop + mnMenuHeight);
	  glw.PrintString(ImageRef( nBase + 3, mnMenuTop + *mgvnMenuTextOffset),
			  i->sName + " " + ((*(i->gvnIntValue))?("On"):("Off")));
	  break;

	case Monitor:
	  glColor4d(0,0.5,0.5,dAlpha);
	  FillBox(nBase, nBase + *mgvnMenuItemWidth +1, mnMenuTop, mnMenuTop + mnMenuHeight);
	  glColor4d(0,1,1,dAlpha);
	  LineBox(nBase, nBase + *mgvnMenuItemWidth +1, mnMenuTop, mnMenuTop + mnMenuHeight);
	  glw.PrintString(ImageRef( nBase + 3, mnMenuTop + *mgvnMenuTextOffset),
			  i->sName + " " + GV2.StringValue(i->sParam, true));
	  break;
	  
	case Slider:
	  {
	    glColor4d(0.0,0.0,0.5,dAlpha);
	    FillBox(nBase, nBase + *mgvnMenuItemWidth +1, mnMenuTop, mnMenuTop + mnMenuHeight);
	    glColor4d(0.5,0.0,0.5,dAlpha);
	    double dFrac = (double) (*(i->gvnIntValue) - i->min) / (i->max - i->min);
	    if(dFrac<0.0)
	      dFrac = 0.0;
	    if(dFrac>1.0)
	      dFrac = 1.0;
	    FillBox(nBase, (int) (nBase + dFrac * (*mgvnMenuItemWidth +1)), mnMenuTop, mnMenuTop + mnMenuHeight);
	    glColor4d(0,1,1,dAlpha);
	    LineBox(nBase, nBase + *mgvnMenuItemWidth +1, mnMenuTop, mnMenuTop + mnMenuHeight);
	    ostringstream ost;
	    ost << i->sName << " " << *(i->gvnIntValue);
	    glw.PrintString(ImageRef( nBase + 3, mnMenuTop + *mgvnMenuTextOffset),
			    ost.str());
	  }
	  break;
	}
      nBase += *mgvnMenuItemWidth;
    };
  glColor4d(0.5, 0.5, 0,dAlpha);
  FillBox(mnWidth - *mgvnMenuItemWidth, mnWidth-1, mnMenuTop, mnMenuTop + mnMenuHeight);
  glColor4d(1,1,0,dAlpha);
  LineBox(mnWidth - *mgvnMenuItemWidth, mnWidth-1, mnMenuTop, mnMenuTop + mnMenuHeight);
  ImageRef ir( mnWidth - *mgvnMenuItemWidth + 5, mnMenuTop + *mgvnMenuTextOffset);
  if(msCurrentSubMenu == "Root")
    glw.PrintString(ir, msTitle+":");
  else
    glw.PrintString(ir, msCurrentSubMenu+":");
};


bool GLWindowMenu::HandleClick(int nMouseButton, int state, int x, int y)
{
  if(!*mgvnEnabled)
    return false;

  if((y<mnMenuTop)||(y>mnMenuTop + mnMenuHeight))
    return false;
  if(x<mnLeftMostCoord)
    return false;
  
  // if no menu displayed, then must display root menu!
  if(msCurrentSubMenu == "")
    {
      msCurrentSubMenu = "Root";
      return true;
    };
  
  // Figure out which button was pressed:
  int nButtonNumber = (mnWidth - x) / *mgvnMenuItemWidth;
  if(nButtonNumber > (int)(mmSubMenus[msCurrentSubMenu].mvItems.size()))
    nButtonNumber = 0;

  if(nButtonNumber==0) // Clicked on menu name .. . go to root.
    {
      if(msCurrentSubMenu =="Root")
	msCurrentSubMenu = "";
      else
	msCurrentSubMenu = "Root";
      return true;
    };
  
  MenuItem SelectedItem  = mmSubMenus[msCurrentSubMenu].mvItems[nButtonNumber-1];
  msCurrentSubMenu=SelectedItem.sNextMenu;
  switch(SelectedItem.type)
    {
    case Button:
      GUI.ParseLine(SelectedItem.sParam);
      break;
    case Toggle:
      *(SelectedItem.gvnIntValue)^=1;
      break;
    case Slider:
      {
	if(nMouseButton == GLWindow::BUTTON_WHEEL_UP)
	  {
	    *(SelectedItem.gvnIntValue)+=1;
	    if(*(SelectedItem.gvnIntValue) > SelectedItem.max)
	      *(SelectedItem.gvnIntValue) = SelectedItem.max;
	  }
	else if(nMouseButton == GLWindow::BUTTON_WHEEL_DOWN)
	  {
	    *(SelectedItem.gvnIntValue)-=1;
	    if(*(SelectedItem.gvnIntValue) < SelectedItem.min)
	      *(SelectedItem.gvnIntValue) = SelectedItem.min;
	  }
	else
	  {
	    int nPos = *mgvnMenuItemWidth - ((mnWidth - x) % *mgvnMenuItemWidth);
	    double dFrac = (double) nPos / *mgvnMenuItemWidth;
	    *(SelectedItem.gvnIntValue) = (int)(dFrac * (1.0 + SelectedItem.max - SelectedItem.min)) + SelectedItem.min;
	  };
      }
      break;
    case Monitor:
      break;
    };
  return true;
  
};






