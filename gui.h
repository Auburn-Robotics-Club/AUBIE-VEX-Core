#pragma once
//TEST
extern vex::brain Brain;
using namespace vex;

class GUIObject {
protected:
  int x;
  int y;
  int width;
  int height;
  void (*drawing)(int x, int y, int w, int h);

public:
  GUIObject(int X, int Y, int W, int H, void (*drawFunct)(int x, int y, int w, int h)){
    x = X;
    y = Y;
    width = W;
    height = H;
    drawing = drawFunct;
  }

  int getX(){
    return x;
  }

  void setX(int newX){
    x = newX;
  }

  void setY(int newY){
    y = newY;
  }

  int getY(){
    return y;
  }
  
  void setWidth(int newWidth){
    width = newWidth;
  }

  int getWidth(){
    return width;
  }

  void setHeight(int newHeight){
    height = newHeight;
  }

  int getHeight(){
    return height;
  }

  void draw(){
    drawing(x, y, width, height);
  }
};

class GUICallable : public GUIObject {
protected:
  void (*drawing)(int x, int y, int w, int h, bool press);
  void (*callback)();
  bool isPressed = false;

public:
  GUICallable(int X, int Y, int W, int H, void (*drawFunct)(int x, int y, int w, int h, bool press), void (*call)()) :
              GUIObject(X, Y, W, H, nullptr) {
    drawing = drawFunct;
    
  }

  bool call(int X, int Y){
      if((X >= x) && (X <= (x+width))){
        if((Y >= y) && (Y <= (y+height))){
          callback();
          isPressed = true;
          return true;
        }
      }
      return false;
    }
};

class ButtonGUI {
  public:
    int x;
    int y;
    int width;
    int height;
    void (*callback)();
    void (*drawing)(int x, int y, int w, int h, bool press);
    int *point;
    bool isPressed = false;

    ButtonGUI(int X, int Y, int W, int H, void (*call)(), void (*drawFunct)(int x, int y, int w, int h, bool press)){
      x = X;
      y = Y;
      width = W;
      height = H;
      callback = call;
      drawing = drawFunct;
    }

    bool call(int X, int Y){
      if((X >= x) && (X <= (x+width))){
        if((Y >= y) && (Y <= (y+height))){
          callback();
          isPressed = true;
          return true;
        }
      }
      return false;
    }

    void draw(){
      drawing(x, y, width, height, isPressed);
    }
};

void drawLoadingScreen(){
  Brain.Screen.clearScreen();
  if(Brain.SDcard.isInserted()){
    Brain.Screen.drawImageFromFile("images/brainscreen_loading.png", 0, 0);
  } else {

  }
}

void drawSelectingScreen(){
  Brain.Screen.clearScreen();
  if(Brain.SDcard.isInserted()){
    Brain.Screen.drawImageFromFile("images/brainscreen_selecting.png", 0, 0);
  }  else {

  }
}

void drawFrisbeeButton(int x, int y, int w, int h, bool press){
  if(Brain.SDcard.isInserted()){
    Brain.Screen.drawImageFromFile("images/button.png", x, y);
  }  else {
    if(press){
      Brain.Screen.setFillColor(green);
    }else{
      Brain.Screen.setFillColor(red);
    }
    Brain.Screen.drawRectangle(x, y, w, h);
  }
}