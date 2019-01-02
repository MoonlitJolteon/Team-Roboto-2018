#include <Adafruit_GFX.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_NeoMatrix.h>
#include <SD.h>
#include <SPI.h>
//#include <TFT.h>

#define SD_CS  BUILTIN_SDCARD

Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(44, 11, 2, 1, 6,
                            NEO_MATRIX_TOP     + NEO_MATRIX_LEFT +
                            NEO_MATRIX_ROWS + NEO_MATRIX_ZIGZAG +
                            NEO_TILE_TOP + NEO_TILE_LEFT +
                            NEO_TILE_ROWS + NEO_TILE_PROGRESSIVE,
                            NEO_GRB            + NEO_KHZ800);
int mac = 100;
String commands[71][16];
String test;
void setup() {
  Serial.println("test");
  matrix.begin();
  matrix.setBrightness(15);
  matrix.setTextColor( matrix.Color(255, 0, 0) );
  matrix.setTextWrap(false);
  matrix.fillScreen(matrix.Color(50, 50, 50));
  matrix.show();
  

  SD.begin(SD_CS);
  File text = SD.open("Current.txt");
  test = text.readString();
  Serial.println(test);
  char ch[test.length()];
  text.close();
  test.toCharArray(ch, test.length() + 1);
  File text2 = SD.open(ch);
  int x = 0;
  int y = 0;
  char t2;

  for (int i = 0; i < text2.size(); i++) {
    t2 = text2.read();
    if (t2 == char(10)) {
      x++;
      y = 0;
    } else if (t2 == char(32)) {
      y++;
    } else {
      commands[x][y] = commands[x][y] + t2;
    }
  }
  test = "";
}

void loop() {
  for (int y = 0; y < mac; y++) {
    int comint[16];
    for (int x = 0; x < 16; x++) {
      comint[x] = commands[y][x].toInt();
    }
    if (commands[y][0] == "slide") {
        slide(matrix.Color(comint[1], comint[2], comint[3]), matrix.Color(comint[4], comint[5], comint[6]));
        Serial.println("slide");
    } else if (commands[y][0] == "scrollText") {
        scrollText(commands[y][1].replace("_", " "), matrix.Color(comint[2], comint[3], comint[4]), matrix.Color(comint[5], comint[6], comint[7]));
        Serial.println("scrollText");
    } else if (commands[y][0] == "drawMatrix") {
        Serial.println("drawMatrix");
        drawMatrix();
    } else if (commands[y][0] == "waveFade") {
        Serial.println("waveFade");
        waveFade(comint[1], comint[2] , comint[3] , comint[4] );
    } else if (commands[y][0] == "doTheWave") {
        Serial.println("doTheWave");
        doTheWave(comint[1], comint[2], matrix.Color(comint[3], comint[4], comint[5]), matrix.Color(comint[6], comint[7], comint[8]));
    } else if (commands[y][0] == "redVBlue") {
        Serial.println("redVBlue");
        redVBlue();
    } else if (commands[y][0] == "delay") {
        Serial.println("delay");
        delay(comint[1]);
    } else if (commands[y][0] == "image") {
        Serial.println("image");
        image(commands[y][1]);
    } else if (commands[y][0] == "scrollImage") {
        Serial.println("scrollImage");
        scrollImage(commands[y][1]);
    } else if (commands[y][0] == "anImage") {
        Serial.println("anImage");
        anImage(commands[y][1], comint[2], comint[3]);
    }
  }
}










int Delay = 10;

void waveFade(int rLength, int gLength, int bLength, int time1) {
  for (int i = 0; i < time1 * 1000 / 60; i++) {
    for (int x = 0; x < matrix.width(); x++) {
      int r = waver(x, i, rLength, 128);
      int g = waver(x, i, gLength, 128);
      int b = waver(x, i, bLength, 128);
      matrix.drawLine(x, 0, x, matrix.height(), matrix.Color(r, g, b));
    }
    matrix.show();
    delay(Delay);
  }
}

int waver(int x, int i, int length1, int hightm) {
  return floor(hightm * cos((x - i) * 2 * PI / length1) + hightm);
}

void doTheWave(int time1, int length1, int color1, int color2) {
  for (int i = 0; i < time1 * 1000 / Delay; i++) {
    for (int x = 0; x < matrix.width(); x++) {
      for (int y = 0; y < matrix.height(); y++) {
        if (y == waver(x, i, length1, 5)) {
          matrix.drawPixel(x, y, color1);
        } else {
          matrix.drawPixel(x, y, color2);
        }
      }
    }
    matrix.show();
    delay(Delay);
  }
}
void slide(int colorOne, int colorTwo) {
  for (int x = 0; x < matrix.width(); x++) {
    for (int y = 0; y < matrix.height(); y++) {
      matrix.drawPixel(x, y, colorOne);
    }
    delay(10);
    matrix.show();
  }
  for (int x = 0; x < matrix.width(); x++) {
    for (int y = 0; y < matrix.height(); y++) {
      matrix.drawPixel(x, y, colorTwo);
    }
    delay(10);
    matrix.show();
  }
}

void scrollText(String textToDisplay, int color1, int color2) {
  int x = matrix.width();

  // Account for 6 pixel wide characters plus a space
  int pixelsInText = textToDisplay.length() * 7;

  matrix.setCursor(x, 0);
  matrix.print(textToDisplay);
  matrix.show();
  matrix.setTextColor( color1 );
  while (x > (0 - pixelsInText)) {
    matrix.fillScreen(color2);
    matrix.setCursor(--x, 2);
    matrix.print(textToDisplay);
    matrix.show();
    delay(10);
  }

}

void drawMatrix() {
  int drawerWidth = 88;
  int drawerLength = 11;

  int drawer[drawerLength][drawerWidth] {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
  };
  for (int x = 0; x < drawerWidth; x++) {
    for (int y = 0; y < drawerLength; y++) {
      if (drawer[y][x] == 1) {
        matrix.drawPixel(x, y, matrix.Color(255, 0, 0));
      } else {
        matrix.drawPixel(x, y, matrix.Color(0, 0, 100));
      }
    }
    matrix.show();
  }
}
void redVBlue() {
  matrix.fillRect(0, 0, 44, 11, matrix.Color(0, 0, 255));
  matrix.fillRect(44, 0, 87, 11, matrix.Color(255, 0, 0));
  matrix.setTextColor( matrix.Color(255, 0, 0) );
  matrix.setCursor(10, 2);
  matrix.print("Blue");
  matrix.setTextColor( matrix.Color(0, 0, 255) );
  matrix.setCursor(58, 2);
  matrix.print("Red");
  matrix.show();
}
/*void siren(int time1) {
  for (int x = 0; x > time1 * 200 / 1000; x++0) {
    matrix.fillRect(0, 0, 44, 11, matrix.Color(0, 0, 255));
    matrix.fillRect(44, 0, 87, 11, matrix.Color(0, 0, 0));
    matrix.show();
    delay(200);
    matrix.fillRect(0, 0, 44, 11, matrix.Color(0, 0, 0));
    matrix.fillRect(44, 0, 87, 11, matrix.Color(255, 0, 0));
    matrix.show();
    delay(200);
  }
  }
*/
void image(String filename) {

  char ch[filename.length()];
  filename.toCharArray(ch, filename.length());
  File img = SD.open(ch);

  int st = 0;
  img.seek(10);
  st = img.read();

  img.seek(st);

  int red;
  int blue;
  int green;
  for (int y = 10; y > -1; y--) {
    for (int x = 0; x < 88; x++) {
      blue = img.read();
      green = img.read();
      red = img.read();
      matrix.drawPixel(x, y, matrix.Color(red, green, blue));
    }
  }
  matrix.show();
  img.close();
}
void scrollImage(String filename) {
  for (int i = 1; i < 89; i++) {
    char ch[filename.length()];
    filename.toCharArray(ch, filename.length());
    File img = SD.open(ch);

    int st = 0;
    img.seek(10);
    st = img.read();

    img.seek(st);

    int red;
    int blue;
    int green;
    for (int y = 10; y > -1; y--) {
      for (int x = 0; x < 88; x++) {
        blue = img.read();
        green = img.read();
        red = img.read();
        if (x < i) {
          matrix.drawPixel(x, y, matrix.Color(red, green, blue));
        }
      }
    }
    matrix.show();
    delay(5);
    img.close();
  }
}
void anImage(String filename, int frames, int del) {

  char ch1[filename.length()];
  filename.toCharArray(ch1, filename.length() + 1);
  File img = SD.open(ch1);

  int st = 0;
  img.seek(10);
  st = img.read();

  img.seek(st);
  for (int i = 0; i < frames; i++) {
    int red;
    int blue;
    int green;
    for (int y = 10; y > -1; y--) {
      for (int x = 0; x < 88; x++) {
        blue = img.read();
        green = img.read();
        red = img.read();
        matrix.drawPixel(x, y, matrix.Color(red, green, blue));
      }
    }
    matrix.show();
    delay(del);
  }
  img.close();
}
void testText(String frames) {

  matrix.fillRect(0, 0, 88, 11, matrix.Color(0, 0, 0));
  matrix.setCursor(1, 1);
  matrix.print(frames);
  matrix.show();
  delay(3000);
}











