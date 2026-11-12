#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3F, 16, 2);

void processInput(String input);
void prepareContinuous(String text);
void updateDisplay(bool force = false);
String padTo16(String text);
void prepareLineScroll(int line, String text);
void updateLineScroll();
void clearDisplay();

String lines[10];         
int totalLines = 0;
int scrollIndex = 0;
unsigned long lastScroll = 0;

bool modeContinuous = false; 

String scrollText[2] = {"", ""};
int scrollCharIndex[2] = {0, 0};

void setup() {
  Serial1.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.clear();
}

void loop() {
  if (Serial1.available()) {
    String message = Serial1.readStringUntil('\n');
    message.trim();
    processInput(message);
  }

  if (modeContinuous) {
    if (millis() - lastScroll >= 2000 && totalLines > 0) {
      lastScroll = millis();
      updateDisplay();
    }
  } else {
    if (millis() - lastScroll >= 300) {
      lastScroll = millis();
      updateLineScroll();
    }
  }
}

void processInput(String input) {
  if (input.equalsIgnoreCase("clear")) {
    clearDisplay();
    return;
  }

  if (input.startsWith("L1:")) {
    modeContinuous = false;
    prepareLineScroll(0, input.substring(3));
  }
  else if (input.startsWith("L2:")) {
    modeContinuous = false;
    prepareLineScroll(1, input.substring(3));
  }
  else if (input.startsWith("Continuous:")) {
    modeContinuous = true;
    prepareContinuous(input.substring(11));
  }
}

void clearDisplay() {
  lcd.clear();
  scrollText[0] = "";
  scrollText[1] = "";
  totalLines = 0;
}

void prepareLineScroll(int line, String text) {
  scrollText[line] = text + " ||"; 
  scrollCharIndex[line] = 0;

  lcd.setCursor(0, line);
  lcd.print(padTo16(scrollText[line].substring(0, min(16, scrollText[line].length()))));
}

void updateLineScroll() {
  for (int line = 0; line < 2; line++) {
    if (scrollText[line].length() > 16) {
      String displayText = scrollText[line].substring(scrollCharIndex[line]);
      if (displayText.length() < 16)
        displayText += scrollText[line].substring(0, 16 - displayText.length());

      lcd.setCursor(0, line);
      lcd.print(displayText.substring(0, 16));

      scrollCharIndex[line]++;
      if (scrollCharIndex[line] >= scrollText[line].length())
        scrollCharIndex[line] = 0;
    }
  }
}

void prepareContinuous(String text) {
  totalLines = 0;
  scrollIndex = 0;

  for (int i = 0; i < 10; i++) lines[i] = "";

  while (text.length() > 0) {
    lines[totalLines] = text.substring(0, 16);
    text = text.substring(16);
    totalLines++;
    if (totalLines >= 10) break;
  }

  lcd.clear();
  updateDisplay(true);
}

void updateDisplay(bool force) {
  if (totalLines == 0) return;

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print(lines[scrollIndex]);

  if (scrollIndex + 1 < totalLines) {
    lcd.setCursor(0, 1);
    lcd.print(lines[scrollIndex + 1]);
  }

  scrollIndex++;
  if (scrollIndex >= totalLines) scrollIndex = 0;
}

String padTo16(String text) {
  while (text.length() < 16) text += " ";
  return text;
}
