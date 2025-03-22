// Code test button 
  pressed = tft.getTouch(&xpos, &ypos);

  // Draw a white spot at the detected coordinates
  if (pressed) {
    if ((xpos > REDBUTTON_X) && (xpos < (REDBUTTON_X + REDBUTTON_W))) {
      if ((ypos > REDBUTTON_Y) && (ypos <= (REDBUTTON_Y + REDBUTTON_H))) {
        tft.setTextColor(TFT_WHITE);
        //tft.setTextDatum(TC_DATUM);  // Centre text on x,y position
        tft.setFreeFont(FSB9);       // Select the font
        tft.drawString(String(temperature,1), REDBUTTON_X, REDBUTTON_Y, GFXFF);
        Serial.print("da cham vaoooo ");
      }
    }

    //tft.fillCircle(xpos, xpos, 2, TFT_BLACK);
    Serial.print("x,y = ");
    Serial.print(xpos);
    Serial.print(",");
    Serial.println(ypos);
    
    pressed = 0;
  }


  //----------------------------