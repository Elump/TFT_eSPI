// The following touch screen support code by maxpautsch was merged 1/10/17
// https://github.com/maxpautsch

// Define TOUCH_CS is the user setup file to enable this code

// A demo is provided in examples Generic folder

// Additions by Bodmer to double sample, use Z value to improve detection reliability
// and to correct rotation handling

// See license in root directory.

// Define a default pressure threshold
#ifndef Z_THRESHOLD
  #define Z_THRESHOLD 350 // Touch pressure threshold for validating touches
#endif

/***************************************************************************************
** Function name:           begin_touch_read_write - was spi_begin_touch
** Description:             Start transaction and select touch controller
***************************************************************************************/
// The touch controller has a low SPI clock rate
inline void TFT_eSPI::begin_touch_read_write(void){
  DMA_BUSY_CHECK;
  CS_H; // Just in case it has been left low
  #if defined (SPI_HAS_TRANSACTION) && defined (SUPPORT_TRANSACTIONS)
    if (locked) {locked = false; spi.beginTransaction(SPISettings(SPI_TOUCH_FREQUENCY, MSBFIRST, SPI_MODE0));}
  #else
    spi.setFrequency(SPI_TOUCH_FREQUENCY);
  #endif
  SET_BUS_READ_MODE;
  T_CS_L;
}

/***************************************************************************************
** Function name:           end_touch_read_write - was spi_end_touch
** Description:             End transaction and deselect touch controller
***************************************************************************************/
inline void TFT_eSPI::end_touch_read_write(void){
  T_CS_H;
  #if defined (SPI_HAS_TRANSACTION) && defined (SUPPORT_TRANSACTIONS)
    if(!inTransaction) {if (!locked) {locked = true; spi.endTransaction();}}
  #else
    spi.setFrequency(SPI_FREQUENCY);
  #endif
  //SET_BUS_WRITE_MODE;  //CL200930: keep de-activated this line
}

/***************************************************************************************
** Function name:           Legacy - deprecated
** Description:             Start/end transaction
***************************************************************************************/
void TFT_eSPI::spi_begin_touch() {begin_touch_read_write();}
void TFT_eSPI::spi_end_touch()   {  end_touch_read_write();}

/***************************************************************************************
** Function name:           getTouchRaw
** Description:             read raw touch position.  Always returns true.
***************************************************************************************/
uint8_t TFT_eSPI::getTouchRaw(uint16_t *x, uint16_t *y){
  uint16_t tmp;

  begin_touch_read_write();
  
  // Start YP sample request for x position, read 4 times and keep last sample
  spi.transfer(0xd0);                    // Start new YP conversion
  spi.transfer(0);                       // Read first 8 bits
  spi.transfer(0xd0);                    // Read last 8 bits and start new YP conversion
  spi.transfer(0);                       // Read first 8 bits
  spi.transfer(0xd0);                    // Read last 8 bits and start new YP conversion
  spi.transfer(0);                       // Read first 8 bits
  spi.transfer(0xd0);                    // Read last 8 bits and start new YP conversion

  tmp = spi.transfer(0);                   // Read first 8 bits
  tmp = tmp <<5;
  tmp |= 0x1f & (spi.transfer(0x90)>>3);   // Read last 8 bits and start new XP conversion

  *x = tmp;

  // Start XP sample request for y position, read 4 times and keep last sample
  spi.transfer(0);                       // Read first 8 bits
  spi.transfer(0x90);                    // Read last 8 bits and start new XP conversion
  spi.transfer(0);                       // Read first 8 bits
  spi.transfer(0x90);                    // Read last 8 bits and start new XP conversion
  spi.transfer(0);                       // Read first 8 bits
  spi.transfer(0x90);                    // Read last 8 bits and start new XP conversion

  tmp = spi.transfer(0);                 // Read first 8 bits
  tmp = tmp <<5;
  tmp |= 0x1f & (spi.transfer(0)>>3);    // Read last 8 bits

  *y = tmp;

  end_touch_read_write();

  return true;
}

/***************************************************************************************
** Function name:           getTouchRawZ
** Description:             read raw pressure on touchpad and return Z value. 
***************************************************************************************/
uint16_t TFT_eSPI::getTouchRawZ(void){

  begin_touch_read_write();

  // Z sample request
  int16_t tz = 0xFFF;
  spi.transfer(0xb0);               // Start new Z1 conversion
  tz += spi.transfer16(0xc0) >> 3;  // Read Z1 and start Z2 conversion
  tz -= spi.transfer16(0x00) >> 3;  // Read Z2

  end_touch_read_write();

  if (tz == 4095) tz = 0;

  return (uint16_t)tz;
}

/***************************************************************************************
** Function name:           validTouch
** Description:             read validated position. Return false if not pressed. 
***************************************************************************************/
#define _RAWERR 20 // Deadband error allowed in successive position samples
uint8_t TFT_eSPI::validTouch(uint16_t *x, uint16_t *y, uint16_t threshold){
  uint16_t x_tmp, y_tmp, x_tmp2, y_tmp2;

  // Wait until pressure stops increasing to debounce pressure
  uint16_t z1 = 1;  //CL200930: actual pressure
  uint16_t z2 = 0;  //CL200930: last pressure
  while (z1 > z2)
  {
    z2 = z1;
    z1 = getTouchRawZ();
    delay(1);
  }

  //  Serial.print("Z = ");Serial.println(z1);

  if (z1 <= threshold) return false;
    
  getTouchRaw(&x_tmp,&y_tmp);

  //  Serial.print("Sample 1 x,y = "); Serial.print(x_tmp);Serial.print(",");Serial.print(y_tmp);
  //  Serial.print(", Z = ");Serial.println(z1);

  delay(1); // Small delay to the next sample
  if (getTouchRawZ() <= threshold) return false;

  delay(2); // Small delay to the next sample
  getTouchRaw(&x_tmp2,&y_tmp2);
  
  //  Serial.print("Sample 2 x,y = "); Serial.print(x_tmp2);Serial.print(",");Serial.println(y_tmp2);
  //  Serial.print("Sample difference = ");Serial.print(abs(x_tmp - x_tmp2));Serial.print(",");Serial.println(abs(y_tmp - y_tmp2));

  if (abs(x_tmp - x_tmp2) > _RAWERR) return false;
  if (abs(y_tmp - y_tmp2) > _RAWERR) return false;
  
  // CL200930: avarage of two samples
  *x = (x_tmp+x_tmp2)>>1;
  *y = (y_tmp+y_tmp2)>>1;
  
  return true;
}
  
/***************************************************************************************
** Function name:           getTouch
** Description:             read callibrated position. Return false if not pressed. 
***************************************************************************************/
uint8_t TFT_eSPI::getTouch(uint16_t *x, uint16_t *y, uint16_t threshold){
  uint16_t x_tmp, y_tmp;
  
  if (threshold<20) threshold = 20;
  if (_pressTime > millis()) threshold=20;

  uint8_t n = 5;
  uint8_t valid = 0;
  while (n--)
  {
    if (validTouch(&x_tmp, &y_tmp, threshold)) valid++;;
  }

  if (valid<1) { _pressTime = 0; return false; }
  
  _pressTime = millis() + 50;

  convertRawXY(&x_tmp, &y_tmp);

  if (x_tmp >= _width || y_tmp >= _height) return false;

  _pressX = x_tmp;
  _pressY = y_tmp;
  *x = _pressX;
  *y = _pressY;
  return valid;
}

/***************************************************************************************
** Function name:           convertRawXY
** Description:             convert raw touch x,y values to screen coordinates 
***************************************************************************************/
void TFT_eSPI::convertRawXY(uint16_t *x, uint16_t *y)
{
  uint16_t x_tmp = *x, y_tmp = *y, xx, yy;

  if(!touchCalibration_rotate){
    xx=(x_tmp-touchCalibration_x0)*_width/touchCalibration_x1;
    yy=(y_tmp-touchCalibration_y0)*_height/touchCalibration_y1;
    if(touchCalibration_invert_x)
      xx = _width - xx;
    if(touchCalibration_invert_y)
      yy = _height - yy;
  } else {
    xx=(y_tmp-touchCalibration_x0)*_width/touchCalibration_x1;
    yy=(x_tmp-touchCalibration_y0)*_height/touchCalibration_y1;
    if(touchCalibration_invert_x)
      xx = _width - xx;
    if(touchCalibration_invert_y)
      yy = _height - yy;
  }
  *x = xx;
  *y = yy;
}

/***************************************************************************************
** Function name:           calibrateTouch
** Description:             generates calibration parameters for touchscreen. 
***************************************************************************************/
void TFT_eSPI::calibrateTouch(uint16_t *parameters, uint32_t color_fg, uint32_t color_bg, uint8_t size){
  int16_t values[] = {0,0,0,0,0,0,0,0};
  uint16_t x_tmp, y_tmp;
  uint16_t x_offset, y_offset;    // added: Offset for calibration points

  // CL: counter for 4 corners + last loop to clear the calibration points
  for(uint8_t i = 0; i<=4; i++){
    //Clear all 4 arrow areas
	  fillRect(size/2, size/2, size+2, size+2, color_bg);				    			         //up left
    fillRect(size/2, _height-size-size/2, size+2, size+2, color_bg);				     //bot left
    fillRect(_width-size-size/2, size/2, size+2, size+2, color_bg);					     //up right
    fillRect(_width-size-size/2, _height-size-size/2, size+2, size+2, color_bg); //bot right

    if (i == 4) break; // last loop used to clear the arrows
    
    switch (i) {
      case 0: // up left
        //CL: alternativ arrow not target cross 
        //drawLine(0, 0, 0, size, color_fg);
        //drawLine(0, 0, size, 0, color_fg);
        //drawLine(0, 0, size , size, color_fg);
		
        //offset from corner
        x_offset=size/2;
        y_offset=size/2;
        break;
      case 1: // bot left
        //CL: alternativ arrow not target cross 
        //drawLine(0, _height-size-1, 0, _height-1, color_fg);
        //drawLine(0, _height-1, size, _height-1, color_fg);
        //drawLine(size, _height-size-1, 0, _height-1 , color_fg);
		
        //offset from corner
        x_offset=size/2;
        y_offset=_height-size-size/2;
        break;
      case 2: // up right
        //CL: alternativ arrow not target cross 
        //drawLine(_width-size-1, 0, _width-1, 0, color_fg);
        //drawLine(_width-size-1, size, _width-1, 0, color_fg);
        //drawLine(_width-1, size, _width-1, 0, color_fg);
		
        //offset from corner
        x_offset=_width-size-size/2;
        y_offset=size/2;
        break;
      case 3: // bot right
        //CL: alternativ arrow not target cross 
        //drawLine(_width-size-1, _height-size-1, _width-1, _height-1, color_fg);
        //drawLine(_width-1, _height-1-size, _width-1, _height-1, color_fg);
        //drawLine(_width-1-size, _height-1, _width-1, _height-1, color_fg);
		
        //offset from corner
        x_offset=_width-size-size/2;
        y_offset=_height-size-size/2;
        break;
    }
    
    //CL: target cross with offset from corner
    drawLine(size/2+x_offset, y_offset, size/2+x_offset, size+y_offset, color_fg);  //vertical line
    drawLine(x_offset, size/2+y_offset, size+x_offset, size/2+y_offset, color_fg);  //horizontal line
    drawCircle(size/2+x_offset, size/2+y_offset, size/4+size/8, color_fg);  //circle

    // user has to get the chance to release
    if(i>0) delay(1000);

    // CL: get average of 8 samples
    for(uint8_t j= 0; j<8; j++){
      //CL: no need to use a lower detect threshold ((Z_THRESHOLD/2) since sensing is not too close in corners
      while(!validTouch(&x_tmp, &y_tmp, Z_THRESHOLD));
        values[i*2  ] += x_tmp;
        values[i*2+1] += y_tmp;
      }

      values[i*2  ] /= 8;
      values[i*2+1] /= 8;
      //CL: offset still needs compensation
  }


  // from case 0 to case 1, the y value changed. 
  // If the measured delta of the touch x axis is bigger than the delta of the y axis, the touch and TFT axes are switched.
  touchCalibration_rotate = false;
  if(abs(values[0]-values[2]) > abs(values[1]-values[3])){
    touchCalibration_rotate = true;
    touchCalibration_x0 = (values[1] + values[3])/2; // calc min x
    touchCalibration_x1 = (values[5] + values[7])/2; // calc max x
    touchCalibration_y0 = (values[0] + values[4])/2; // calc min y
    touchCalibration_y1 = (values[2] + values[6])/2; // calc max y
  } else {
    touchCalibration_x0 = (values[0] + values[2])/2; // calc min x
    touchCalibration_x1 = (values[4] + values[6])/2; // calc max x
    touchCalibration_y0 = (values[1] + values[5])/2; // calc min y
    touchCalibration_y1 = (values[3] + values[7])/2; // calc max y
  }

  // in addition, the touch screen axis could be in the opposite direction of the TFT axis
  touchCalibration_invert_x = false;
  if(touchCalibration_x0 > touchCalibration_x1){
    values[0]=touchCalibration_x0;
    touchCalibration_x0 = touchCalibration_x1;
    touchCalibration_x1 = values[0];
    touchCalibration_invert_x = true;
  }
  touchCalibration_invert_y = false;
  if(touchCalibration_y0 > touchCalibration_y1){
    values[0]=touchCalibration_y0;
    touchCalibration_y0 = touchCalibration_y1;
    touchCalibration_y1 = values[0];
    touchCalibration_invert_y = true;
  }
  
  // CL: compensat for clibration offset to real corners
  //
  // real x0  measured x0      measured x1  real x1
  // |          |                    |          |
  //(size/2+size/2)                 (size/2+size/2)
  //
  // |                                          |
  //  ----- _width -----------------------------
  //            |                    |       
  //              _width - 2* size
  //
  // measured x1 - measured x0 /_width - 2* size = touch_resolution per pixel
  // offset = touch_resolution per pixel * (size/2+size/2)
  //
  // use times 8 just for better resulution
  x_offset = (touchCalibration_x1 * 8 - (touchCalibration_x0 * 8)) / (_width  - (2 * size)) * size;
  y_offset = (touchCalibration_y1 * 8 - (touchCalibration_y0 * 8)) / (_height - (2 * size)) * size; 
  touchCalibration_x0 -= (x_offset/8);
  touchCalibration_y0 -= (y_offset/8);
  touchCalibration_x1 += (x_offset/8);
  touchCalibration_y1 += (y_offset/8);

  // pre calculate
  touchCalibration_x1 -= touchCalibration_x0;
  touchCalibration_y1 -= touchCalibration_y0;

  // avoid "0" values
  if(touchCalibration_x0 == 0) touchCalibration_x0 = 1;
  if(touchCalibration_x1 == 0) touchCalibration_x1 = 1;
  if(touchCalibration_y0 == 0) touchCalibration_y0 = 1;
  if(touchCalibration_y1 == 0) touchCalibration_y1 = 1;

  // export parameters, if pointer valid
  if(parameters != NULL){
    parameters[0] = touchCalibration_x0;
    parameters[1] = touchCalibration_x1;
    parameters[2] = touchCalibration_y0;
    parameters[3] = touchCalibration_y1;
    parameters[4] = touchCalibration_rotate | (touchCalibration_invert_x <<1) | (touchCalibration_invert_y <<2);
  }
}


/***************************************************************************************
** Function name:           setTouch
** Description:             imports calibration parameters for touchscreen. 
***************************************************************************************/
void TFT_eSPI::setTouch(uint16_t *parameters){
  touchCalibration_x0 = parameters[0];
  touchCalibration_x1 = parameters[1];
  touchCalibration_y0 = parameters[2];
  touchCalibration_y1 = parameters[3];

  if(touchCalibration_x0 == 0) touchCalibration_x0 = 1;
  if(touchCalibration_x1 == 0) touchCalibration_x1 = 1;
  if(touchCalibration_y0 == 0) touchCalibration_y0 = 1;
  if(touchCalibration_y1 == 0) touchCalibration_y1 = 1;

  touchCalibration_rotate = parameters[4] & 0x01;
  touchCalibration_invert_x = parameters[4] & 0x02;
  touchCalibration_invert_y = parameters[4] & 0x04;
}
