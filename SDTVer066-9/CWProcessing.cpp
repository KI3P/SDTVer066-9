#ifndef BEENHERE
#include "SDT.h"
#endif

//=================  AFP10-18-22 ================
/*****
  Purpose: Select CW Filter. CWFilterIndex has these values:
           0 = 840Hz
           1 = 1kHz
           2 = 1.3kHz
           3 = 1.8kHz
           4 = 2kHz
           5 = Off

  Parameter list:
    void

  Return value:
    void
*****/
void SelectCWFilter() 
{
  int val;
  long lastFilter = 0;

  filterEncoderMove = 0;
  CWFilterIndex = 0;
  // Nonsense change to test github.dev
  tft.setFontScale((enum RA8875tsize)1);

  tft.fillRect(SECONDARY_MENU_X, MENUS_Y, EACH_MENU_WIDTH, CHAR_HEIGHT, RA8875_MAGENTA);
  tft.setTextColor(RA8875_WHITE);
  tft.setCursor(SECONDARY_MENU_X + 1, MENUS_Y + 1);
//  tft.print("current index:");
//  tft.setCursor(SECONDARY_MENU_X + 200, MENUS_Y + 1);
  tft.print(CWFilter[filterEncoderMove]);

  while (true) {
    if (filterEncoderMove != 0) {           // Changed encoder?
      CWFilterIndex += filterEncoderMove;   // Yep
      lastFilter = CWFilterIndex;
      if (CWFilterIndex < 0) {
        CWFilterIndex = 5;
      } else { 
        if (CWFilterIndex > 5)
          CWFilterIndex = 0;
      }
      tft.fillRect(SECONDARY_MENU_X, MENUS_Y + 1, EACH_MENU_WIDTH, CHAR_HEIGHT, RA8875_MAGENTA);
      tft.setCursor(SECONDARY_MENU_X + 1, MENUS_Y + 1);
      tft.print(CWFilter[CWFilterIndex]);
      filterEncoderMove = 0;
    }

    val = ReadSelectedPushButton();  // Read pin that controls all switches
    val = ProcessButtonPress(val);                                            ///////////////////////////////////////////////////
    if (val == MENU_OPTION_SELECT) {  // Make a choice??
      if (CWFilterIndex == 5)               // Cancel? Don't update index
        return;

      CWFilterIndex = lastFilter;
      EEPROMData.CWFilterIndex = lastFilter;
      EEPROMWrite();
      UpdateWPMField();
      break;
    }
  }
  tft.setTextColor(RA8875_WHITE);
//  EraseMenus();
  return;
}
//=================  AFP10-18-22 ================
/*****
  Purpose: to process CW specific signals

  Parameter list:
    void

  Return value:
    void

*****/
void DoCWReceiveProcessing() {  // All New AFP 09-19-22
  float goertzelMagnitude1;
  float goertzelMagnitude2;
  int audioTemp;  // KF5N
  //arm_copy_f32(float_buffer_R, float_buffer_R_CW, 256);
  //arm_biquad_cascade_df2T_f32(&S1_CW_Filter, float_buffer_R, float_buffer_R_CW, 256);//AFP 09-01-22
  //arm_biquad_cascade_df2T_f32(&S1_CW_Filter, float_buffer_L, float_buffer_L_CW, 256);//AFP 09-01-22

  arm_fir_f32(&FIR_CW_DecodeL, float_buffer_L, float_buffer_L_CW, 256);  // AFP 10-25-22  Park McClellan FIR filter const Group delay
  arm_fir_f32(&FIR_CW_DecodeR, float_buffer_R, float_buffer_R_CW, 256);  // AFP 10-25-22

  //  if (decoderFlag == DECODE_OFF) {                  // AFP 09-27-22
  if (decoderFlag == DECODE_ON) {  // JJP 7/20/23

    //=== end CW Filter ===

    // ----------------------  Correlation calculation  AFP 02-04-22 -------------------------

    //Calculate correlation between calc sine and incoming signal

    arm_correlate_f32(float_buffer_R_CW, 256, sinBuffer, 256, float_Corr_BufferR);
    arm_max_f32(float_Corr_BufferR, 511, &corrResultR, &corrResultIndexR);
    //running average of corr coeff. R
    aveCorrResultR = .7 * corrResultR + .3 * aveCorrResultR;
    arm_correlate_f32(float_buffer_L_CW, 256, sinBuffer, 256, float_Corr_BufferL);
    //get max value of correlation
    arm_max_f32(float_Corr_BufferL, 511, &corrResultL, &corrResultIndexL);
    //running average of corr coeff. L
    aveCorrResultL = .7 * corrResultL + .3 * aveCorrResultL;
    aveCorrResult = (corrResultR + corrResultL) / 2;
    // Calculate Goertzel Mahnitude of incomming signal
    goertzelMagnitude1 = goertzel_mag(256, 750, 24000, float_buffer_L_CW);  //AFP 10-25-22
    goertzelMagnitude2 = goertzel_mag(256, 750, 24000, float_buffer_R_CW);  //AFP 10-25-22
    goertzelMagnitude = (goertzelMagnitude1 + goertzelMagnitude2) / 2;
    //Combine Correlation and Gowetzel Coefficients
    combinedCoeff = 10 * aveCorrResult * 100 * goertzelMagnitude;
    combinedCoeff2 = combinedCoeff;
    // ==========  Changed CW decode "lock" indicator
    if (combinedCoeff > 50) {  // AFP 10-26-22
      tft.fillRect(745, 448, 15, 15, RA8875_GREEN);
    } else if (combinedCoeff < 50) {  // AFP 10-26-22
      CWLevelTimer = millis();
      if (CWLevelTimer - CWLevelTimerOld > 2000) {
        CWLevelTimerOld = millis();
        tft.fillRect(744, 447, 17, 17, RA8875_BLACK);
      }
    }
    combinedCoeff2Old = combinedCoeff2;
    tft.drawFastVLine(BAND_INDICATOR_X + 22, AUDIO_SPECTRUM_BOTTOM - 118, 118, RA8875_GREEN);  //CW lower freq indicator
    tft.drawFastVLine(BAND_INDICATOR_X + 30, AUDIO_SPECTRUM_BOTTOM - 118, 118, RA8875_GREEN);  //CW upper freq indicator
    if (combinedCoeff > 50) {                                                                  // if  have a reasonable corr coeff, >50, then we have a keeper. // AFP 10-26-22
      audioTemp = 1;
    } else {
      audioTemp = 0;
    }
    //==============  acquire data on CW  ================
    DoCWDecoding(audioTemp);
  }
}

/*****
  Purpose: to provide spacing between letters

  Parameter list:
  void

  Return value:
  void

  CAUTION: Assumes that a global named ditLength holds the value for dit spacing
*****/
void LetterSpace() {
  MyDelay(3UL * ditLength);
}
/*****
  Purpose: to provide spacing between words

  Parameter list:
  void

  Return value:
  void

  CAUTION: Assumes that a global named ditLength holds the value for dit spacing
*****/
void WordSpace() {
  MyDelay(7UL * ditLength);
}

/*****
  Purpose: to send a Morse code character

  Parameter list:
    char code       the code for the letter to send

  Return value:
    void
*****/
void SendCode(char code) {
  int i;

  for (i = 7; i >= 0; i--)  // Find the sentinel. Loop looks for first 1,
    if (code & (1 << i))    // which marks the start of the letter:   0b11000 = 'B'
      break;

  for (i--; i >= 0; i--) {  // Now look at rest of binary value: 0b1000 = B after reading sentinel
    if (code & (1 << i))    // If it's a 1, send a dah, otherwise...
      Dah();
    else
      Dit();  // ...send a dit
  }
  LetterSpace();
}


/*****
  Purpose: to send a Morse code character


  Parameter list:
  char myChar       The character to be sent

  Return value:
  void
*****/
void Send(char myChar) {
  if (isalpha(myChar)) {
    if (islower(myChar)) {
      myChar = toupper(myChar);
    }
    SendCode(letterTable[myChar - 'A']);  // Make into a zero-based array index
    return;
  } else if (isdigit(myChar)) {
    SendCode(numberTable[myChar - '0']);  // Same deal here...
    return;
  }

  switch (myChar) {  // Non-alpha and non-digit characters
    case '\r':
    case '\n':
    case '!':
      SendCode(0b01101011);  // exclamation mark 33
      break;
    case '"':
      SendCode(0b01010010);  // double quote 34
      break;
    case '$':
      SendCode(0b10001001);  // dollar sign 36
      break;
    case '@':
      SendCode(0b00101000);  // ampersand 38
      break;
    case '\'':
      SendCode(0b01011110);  // apostrophe 39
      break;

    case '(':
    case ')':
      SendCode(0b01011110);  // parentheses (L) 40, 41
      break;

    case ',':
      SendCode(0b01110011);  // comma 44
      break;

    case '.':
      SendCode(0b01010101);  // period  46
      break;
    case '-':
      SendCode(0b00100001);  // hyphen 45
      break;
    case ':':
      SendCode(0b01111000);  // colon 58
      break;
    case ';':
      SendCode(0b01101010);  // semi-colon 59
      break;
    case '?':
      SendCode(0b01001100);  // question mark 63
      break;
    case '_':
      SendCode(0b01001101);  // underline 95
      break;

    case (char)182:
      SendCode(0b01101000);  // paragraph #182, '¶'
      break;

    case ' ':  // Space
      WordSpace();
      break;

    default:
      WordSpace();
      break;
  }
}

/*****
  Purpose: establish the dit length for code transmission. Crucial since
    all spacing is done using dit length

  Parameter list:
    int wpm

  Return value:
    void
*****/
void SetDitLength(int wpm) {
  ditLength = 1200 / wpm;
}

/*****
  Purpose: establish the dit length for code transmission. Crucial since
    all spacing is done using dit length

  Parameter list:
    int wpm

  Return value:
    void
*****/
void SetTransmitDitLength(int wpm) {
  transmitDitLength = 1200 / wpm;  // JJP 8/19/23
}

/*****
  Purpose: Select straight key or keyer

  Parameter list:
    void

  Return value:
    void
*****/
void SetKeyType() {
  const char *keyChoice[] = { "Straight Key", "Keyer", "Cancel" };

  keyType = EEPROMData.keyType = SubmenuSelect(keyChoice, 3, 0);
  //keyType = SetSecondaryMenuIndex();
  // Make sure the paddleDit and paddleDah variables are set correctly for straight key.
  // Paddle flip can reverse these, making the straight key inoperative.  KF5N August 9, 2023
  if (keyType == 0) {
    paddleDit = KEYER_DIT_INPUT_TIP;
    paddleDah = KEYER_DAH_INPUT_RING;
  }
}


/*****
  Purpose: Set up key at power-up.

  Parameter list:
    void

  Return value:
    void
*****/
void SetKeyPowerUp() {
  if (keyType == 0) {
    paddleDit = KEYER_DIT_INPUT_TIP;
    paddleDah = KEYER_DAH_INPUT_RING;
    return;
  }
  if (paddleFlip) {  // Means right-paddle dit
    paddleDit = KEYER_DAH_INPUT_RING;
    paddleDah = KEYER_DIT_INPUT_TIP;
  } else {
    paddleDit = KEYER_DIT_INPUT_TIP;
    paddleDah = KEYER_DAH_INPUT_RING;
  }
}


/*****
  Purpose: Allow user to set the sidetone volume.  KF5N August 31, 2023

  Parameter list:
    void

  Return value;
    void
*****/

void SetSideToneVolume() 
{
  int val, sidetoneDisplay;


  Q_in_L.clear();  // Clear other buffers too?
  Q_in_R.clear();
  tft.setFontScale((enum RA8875tsize)1);
  tft.fillRect(SECONDARY_MENU_X - 50, MENUS_Y, EACH_MENU_WIDTH + 60, CHAR_HEIGHT, RA8875_MAGENTA);
  tft.setTextColor(RA8875_WHITE);
  tft.setCursor(SECONDARY_MENU_X - 48, MENUS_Y + 1);
  tft.print("Sidetone Volume:");
  tft.setCursor(SECONDARY_MENU_X + 220, MENUS_Y + 1);
  sidetoneDisplay = (int)(sidetoneVolume);
  tft.print(sidetoneDisplay);  // Display in range of 0 to 100.
  modeSelectInR.gain(0, 0);
  modeSelectInL.gain(0, 0);
  modeSelectInExR.gain(0, 0);
  modeSelectOutL.gain(0, 0);
  modeSelectOutR.gain(0, 0);
  modeSelectOutExL.gain(0, 0);
  modeSelectOutExR.gain(0, 0);

  modeSelectOutL.gain(1, 0.0);  // Sidetone  AFP 10-01-22
  modeSelectOutR.gain(1, 0.0);  // Sidetone  AFP 10-01-22

  while (true) {
    if (digitalRead(paddleDit) == LOW || digitalRead(paddleDah) == LOW) CW_ExciterIQData();

    if (filterEncoderMove != 0) {
      //      sidetoneVolume = sidetoneVolume + (float)filterEncoderMove * 0.001;  // sidetoneVolume range is 0.0 to 1.0 in 0.001 steps.  KF5N August 29, 2023
      sidetoneDisplay = sidetoneDisplay + filterEncoderMove;  // * 0.001;  // sidetoneVolume range is 0.0 to 1.0 in 0.001 steps.  KF5N August 29, 2023
      if (sidetoneDisplay < 0)
        sidetoneDisplay = 0;
      else if (sidetoneDisplay > 100)  // 100% max
        sidetoneDisplay = 100;
      tft.fillRect(SECONDARY_MENU_X + 200, MENUS_Y, 70, CHAR_HEIGHT, RA8875_MAGENTA);
      tft.setCursor(SECONDARY_MENU_X + 220, MENUS_Y + 1);
      sidetoneVolume = (float32_t)sidetoneDisplay;
      tft.setTextColor(RA8875_WHITE);
      tft.print(sidetoneDisplay);
      filterEncoderMove = 0;
    }
    modeSelectOutL.gain(1, volumeLog[(int)sidetoneVolume]);  // Sidetone  AFP 10-01-22
                                                             //    modeSelectOutR.gain(1, volumeLog[(int)sidetoneVolume]);  // Right side not used.  KF5N September 1, 2023
    val = ReadSelectedPushButton();                          // Read pin that controls all switches
    val = ProcessButtonPress(val);
    if (val == MENU_OPTION_SELECT) {  // Make a choice??
      EEPROMData.sidetoneVolume = sidetoneVolume;
      EEPROMWrite();
      break;
    }
  }
  EraseMenus();
  lastState = 1111;  // This is required due to the function deactivating the receiver.  This forces a pass through the receiver set-up code.  KF5N October 7, 2023

}


//==================================== Decoder =================
//DB2OO, 29-AUG-23: moved col declaration here
static int col = 0;  // Start at lower left

/*****
    DB2OO, 29-AUG-23: added
  Purpose: This function clears the morse code text buffer

  Parameter list:
    

  Return value
    void
*****/
void MorseCharacterClear(void) {
  col = 0;
  decodeBuffer[col] = '\0';  // Make it a string
}

/*****
  Purpose: This function displays the decoded Morse code below waterfall. Arranged as:

  Parameter list:
    char currentLetter

  Return value
    void
*****/
void MorseCharacterDisplay(char currentLetter) {
  if (col < MAX_DECODE_CHARS) {  // Start scrolling??
    decodeBuffer[col] = currentLetter;
    col++;
    decodeBuffer[col] = '\0';  // Make is a string
  } else {
    //DB2OO, 25-AUG-23: use memmove instead of memcpy(), to avoid the warning
    memmove(decodeBuffer, &decodeBuffer[1], MAX_DECODE_CHARS - 1);  // Slide array down 1 character.
    decodeBuffer[col - 1] = currentLetter;                          // Add to end
    decodeBuffer[col] = '\0';                                       // Make is a string
  }
  tft.fillRect(CW_TEXT_START_X, CW_TEXT_START_Y, CW_MESSAGE_WIDTH, CW_MESSAGE_HEIGHT * 2, RA8875_BLACK);
  tft.setFontScale((enum RA8875tsize)1);
  tft.setTextColor(RA8875_WHITE);
  tft.setCursor(CW_TEXT_START_X, CW_TEXT_START_Y);
  tft.print(decodeBuffer);
}


/*****
  Purpose: When the CW decoder is active, this function allows the user to set the ditLenght, which updates
           the display for the new WPM.

  Parameter list:
    void

  Return value;
    void
*****/
void DisplayDitLength() {
  tft.setFontScale((enum RA8875tsize)0);  // Erase old WPM value
  tft.fillRect(FIELD_OFFSET_X + 6 * tft.getFontWidth(), DECODER_Y, tft.getFontWidth() * 15, tft.getFontHeight(), RA8875_BLACK);
  tft.setCursor(FIELD_OFFSET_X + 6 * tft.getFontWidth(), DECODER_Y);  // Show estimated WPM
  tft.setTextColor(RA8875_WHITE);
  tft.print("[ ");
  tft.setTextColor(RA8875_GREEN);
  tft.print(1200 / ditLength);
  tft.setTextColor(RA8875_WHITE);
  tft.print(" ]");
}


/*****
  Purpose: This function looks up the current character in the decode array using a binary search algorithm.
           It uses a modified binary search algorith that can be seen in Chapter 10, using Figure 10-9.

                      index=0
                      dash_jump=128
                      for each received element
                        dash_jump=dash_jump/2
                        index = index + (e=='.')?1:dash_jump
                      endfor
                      ascii = lookupstring[index]

  Parameter list:
    char currentAtom      is it a dit or a dah?

  Return value
    void
*****/
void Lookup(char currentAtom) {
  /* This shows letter placement in the array after walking the binary tree

char *bigMorseCodeTree  = (char *) "-EISH5--4--V---3--UF--------?-2--ARL---------.--.WP------J---1--TNDB6--.--X/-----KC------Y------MGZ7----,Q------O-8------9--0----";
//                                  012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678
//                                           10        20        30        40        50        60        70        80        90       100       110       120
*/
  currentDashJump = currentDashJump >> 1;  // Fast divide by 2

  if (currentAtom == '.') {
    currentDecoderIndex++;
  } else {
    currentDecoderIndex += currentDashJump;
  }
}

/*****
  Purpose: This function uses the current WPM to set an estimate ditLength any time the tune
           endcoder is changed

  Parameter list:
    void

  Return value
    void
*****/
void ResetHistograms() {
  gapAtom = 80;
  ditLength = 80;  // Start with 15wpm ditLength
  gapChar = 240;
  dahLength = 240;
  thresholdGeometricMean = 160;  // Use simple mean for starters so we don't have 0
  aveDitLength = ditLength;
  aveDahLength = dahLength;
  valRef1 = 0;
  valRef2 = 0;
  // Clear graph arrays
  memset(signalHistogram, 0, HISTOGRAM_ELEMENTS * sizeof(uint32_t));
  memset(gapHistogram, 0, HISTOGRAM_ELEMENTS * sizeof(uint32_t));
  currentWPM = 1200 / ditLength;
  UpdateWPMField();
}

/*****
  Purpose: This function draws the plot axes in the display's waterfall space

  Parameter list:
    void

  Return value;
    void
*****/
void DrawSignalPlotFrame() {
  int offset;
  float val = 0.0;
  tft.fillRect(WATERFALL_LEFT_X, FIRST_WATERFALL_LINE - 5, MAX_WATERFALL_WIDTH + 10, MAX_WATERFALL_ROWS + 30, RA8875_BLACK);

  tft.setFontScale(0);
  tft.setTextColor(RA8875_GREEN);
  tft.drawFastVLine(WATERFALL_LEFT_X + 60, FIRST_WATERFALL_LINE + 5, MAX_WATERFALL_ROWS - 25, RA8875_GREEN);
  tft.drawFastHLine(WATERFALL_LEFT_X + 60, WATERFALL_BOTTOM - 20, MAX_WATERFALL_WIDTH - 80, RA8875_GREEN);
  offset = WATERFALL_BOTTOM - 30;
  for (int i = 0; i < 5; i++) {
    tft.setCursor(WATERFALL_LEFT_X + 15, offset - (i * 30));
    tft.print(val);
    tft.print(" -");
    val += 2.0;
  }
  tft.setTextColor(RA8875_WHITE);
  tft.setCursor(WATERFALL_LEFT_X, FIRST_WATERFALL_LINE);
  tft.print("Signal");
  tft.setCursor(MAX_WATERFALL_WIDTH >> 1, WATERFALL_BOTTOM - 20);
  tft.print("Time");
}

/*****
  Purpose: This function plots the CW signal in the display's waterfall space

  Parameter list:
    float val         the current signal value

  Return value;
    void
*****/
void DoSignalPlot(float val) {
  int i, j;
  int location;
  static short int signalArray[MAX_WATERFALL_ROWS + 1][MAX_WATERFALL_WIDTH + 1];

  location = map(val, 0, 8.0, WATERFALL_TOP_Y, WATERFALL_BOTTOM);  // What row to activate?
  signalArray[location][MAX_WATERFALL_WIDTH] = RA8875_WHITE;       // Turn pixel on.
  for (i = 0; i < MAX_WATERFALL_ROWS; i++) {
    memmove(&signalArray[i], &signalArray[i + 1], MAX_WATERFALL_WIDTH);
  }
  for (i = 0; i < MAX_WATERFALL_ROWS; i++) {
    for (j = 0; j < MAX_WATERFALL_WIDTH; j++) {
      if (signalArray[i][j] != 0) {
        tft.setCursor(WATERFALL_LEFT_X + 61 + i, FIRST_WATERFALL_LINE + 6 + j);
        tft.print('.');
      }
    }
  }
}

// This function was re-factored into a state machine by KF5N October 29, 2023.
/*****
  Purpose: Called when in CW mode and decoder flag is set. Function assumes:

      dit           = 1
      dah           = dit * 3
      inter-atom    = dit
      inter-letter  = dit * 3
      inter-word    = dit * 7

      You can distinguish between dah and inter-letter by presence/absence of signal. Same for inter-atom.

  Parameter list:
    float audioValue        the strength of audio signal

  Return value;
    void
*****/
// charProcessFlag means a character is being decoded.  blankFlag indicates a blank has already been printed.
bool charProcessFlag, blankFlag;
int currentTime, interElementGap, noSignalTimeStamp;
char *bigMorseCodeTree = (char *)"-EISH5--4--V---3--UF--------?-2--ARL---------.--.WP------J---1--TNDB6--.--X/-----KC------Y------MGZ7----,Q------O-8------9--0----";
//                                012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678
//                                         10        20        30        40        50        60        70        80        90       100       110       120

void DoCWDecoding(int audioValue) {
  static long oldTime = millis();

  switch (decodeStates) {
    
    case state0:                                                // State 0.  Detects start of signal and starts timer.
      // Detect signal and redirect to appropriate state.
      if (audioValue == 1) {
        signalStart = millis();                                                                              // Time stamp beginning of signal.
        decodeStates = state1;                                                                               // Go to "signalStart" state.
        gapLength = signalStart - signalEnd;                                                                 // Calculate the time gap between the start of this new signal and the end of the last one.
        if (gapLength > LOWEST_ATOM_TIME                                                                     // range
            && (uint32_t)gapLength < (uint32_t)(thresholdGeometricMean * 3)
            && signalStart - oldTime > 5000L) {                                                              // Only call histogram every 5 seconds
          DoGapHistogram(gapLength);                                                                         // Map the gap in the signal
          oldTime = signalStart;                                                                             // Reset old time
        }
        break;
      }
      noSignalTimeStamp = millis();
      interElementGap = noSignalTimeStamp - signalEnd;
      if ((interElementGap > ditLength * 1.95) && charProcessFlag) {  // use thresholdGeometricMean??? was ditLength. End of character!  65 * 2
        decodeStates = state5;                                        // Character ended, print it!
        break;
      }
      if (interElementGap > ditLength * 4.5 && not blankFlag && not charProcessFlag) {  // A big gap, print a blank, but don't repeat a blank.  85 * 3.5
        decodeStates = state6;
        break;
      }
      decodeStates = state0;  // Stay in state0; no signal.
      break;                                                                                                 // End state0

    case state1:                                                // Times a signal and measures its duration.  The next state determines if the signal is a dit or a dah.
      if (audioValue == 0) {
        currentTime       = millis();
        signalElapsedTime = currentTime - signalStart;          // Calculate the duration of the signal.
//                                                                 Ignore short noisy signal bursts:
        if (signalElapsedTime < LOWEST_ATOM_TIME) {             // A hiccup or a real signal?  Make this a fraction of ditLength instead???
          decodeStates    = state0;                             // False signal, start over.
          break;
        }
        if (signalElapsedTime > LOWEST_ATOM_TIME                // Valid elapsed time?
            && signalElapsedTime < HISTOGRAM_ELEMENTS
            && currentTime - oldTime > 5000L) {                 // Only call histogram every 5 seconds
          DoSignalHistogram(signalElapsedTime);                 // Yep
          oldTime = currentTime;                                // Reset old time
        }
        signalEnd         = currentTime;                        // Time gap to next signal.
        decodeStates      = state2;                             // Proceed to state2.  A timed signal is available and must be processed.
        break;
      }
      decodeStates = state1;  // Signal still present, stay in state1.
      break;                  // End state1

    case state2:                                                // Determine if a timed signal was a dit or a dah and increment the decode tree.
      if (signalElapsedTime > (0.5 * ditLength)) {              // Use the geometric mean instead of ditLength???
        currentDashJump = currentDashJump >> 1;                 // Fast divide by 2
        if (signalElapsedTime < (int)thresholdGeometricMean) {  // It was a dit
          charProcessFlag = true;                               
          currentDecoderIndex++;
        } else {  // It's a dah!
          charProcessFlag = true;
          currentDecoderIndex += currentDashJump;
        }
      }
      decodeStates = state0;  // Begin process again.
      break;                  // End state2

    case state5:                                                // Display the character
      MorseCharacterDisplay(bigMorseCodeTree[currentDecoderIndex]);  // This always prints.  How do blanks get printed.
      currentDecoderIndex = 0;                                       //Reset everything if char or word
      currentDashJump     = DECODER_BUFFER_SIZE;
      charProcessFlag     = false;  // Char printed and no longer in progress.
      decodeStates        = state0;    // Start process for next incoming character.
      blankFlag           = false;
      break;                                                    // End state5

    case state6:                                                //  Blank printing state.
      MorseCharacterDisplay(' ');

      tft.setFontScale((enum RA8875tsize)0);                    // Show estimated WPM
      tft.setTextColor(RA8875_GREEN);
      tft.fillRect(DECODER_X + 104, DECODER_Y, tft.getFontWidth() * 10, tft.getFontHeight(), RA8875_BLACK);
      tft.setCursor(DECODER_X + 105, DECODER_Y);
      tft.print("(");
      tft.print(1200L / (dahLength / 3));
      tft.print(" WPM)");
      tft.setTextColor(RA8875_WHITE);
      tft.setFontScale((enum RA8875tsize)3);

      blankFlag = true;
      decodeStates = state0;  // Start process for next incoming character.
      break;

    default:
      break;
  }
}


/*
void DoCWDecoding(int audioValue) {

  if (audioValue == 1 && signalStart == 0L) {  // This is the start of the signal
    signalStart = millis();
    gapEnd = signalStart;           // Must be at noise end
    gapLength = gapEnd - gapStart;  // How long was the gap between signals?

    if (gapLength > LOWEST_ATOM_TIME && (uint32_t)gapLength < (uint32_t)(thresholdGeometricMean * 3)) {  // range
      DoGapHistogram(gapLength);                                                                         // Map the gap in the signal
    }
    signalEnd = 0L;          // Allows us to know timing started, but not ended
    signalElapsedTime = 0L;  // Signal is just starting
    gapStart = 0L;           // Reset noise measures
  }

  if (audioValue == 0 && signalStart != 0L) {     // Has signal has just ended?
    signalEnd = millis();                         // Yep, mark end of signal, but also...
    gapStart = signalEnd;                         // ...mark the start of the gap
    signalElapsedTime = signalEnd - signalStart;  // How long was signal on?

    if (signalElapsedTime < LOWEST_ATOM_TIME) {  // A hiccup or a real signal?
      signalElapsedTime = 0L;
    }
    signalStart = 0L;
    if (signalElapsedTime > LOWEST_ATOM_TIME && signalElapsedTime < HISTOGRAM_ELEMENTS) {  // Valid elapsed time?
      DoSignalHistogram(signalElapsedTime);                                                //Yep
    }

    if (gapLength > ditLength * 1.95) {  // Is a char done??
      MorseCharacterDisplay(bigMorseCodeTree[currentDecoderIndex]);
      if (gapLength > ditLength * 4.5) {  // good over 15WPM on W1AW; no Fransworth
        MorseCharacterDisplay(' ');
        tft.setFontScale((enum RA8875tsize)0);  // Show estimated WPM
        tft.setTextColor(RA8875_GREEN);
        tft.fillRect(DECODER_X + 104, DECODER_Y, tft.getFontWidth() * 10, tft.getFontHeight(), RA8875_BLACK);
        tft.setCursor(DECODER_X + 105, DECODER_Y);
        tft.print("(");
        tft.print(1200L / (dahLength / 3));
        tft.print(" WPM)");
        tft.setTextColor(RA8875_WHITE);
        tft.setFontScale((enum RA8875tsize)3);
      }
      currentDecoderIndex = 0;  //Reset everything if char or word
      currentDashJump = DECODER_BUFFER_SIZE;
      gapLength = 0L;
    }

    //================
    if (signalElapsedTime > (0.5 * ditLength) && signalElapsedTime < (1.5 * dahLength)) {  // If not a char delimiter
      currentDashJump = currentDashJump >> 1;                                              // Fast divide by 2
      if (signalElapsedTime < thresholdGeometricMean) {                                    // It was a dit
        currentDecoderIndex++;
      } else {
        if (signalElapsedTime > thresholdGeometricMean && signalStart == 0L) {  // It was a dah
          currentDecoderIndex += currentDashJump;
        }
      }
    }
  }
}
*/

/*****
  Purpose: This function creates a distribution of the gaps between signals, expressed
           in milliseconds. The result is a tri-modal distribution around three timings:
            1. inter-atom time (one dit length)
            2. inter-character (three dit lengths)
            3. word end (seven dit lengths)

  Parameter list:
    long val        the duration of the signal gap (ms)

  Return value;
    void

*****/
void DoGapHistogram(long gapLen) {
  int32_t tempAtom, tempChar;
  int32_t atomIndex, charIndex, firstDit, temp;
  uint32_t offset;

  if (gapHistogram[gapLen] > 10) {  // Need over 1 so we don't have fractional value
    for (int k = 0; k < HISTOGRAM_ELEMENTS; k++) {
      gapHistogram[k] = (uint32_t)(.8 * (float)gapHistogram[k]);
    }
  }

  gapHistogram[gapLen]++;  // Add new signal to distribution

  atomIndex = charIndex = 0;
  if (gapLen <= thresholdGeometricMean) {                                                                                 // Find new dit length
    JackClusteredArrayMax(gapHistogram, (uint32_t)thresholdGeometricMean, &tempAtom, &atomIndex, &firstDit, (int32_t)1);  // Find max dit gap
    if (atomIndex) {                                                                                                      // if something found
      gapAtom = atomIndex;
    }
    for (int j = 0; j < HISTOGRAM_ELEMENTS; j++) {                        // count down
      if (gapHistogram[HISTOGRAM_ELEMENTS - j] > 0 && endGapFlag == 0) {  //Look for non-zero entries in the histogram
        if (HISTOGRAM_ELEMENTS - j < gapAtom * 2) {                       // limit search to probable gapAtom entries
          topGapIndex = HISTOGRAM_ELEMENTS - j;                           //Upper end of gapAtom range
          endGapFlag = 1;                                                 // set flag so we know tha this is the top of the gapAtom range
        }
      }
      if (topGapIndex > 2 * gapAtom) topGapIndex = topGapIndexOld;  // discard outliers
    }
    endGapFlag = 0;                //reset flag
    topGapIndexOld = topGapIndex;  //Keep good value for reference
  } else {                         // dah calculation
    if (gapLen <= thresholdGeometricMean * 2) {
      offset = (uint32_t)(thresholdGeometricMean * 2);  // Find number of elements to check
      JackClusteredArrayMax(&gapHistogram[(int32_t)thresholdGeometricMean + 1], offset, &tempChar, &charIndex, &temp, (int32_t)3);
      if (charIndex)  // if something found
        gapChar = charIndex;
    }
  }
  if (atomIndex) {
    gapAtom = atomIndex;
  }
  if (charIndex) {
    gapChar = charIndex;
  }
}

/*****
  Purpose: This function replaces the arm_max_float32() function that finds the maximum element in an array.
           The histograms are "fuzzy" in the sense that dits and dahs "cluster" around a maximum value rather
           than having a single max value. This algorithm looks at a given cell and the adds in the previous
           (index - 1) and next (index + 1) cells to get the total for that index.

  Parameter list:
    int32_t *array         the base address of the array to search
    int32_t elements       the number of elements of the array to examine
    int32_t *maxCount      the largest clustered value found
    int32_t *maxIndex      the index of the center of the cluster
    int32_t *firstNonZero  the first cell that has a non-zero value
    int32_t clusterSpread  tells how far previous and ahead elements are to be included in the measure.
                            Must be an odd integer > 1.

  Return value;
    void
*****/
void JackClusteredArrayMax(int32_t *array, int32_t elements, int32_t *maxCount, int32_t *maxIndex, int32_t *firstNonZero, int32_t spread) {
  int32_t i, j, clusteredIndex;
  int32_t clusteredMax, temp;

  *maxCount = '\0';  // Reset to empty
  *maxIndex = '\0';

  clusteredMax = 0;
  clusteredIndex = -1;  // Now we can check for an error

  for (i = spread; i < elements - spread; i++) {  // Start with 1 so we can look at the previous element's value
    temp = 0;
    for (j = i - spread; j <= i + spread; j++) {
      temp += array[j];
      ;  // Include adjacent elements
    }

    if (temp >= clusteredMax) {
      clusteredMax = temp;
      clusteredIndex = i;
    }
  }
  if (clusteredIndex > 0) {
    *maxCount = array[clusteredIndex];
    *maxIndex = clusteredIndex;
  }
}
/*****
  Purpose: This function creates a distribution of the dit and dahs lengths, expressed in
  milliseconds. The result is a bi-modal distribution around those two timings. The
  modal value is then used for the timing of the decoder. The range should be between 20
  (60wpm) and 240 (5wpm)

  Parameter list:
  long val        the strength of audio signal

  Return value;
  void

*****/
void DoSignalHistogram(long val) {
  float compareFactor = 2.0;
  int32_t firstNonEmpty;
  int32_t tempDit, tempDah;
  int32_t offset;

  if (valFlag == 0) {
    valRef1 = signalElapsedTime;
    signalStartOld = millis();
    valFlag = 1;
  }

  if (millis() - signalStartOld > LOWEST_ATOM_TIME && valFlag == 1) {
    gapRef1 = gapLength;
    valRef2 = signalElapsedTime;
    valFlag = 0;
  }

  if ((valRef2 >= valRef1 * compareFactor && gapRef1 <= valRef1 * compareFactor)
      || (valRef1 >= valRef2 * compareFactor && gapRef1 <= valRef2 * compareFactor)) {
    // See if consecutive signal lengths in approximate dit to dah ratio and which one is larger
    if (valRef2 >= valRef1) {
      aveDitLength = (long)(0.9 * aveDitLength + 0.1 * valRef1);  //Do some dit length averaging
      aveDahLength = (long)(0.9 * aveDahLength + 0.1 * valRef2);
    } else {
      aveDitLength = (long)(0.9 * aveDitLength + 0.1 * valRef2);  // Use larger one. Note reversal of calc order
      aveDahLength = (long)(0.9 * aveDahLength + 0.1 * valRef1);  // Do some dah length averaging
    }
  }
  thresholdGeometricMean = sqrt(aveDitLength * aveDahLength);    //calculate geometric mean
  thresholdArithmeticMean = (aveDitLength + aveDahLength) >> 1;  // Fast divide by 2 on integer data

  signalHistogram[val]++;  // Don't care which half it's in, just put it in

  offset = (uint32_t)thresholdGeometricMean - 1;  // Only do cast once
  // Dit calculation
  // 2nd parameter means we only look for dits below the geomean.

  for (int32_t j = (int32_t)thresholdGeometricMean; j; j--) {
    if (signalHistogram[j] != 0) {
      firstNonEmpty = j;
      break;
    }
  }

  JackClusteredArrayMax(signalHistogram, offset, &tempDit, (int32_t *)&ditLength, &firstNonEmpty, (int32_t)1);
  // dah calculation
  // Elements above the geomean. Note larger spread: higher variance
  JackClusteredArrayMax(&signalHistogram[offset], HISTOGRAM_ELEMENTS - offset, &tempDah, (int32_t *)&dahLength, &firstNonEmpty, (uint32_t)3);
  dahLength += (uint32_t)offset;

  if (tempDit > SCALE_CONSTANT && tempDah > SCALE_CONSTANT) {  //Adaptive dit signalHistogram[]
    for (int k = 0; k < HISTOGRAM_ELEMENTS; k++) {
      signalHistogram[k] = ADAPTIVE_SCALE_FACTOR * signalHistogram[k];
    }
  }
}

/*****
  Purpose: Calculate Goertzal Algorithn to enable decoding CW

  Parameter list:
    int numSamples,         // number of sample in data array
    int TARGET_FREQUENCY,   // frequency for which the magnitude of the transform is to be found
    int SAMPLING_RATE,      // Sampling rate in our case 24ksps
    float* data             // pointer to input data array

  Return value;
    float magnitude     //magnitude of the transform at the target frequency

*****/
float goertzel_mag(int numSamples, int TARGET_FREQUENCY, int SAMPLING_RATE, float *data) {
  int k, i;
  float floatnumSamples;
  float omega, sine, cosine, coeff, q0, q1, q2, magnitude, real, imag;

  float scalingFactor = numSamples / 2.0;

  floatnumSamples = (float)numSamples;
  k = (int)(0.5 + ((floatnumSamples * TARGET_FREQUENCY) / SAMPLING_RATE));
  omega = (2.0 * M_PI * k) / floatnumSamples;
  sine = sin(omega);
  cosine = cos(omega);
  coeff = 2.0 * cosine;
  q0 = 0;
  q1 = 0;
  q2 = 0;

  for (i = 0; i < numSamples; i++) {
    q0 = coeff * q1 - q2 + data[i];
    q2 = q1;
    q1 = q0;
  }
  real = (q1 - q2 * cosine) / scalingFactor;  // calculate the real and imaginary results scaling appropriately
  imag = (q2 * sine) / scalingFactor;

  magnitude = sqrtf(real * real + imag * imag);
  return magnitude;
}

/*****
  Purpose:Display horizontal CW Decode level

  Parameter list:
    void

  Return value;
    void

*****/
void CW_DecodeLevelDisplay() {
  int levelMtrOffset = 120;

  // draw S-Meter layout
  tft.drawFastHLine(SMETER_X - levelMtrOffset, SMETER_Y - 1, 100, RA8875_WHITE);
  tft.drawFastHLine(SMETER_X - levelMtrOffset, SMETER_Y + 20, 100, RA8875_WHITE);  // changed 6 to 20

  tft.drawFastVLine(SMETER_X - levelMtrOffset, SMETER_Y - 1, 20, RA8875_WHITE);  // charge 8 to 18
  tft.drawFastVLine(SMETER_X + 100 - levelMtrOffset, SMETER_Y - 1, 20, RA8875_WHITE);
}

/*****
  Purpose: Select CW tone frequency.

  Parameter list:
    void

  Return value:
    void
*****/
FLASHMEM void SelectCWToneOffset() 
{
  const char *CWToneOffsets[] = {"0 Hz", "562.5 Hz", "656.5 Hz", "750 Hz", "843.75 Hz", " Cancel " };
//  const int numCycles[4] = { 6, 7, 8, 9 };

  int val;
  CWToneIndex = EEPROMData.CWToneIndex;
//  long lastoneIndex = 0;

  filterEncoderMove = 0;
  CWToneIndex = 0;

  tft.setFontScale((enum RA8875tsize)1);

  tft.fillRect(SECONDARY_MENU_X, MENUS_Y, EACH_MENU_WIDTH, CHAR_HEIGHT, RA8875_MAGENTA);
  tft.setTextColor(RA8875_WHITE);
  tft.setCursor(SECONDARY_MENU_X + 1, MENUS_Y + 1);
//  tft.print("current index:");
//  tft.setCursor(SECONDARY_MENU_X + 200, MENUS_Y + 1);
  tft.print(CWToneOffsets[filterEncoderMove]);

  while (true) {
    if (filterEncoderMove != 0) {           // Changed encoder?
      CWToneIndex += filterEncoderMove;   // Yep
//      lastoneIndex = CWToneIndex;
      if (CWToneIndex < 0) {
        CWToneIndex = 5;
      } else { 
        if (CWToneIndex > 5)
          CWToneIndex = 0;
      }
      tft.fillRect(SECONDARY_MENU_X, MENUS_Y + 1, EACH_MENU_WIDTH, CHAR_HEIGHT, RA8875_MAGENTA);
      tft.setCursor(SECONDARY_MENU_X + 1, MENUS_Y + 1);
      tft.print(CWToneOffsets[CWToneIndex]);
      filterEncoderMove = 0;
    }

    val = ReadSelectedPushButton();  // Read pin that controls all switches
    val = ProcessButtonPress(val);                                            ///////////////////////////////////////////////////
    if (val == MENU_OPTION_SELECT) {  // Make a choice??
      if (CWToneIndex == 4)               // Cancel? Don't update index
        return;

//      CWToneIndex = lastFilter;
      EEPROMData.CWToneIndex = CWToneIndex;
      EEPROMWrite();
      UpdateWPMField();
      break;
    }
  }
  tft.setTextColor(RA8875_WHITE);
//  EraseMenus();
  return;
/*  
  EEPROMData.CWToneOffset = SubmenuSelect(CWToneOffsets, 5, EEPROMData.CWToneOffset);  // CWFilter is an array of strings.
  // Now generate the values for the buffer which is used to create the CW tone.  The values are discrete because there must be whole cycles.
  if (EEPROMData.CWToneOffset < 4) sineTone(numCycles[EEPROMData.CWToneOffset]);
  EEPROMWrite();  // Save to EEPROM.
  // Clear the current CW filter graphics and then restore the bandwidth indicator bar.  KF5N July 30, 2023
  tft.writeTo(L2);
  tft.clearMemory();
  RedrawDisplayScreen();
*/  
}
