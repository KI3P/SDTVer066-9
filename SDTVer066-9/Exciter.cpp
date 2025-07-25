#ifndef BEENHERE
#include "SDT.h"
#endif

/*****
  Purpose: Create I and Q signals from Mic input

  Parameter list:

  Return value;
    void
    Notes:
    There are several actions in this function
    1.  Read in the data from the ADC into the Left Channel at 192KHz
    2.  Format the L data and Decimate (downsample and filter)the sampled data by x8
          - the new effective sampling rate is now 24KHz
    3.  Process the L data through the 7 EQ filters and combine to a single data stream
    4.  Copy the L channel to the R channel
    5.  Process the R and L through two Hilbert Transformers - L 0deg phase shift and R 90 deg ph shift
          - This create the I (L) and Q(R) channels
    6.  Interpolate 8x (upsample and filter) the data stream to 192KHz sample rate
    7.  Output the data stream thruogh the DACs at 192KHz
*****/
void ExciterIQData() {
  uint32_t N_BLOCKS_EX = N_B_EX;
  int SSB_CalModeTask = 0;

  /**********************************************************************************  AFP 12-31-20
        Get samples from queue buffers
        Teensy Audio Library stores ADC data in two buffers size=128, Q_in_L and Q_in_R as initiated from the audio lib.
        Then the buffers are  read into two arrays sp_L and sp_R in blocks of 128 up to N_BLOCKS.  The arrarys are
        of size BUFFER_SIZE*N_BLOCKS.  BUFFER_SIZE is 128.
        N_BLOCKS = FFT_L / 2 / BUFFER_SIZE * (uint32_t)DF; // should be 16 with DF == 8 and FFT_L = 512
        BUFFER_SIZE*N_BLOCKS = 2024 samples
     **********************************************************************************/
  // are there at least N_BLOCKS buffers in each channel available ?
  if ((uint32_t)Q_in_L_Ex.available() > N_BLOCKS_EX + 0 && (uint32_t)Q_in_R_Ex.available() > N_BLOCKS_EX + 0) 
  {

    // get audio samples from the audio  buffers and convert them to float
    // read in 32 blocks á 128 samples in I and Q
    for (unsigned i = 0; i < N_BLOCKS_EX; i++) {
      sp_L2 = Q_in_L_Ex.readBuffer();
      sp_R2 = Q_in_R_Ex.readBuffer();

      /**********************************************************************************  AFP 12-31-20
          Using arm_Math library, convert to float one buffer_size.
          Float_buffer samples are now standardized from > -1.0 to < 1.0
      **********************************************************************************/
      arm_q15_to_float(sp_L2, &float_buffer_L_EX[BUFFER_SIZE * i], BUFFER_SIZE);  // convert int_buffer to float 32bit
      arm_q15_to_float(sp_R2, &float_buffer_R_EX[BUFFER_SIZE * i], BUFFER_SIZE);  // convert int_buffer to float 32bit
      Q_in_L_Ex.freeBuffer();
      Q_in_R_Ex.freeBuffer();
    }
    /**********************************************************************************  AFP 12-31-20
              Decimation is the process of downsampling the data stream and LP filtering
              Decimation is done in two stages to prevent reversal of the spectrum, which occure with each even
              Decimation.  First select every 4th asmple and then every 2nd sample, yielding 8x downsampling
              192KHz/8 = 24KHz, with 8xsmaller sample sizes
     **********************************************************************************/

    // 192KHz effective sample rate here
    // decimation-by-4 in-place!
    arm_fir_decimate_f32(&FIR_dec1_EX_I, float_buffer_L_EX, float_buffer_L_EX, BUFFER_SIZE * N_BLOCKS_EX);
    arm_fir_decimate_f32(&FIR_dec1_EX_Q, float_buffer_R_EX, float_buffer_R_EX, BUFFER_SIZE * N_BLOCKS_EX);
    // 48KHz effective sample rate here
    // decimation-by-2 in-place
    arm_fir_decimate_f32(&FIR_dec2_EX_I, float_buffer_L_EX, float_buffer_L_EX, 512);
    arm_fir_decimate_f32(&FIR_dec2_EX_Q, float_buffer_R_EX, float_buffer_R_EX, 512);
    // 24KSPS effective sample rate here
    // Set up for calibration routines
    if (twoToneFlag == 0 && IQCalFlag == 0 && SSB_PA_CalFlag == 0) SSB_CalModeTask = 0;  // Regular operation
    if (twoToneFlag == 1 && IQCalFlag == 0 && SSB_PA_CalFlag == 0) SSB_CalModeTask = 1;  // Two-tone Calibrate
    if (twoToneFlag == 0 && IQCalFlag == 1 && SSB_PA_CalFlag == 0) SSB_CalModeTask = 2;  //Transmit IQ calibration
    if (twoToneFlag == 0 && IQCalFlag == 0 && SSB_PA_CalFlag == 1) SSB_CalModeTask = 2;  //SSB initial Power calibrate as reference

   switch (SSB_CalModeTask) {
      //  ============= AFP 02-01-25
      case (0):  // Passthrough
        break;

      case (1):  // Two-tone signal generation - uses Hilbert transfor to generate IQ signals
        arm_add_f32(sinBuffer4, sinBuffer5, float_buffer_L_EX, 256);
        arm_add_f32(sinBuffer4, sinBuffer5, float_buffer_R_EX, 256);
        arm_scale_f32(float_buffer_L_EX, .1, float_buffer_L_EX, 256);
        arm_scale_f32(float_buffer_R_EX, .1, float_buffer_R_EX, 256);
        break;

      case (2):  // //Sine wave generator for transmit IQ Calibrate and Transmit SSB power calibrate
        arm_scale_f32(sinBuffer6, .03, float_buffer_L_EX, 256);
        arm_scale_f32(sinBuffer6, .03, float_buffer_R_EX, 256);
        break;
    }
   arm_scale_f32(float_buffer_L_EX, (float)XAttenSSB[currentBand] / 10, float_buffer_L_EX, 256);
    arm_scale_f32(float_buffer_R_EX, (float)XAttenSSB[currentBand] / 10, float_buffer_R_EX, 256);

    //============================  Transmit EQ  ========================  AFP 10-02-22
    if (xmitEQFlag == ON) {
      DoExciterEQ();
    }
    //============================ End Receive EQ  AFP 10-02-22
    // Filter the trasmit audio
    //arm_biquad_cascade_df1_f32(&biquadTXAudioLowPass, float_buffer_L_EX, 
    //                           float_buffer_R_EX, 256);
    //arm_copy_f32(float_buffer_R_EX, float_buffer_L_EX, 256); 
    arm_copy_f32(float_buffer_L_EX, float_buffer_R_EX, 256); 
    //  The following converts th sample rate to 12K SPS, applies the Hilbert transforms and then interpolates back to 24K SP.
    //The objective is to extend the lower frequency rnge of the Hilbert transfrom by moving the lowewr limit of the Hilbert usefullness down to below 200Hz.

    //Decimate by 2 to 12K SPS sample rate
    arm_fir_decimate_f32(&FIR_dec3_EX_I, float_buffer_L_EX, float_buffer_L_EX, 256);
    arm_fir_decimate_f32(&FIR_dec3_EX_Q, float_buffer_R_EX, float_buffer_R_EX, 256);
    //Hilbert transforms at 12K, with 5KHz bandwidth, buffer size 128
    arm_fir_f32(&FIR_Hilbert_L, float_buffer_L_EX, float_buffer_L_EX, 128);
    arm_fir_f32(&FIR_Hilbert_R, float_buffer_R_EX, float_buffer_R_EX, 128);
    //Interpolate back to 24K SPS
    arm_fir_interpolate_f32(&FIR_int3_EX_I, float_buffer_L_EX, float_buffer_LTemp, 128);
    arm_fir_interpolate_f32(&FIR_int3_EX_Q, float_buffer_R_EX, float_buffer_RTemp, 128);
    //Back to 24000 SPS
    //Scale to equalize levels
    arm_scale_f32(float_buffer_LTemp, 3.5, float_buffer_L_EX, 256);
    arm_scale_f32(float_buffer_RTemp, 3.5, float_buffer_R_EX, 256);


    /**********************************************************************************
             R and L channels are processed though the two Hilbert Transformers, L at 0 deg and R at 90 deg
             Tthe result are the quadrature data streans, I and Q necessary for Phasing calculations to
             create the SSB signals.
             Two Hilbert Transformers are used to preserve eliminate the relative time delays created during processing of the data
    **********************************************************************************/

    //==== Uising Vol and Filter encoders afjust IQ calibration factors
    if (twoToneFlag == 0 && IQCalFlag == 1 && SSB_PA_CalFlag == 0) {
      IQXAmpCorrectionFactor[currentBand] = GetEncoderValueLiveXCal(-2.0, 2.0, IQXAmpCorrectionFactor[currentBand], correctionIncrement, (char *)"IQ Gain", 3, IQEXChoice);
      IQXPhaseCorrectionFactor[currentBand] = GetEncoderValueLiveXCal2(-2.0, 2.0, IQXPhaseCorrectionFactor[currentBand], correctionIncrement, (char *)"IQ Phase", 3, IQEXChoice);
    }
    // ==== Apply IQ calibration factors
    if (bands[currentBand].mode == DEMOD_LSB) {                                                        //AFP 12-27-21
      arm_scale_f32(float_buffer_L_EX, IQXAmpCorrectionFactor[currentBandA], float_buffer_L_EX, 256);  // Flip SSB sideband KF5N, minus sign was original
      //arm_scale_f32 (float_buffer_L_EX, 1.0, float_buffer_L_EX, 256);     // Flip SSB sideband KF5N, minus sign was original
      IQXPhaseCorrection(float_buffer_L_EX, float_buffer_R_EX, IQXPhaseCorrectionFactor[currentBandA], 256);
      //IQPhaseCorrection(float_buffer_L_EX, float_buffer_R_EX, 0.0, 256);
    } else if (bands[currentBand].mode == DEMOD_USB) {                                                  //AFP 12-27-21
      arm_scale_f32(float_buffer_L_EX, -IQXAmpCorrectionFactor[currentBandA], float_buffer_L_EX, 256);  // Flip SSB sideband KF5N
      IQPhaseCorrection(float_buffer_L_EX, float_buffer_R_EX, IQXPhaseCorrectionFactor[currentBandA], 256);
    }
    arm_scale_f32(float_buffer_R_EX, 1.00, float_buffer_R_EX, 256);

    //exciteMaxL = 0;

    /**********************************************************************************
              Interpolate (upsample the data streams by 8X to create the 192KHx sample rate for output
              Requires a LPF FIR 48 tap 10KHz and 8KHz
     **********************************************************************************/
    //24KHz effective sample rate here
    arm_fir_interpolate_f32(&FIR_int1_EX_I, float_buffer_L_EX, float_buffer_LTemp, 256);
    arm_fir_interpolate_f32(&FIR_int1_EX_Q, float_buffer_R_EX, float_buffer_RTemp, 256);

    // interpolation-by-4,  48KHz effective sample rate here
    arm_fir_interpolate_f32(&FIR_int2_EX_I, float_buffer_LTemp, float_buffer_L_EX, 512);
    arm_fir_interpolate_f32(&FIR_int2_EX_Q, float_buffer_RTemp, float_buffer_R_EX, 512);
    //  192KHz effective sample rate here
    arm_scale_f32(float_buffer_L_EX, 20, float_buffer_L_EX, 2048);  //Scale to compensate for losses in Interpolation
    arm_scale_f32(float_buffer_R_EX, 20, float_buffer_R_EX, 2048);

    /**********************************************************************************  AFP 12-31-20
      CONVERT TO INTEGER AND PLAY AUDIO
    **********************************************************************************/

    for (unsigned i = 0; i < N_BLOCKS_EX; i++) {  //N_BLOCKS_EX=16  BUFFER_SIZE=128 16x128=2048
      sp_L2 = Q_out_L_Ex.getBuffer();
      sp_R2 = Q_out_R_Ex.getBuffer();
      arm_float_to_q15(&float_buffer_L_EX[BUFFER_SIZE * i], sp_L2, BUFFER_SIZE);
      arm_float_to_q15(&float_buffer_R_EX[BUFFER_SIZE * i], sp_R2, BUFFER_SIZE);
      Q_out_L_Ex.playBuffer();  // play it !
      Q_out_R_Ex.playBuffer();  // play it !
     
    }
  }
  
}


/*****
  Purpose: Set the current band relay ON or OFF

  Parameter list:
    int state             OFF = 0, ON = 1

  Return value;
    void
*****/
void SetBandRelay(int state) {
  // There are 4 physical relays.  Turn all of them off.
  for (int i = 0; i < 4; i = i + 1) {
    digitalWrite(bandswitchPins[i], LOW);  // Set ALL band relays low.  KF5N July 21, 2023
  }
  // Set current band relay "on".  Ignore 12M and 10M.  15M and 17M use the same relay.  KF5N September 27, 2023.
  if (currentBand < 5) digitalWrite(bandswitchPins[currentBand], state);
}
