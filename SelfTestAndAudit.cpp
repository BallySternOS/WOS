/**************************************************************************
 *     This file is part of the Bally/Stern OS for Arduino Project.

    I, Dick Hamill, the author of this program disclaim all copyright
    in order to make this program freely available in perpetuity to
    anyone who would like to use it. Dick Hamill, 6/1/2020

    BallySternOS is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    BallySternOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    See <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include "SelfTestAndAudit.h"
#include "WOS_Config.h"
#include "WOS.h"

#define MACHINE_STATE_ATTRACT         0
//#define USE_SB100

unsigned long LastSolTestTime = 0; 
unsigned long LastSelfTestChange = 0;
unsigned long SavedValue = 0;
unsigned long ResetHold = 0;
unsigned long NextSpeedyValueChange = 0;
unsigned long NumSpeedyChanges = 0;
unsigned long LastResetPress = 0;
byte CurValue = 0;
byte CurSound = 0x01;
byte SoundPlaying = 0;
boolean SolenoidCycle = true;


int RunBaseSelfTest(int curState, boolean curStateChanged, unsigned long CurrentTime, byte resetSwitch, byte slamSwitch) {
  byte curSwitch = WOS_PullFirstFromSwitchStack();
  int returnState = curState;
  boolean resetDoubleClick = false;
  unsigned short savedScoreStartByte = 0;
  unsigned short auditNumStartByte = 0;

  if (curSwitch==resetSwitch) {
    ResetHold = CurrentTime;
    if ((CurrentTime-LastResetPress)<400) {
      resetDoubleClick = true;
      curSwitch = SWITCH_STACK_EMPTY;
    }
    LastResetPress = CurrentTime;
  }

  if (ResetHold!=0 && !WOS_ReadSingleSwitchState(resetSwitch)) {
    ResetHold = 0;
    NextSpeedyValueChange = 0;
  }

  boolean resetBeingHeld = false;
  if (ResetHold!=0 && (CurrentTime-ResetHold)>1300) {
    resetBeingHeld = true;
    if (NextSpeedyValueChange==0) {
      NextSpeedyValueChange = CurrentTime;
      NumSpeedyChanges = 0;
    }
  }

  if (slamSwitch!=0xFF && curSwitch==slamSwitch) {
    returnState = MACHINE_STATE_ATTRACT;
  }
  
  if (curSwitch==SW_SELF_TEST_SWITCH && (CurrentTime-LastSelfTestChange)>250) {
    returnState -= 1;
    if (returnState==MACHINE_STATE_TEST_DONE) returnState = MACHINE_STATE_ATTRACT;
    LastSelfTestChange = CurrentTime;
  }

  if (curStateChanged) {
    WOS_SetCoinLockout(false);
    
    for (int count=0; count<4; count++) {
      WOS_SetDisplay(count, 0);
      WOS_SetDisplayBlank(count, 0x00);        
    }

    if (curState<=MACHINE_STATE_TEST_SCORE_LEVEL_1) {
//      WOS_SetDisplayCredits(abs(curState)+MACHINE_STATE_TEST_SOUNDS);
      WOS_SetDisplayCredits(MACHINE_STATE_TEST_SOUNDS-curState);
      WOS_SetDisplayBallInPlay(0, false);
    }
  }

  if (curState==MACHINE_STATE_TEST_LIGHTS) {
    if (curStateChanged) {
      WOS_DisableSolenoidStack();        
      WOS_SetDisableFlippers(true);
      WOS_SetDisplayCredits(0);
      WOS_SetDisplayBallInPlay(1);
      WOS_TurnOffAllLamps();
      for (int count=0; count<WOS_MAX_LAMPS; count++) {
        WOS_SetLampState(count, 1, 0, 500);
      }
      CurValue = 99;
      WOS_SetDisplay(0, CurValue, true);  
    }
    if (curSwitch==resetSwitch || resetDoubleClick) {
      CurValue += 1;
      if (CurValue>99) CurValue = 0;
      if (CurValue==WOS_MAX_LAMPS) {
        CurValue = 99;
        for (int count=0; count<WOS_MAX_LAMPS; count++) {
          WOS_SetLampState(count, 1, 0, 500);
        }
      } else {
        WOS_TurnOffAllLamps();
        WOS_SetLampState(CurValue, 1, 0, 500);
      }      
      WOS_SetDisplay(0, CurValue, true);  
    }    
  } else if (curState==MACHINE_STATE_TEST_DISPLAYS) {
    if (curStateChanged) {
      WOS_TurnOffAllLamps();
      WOS_SetDisplayCredits(0);
      WOS_SetDisplayBallInPlay(2);
      for (int count=0; count<4; count++) {
        WOS_SetDisplayBlank(count, 0x3F);        
      }
      CurValue = 0;
    }
    if (curSwitch==resetSwitch || resetDoubleClick) {
      CurValue += 1;
      if (CurValue>30) CurValue = 0;
    }    
    WOS_CycleAllDisplays(CurrentTime, CurValue);
  } else if (curState==MACHINE_STATE_TEST_SOLENOIDS) {
    if (curStateChanged) {
      WOS_TurnOffAllLamps();
      LastSolTestTime = CurrentTime;
      WOS_EnableSolenoidStack(); 
      WOS_SetDisableFlippers(false);
      WOS_SetDisplayBlank(4, 0);
      WOS_SetDisplayCredits(0);
      WOS_SetDisplayBallInPlay(3);
      SolenoidCycle = true;
      SavedValue = 0;
      WOS_PushToSolenoidStack(SavedValue, 10);
    } 
    if (curSwitch==resetSwitch || resetDoubleClick) {
      SolenoidCycle = (SolenoidCycle) ? false : true;
    }

    if ((CurrentTime-LastSolTestTime)>1000) {
      if (SolenoidCycle) {
        SavedValue += 1;
        if (SavedValue>14) SavedValue = 0;
      }
      WOS_PushToSolenoidStack(SavedValue, 10);
      WOS_SetDisplay(0, SavedValue, true);
      LastSolTestTime = CurrentTime;
    }
    
  } else if (curState==MACHINE_STATE_TEST_SWITCHES) {
    if (curStateChanged) {
      WOS_TurnOffAllLamps();
      WOS_DisableSolenoidStack(); 
      WOS_SetDisableFlippers(true);
      WOS_SetDisplayCredits(0);
      WOS_SetDisplayBallInPlay(4);
    }

    byte displayOutput = 0;
    for (byte switchCount=0; switchCount<40 && displayOutput<4; switchCount++) {
      if (WOS_ReadSingleSwitchState(switchCount)) {
        WOS_SetDisplay(displayOutput, switchCount, true);
        displayOutput += 1;
      }
    }

    if (displayOutput<4) {
      for (int count=displayOutput; count<4; count++) {
        WOS_SetDisplayBlank(count, 0x00);
      }
    }

  } else if (curState==MACHINE_STATE_TEST_SOUNDS) {
    WOS_SetDisplayCredits(0);
    WOS_SetDisplayBallInPlay(5);
#ifdef USE_SB100    
    byte soundToPlay = 0x01 << (((CurrentTime-LastSelfTestChange)/750)%8);
    if (SoundPlaying!=soundToPlay) {
      WOS_PlaySB100(soundToPlay);
      SoundPlaying = soundToPlay;
      WOS_SetDisplay(0, (unsigned long)soundToPlay, true);
      LastSolTestTime = CurrentTime; // Time the sound started to play
    }
    // If the sound play call was more than 300ms ago, turn it off
//    if ((CurrentTime-LastSolTestTime)>300) WOS_PlaySB100(128);
#elif defined (BALLY_STERN_OS_USE_SQUAWK_AND_TALK)
    byte soundToPlay = ((CurrentTime-LastSelfTestChange)/2000)%256;
    if (SoundPlaying!=soundToPlay) {
      WOS_PlaySoundSquawkAndTalk(soundToPlay);
      SoundPlaying = soundToPlay;
      WOS_SetDisplay(0, (unsigned long)soundToPlay, true);
      LastSolTestTime = CurrentTime; // Time the sound started to play
    }
#elif defined (BALLY_STERN_OS_USE_DASH51) 
    byte soundToPlay = ((CurrentTime-LastSelfTestChange)/2000)%32;
    if (SoundPlaying!=soundToPlay) {
      if (soundToPlay==17) soundToPlay = 0;
      WOS_PlaySoundDash51(soundToPlay);
      SoundPlaying = soundToPlay;
      WOS_SetDisplay(0, (unsigned long)soundToPlay, true);
      LastSolTestTime = CurrentTime; // Time the sound started to play
    }
#elif defined (WILLIAMS_TYPE_1_SOUND)
    byte soundToPlay = (((CurrentTime-LastSelfTestChange)/2000)%31)+1;
    if (SoundPlaying!=soundToPlay) {
      WOS_PushToSoundStack(soundToPlay*256, 8);
      SoundPlaying = soundToPlay;
      WOS_SetDisplay(0, (unsigned long)soundToPlay, true);
      LastSolTestTime = CurrentTime; // Time the sound started to play
    }
#endif
  } else if (curState==MACHINE_STATE_TEST_SCORE_LEVEL_1) {
#ifdef USE_SB100    
    if (curStateChanged) WOS_PlaySB100(0);
#endif
    savedScoreStartByte = WOS_AWARD_SCORE_1_EEPROM_START_BYTE;
  } else if (curState==MACHINE_STATE_TEST_SCORE_LEVEL_2) {
    savedScoreStartByte = WOS_AWARD_SCORE_2_EEPROM_START_BYTE;
  } else if (curState==MACHINE_STATE_TEST_SCORE_LEVEL_3) {
    savedScoreStartByte = WOS_AWARD_SCORE_3_EEPROM_START_BYTE;
  } else if (curState==MACHINE_STATE_TEST_HISCR) {
    savedScoreStartByte = WOS_HIGHSCORE_EEPROM_START_BYTE;
  } else if (curState==MACHINE_STATE_TEST_CREDITS) {
    if (curStateChanged) {
      SavedValue = WOS_ReadByteFromEEProm(WOS_CREDITS_EEPROM_BYTE);
      WOS_SetDisplay(0, SavedValue, true);
    }
    if (curSwitch==resetSwitch || resetDoubleClick) {
      SavedValue += 1;
      if (SavedValue>20) SavedValue = 0;
      WOS_SetDisplay(0, SavedValue, true);
      WOS_WriteByteToEEProm(WOS_CREDITS_EEPROM_BYTE, SavedValue & 0x000000FF);
    }
  } else if (curState==MACHINE_STATE_TEST_TOTAL_PLAYS) {
    auditNumStartByte = WOS_TOTAL_PLAYS_EEPROM_START_BYTE;
  } else if (curState==MACHINE_STATE_TEST_TOTAL_REPLAYS) {
    auditNumStartByte = WOS_TOTAL_REPLAYS_EEPROM_START_BYTE;
  } else if (curState==MACHINE_STATE_TEST_HISCR_BEAT) {
    auditNumStartByte = WOS_TOTAL_HISCORE_BEATEN_START_BYTE;
  } else if (curState==MACHINE_STATE_TEST_CHUTE_2_COINS) {
    auditNumStartByte = WOS_CHUTE_2_COINS_START_BYTE;
  } else if (curState==MACHINE_STATE_TEST_CHUTE_1_COINS) {
    auditNumStartByte = WOS_CHUTE_1_COINS_START_BYTE;
  } else if (curState==MACHINE_STATE_TEST_CHUTE_3_COINS) {
    auditNumStartByte = WOS_CHUTE_3_COINS_START_BYTE;
  }

  if (savedScoreStartByte) {
    if (curStateChanged) {
      SavedValue = WOS_ReadULFromEEProm(savedScoreStartByte);
      WOS_SetDisplay(0, SavedValue, true);  
    }

    if (curSwitch==resetSwitch) {
      SavedValue += 1000;
      WOS_SetDisplay(0, SavedValue, true);  
      WOS_WriteULToEEProm(savedScoreStartByte, SavedValue);
    }

    if (resetBeingHeld && (CurrentTime>=NextSpeedyValueChange)) {
      SavedValue += 1000;
      WOS_SetDisplay(0, SavedValue, true);  
      if (NumSpeedyChanges<6) NextSpeedyValueChange = CurrentTime + 400;
      else if (NumSpeedyChanges<50) NextSpeedyValueChange = CurrentTime + 50;
      else NextSpeedyValueChange = CurrentTime + 10;
      NumSpeedyChanges += 1;
    }

    if (!resetBeingHeld && NumSpeedyChanges>0) {
      WOS_WriteULToEEProm(savedScoreStartByte, SavedValue);
      NumSpeedyChanges = 0;
    }
    
    if (resetDoubleClick) {
      SavedValue = 0;
      WOS_SetDisplay(0, SavedValue, true);  
      WOS_WriteULToEEProm(savedScoreStartByte, SavedValue);
    }
  }

  if (auditNumStartByte) {
    if (curStateChanged) {
      SavedValue = WOS_ReadULFromEEProm(auditNumStartByte);
      WOS_SetDisplay(0, SavedValue, true);
    }

    if (resetDoubleClick) {
      SavedValue = 0;
      WOS_SetDisplay(0, SavedValue, true);  
      WOS_WriteULToEEProm(auditNumStartByte, SavedValue);
    }
    
  }
  
  return returnState;
}

unsigned long GetLastSelfTestChangedTime() {
  return LastSelfTestChange;
}


void SetLastSelfTestChangedTime(unsigned long setSelfTestChange) {
  LastSelfTestChange = setSelfTestChange;
}
