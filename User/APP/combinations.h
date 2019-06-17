#ifndef COMBINATIONS_H
#define COMBINATIONS_H
#include "robocon.h"



void StepOver(void);
void CrossTheLine(void);
void CrossTheLine_one_leg(int LegId);
void CrossTheLine_all(void);
void StepOver_one_leg(int LegId);

void StartPosToMiddlePos (void);
void MiddlePosToEndPos (void);

void OpenMvInspect(int color);
void Climbing_Comb(void);
void KeyToken_Test(void);
#endif
