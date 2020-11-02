#ifndef _LOCALIZATE_H
#define _LOCALIZATE_H


int InitProcessComunication();

void InitLocalizate();

void PriorLocation(float& prior_x, float& prior_y, float& prior_z,
                   float& prior_roll, float& prior_pitch, float& prior_yaw,
                   float& prior_v, float& prior_w);

void PosteriorLocation(float& post_x, float& post_y, float& post_z,
                       float& post_roll, float& post_pitch, float& post_yaw,
                       float& post_v, float& post_w);

void DestoryLocalizate();
#endif
