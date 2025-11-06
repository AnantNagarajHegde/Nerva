// stress_rf_model.h   (must be in the same folder as the .ino)
#ifndef STRESS_RF_MODEL_H
#define STRESS_RF_MODEL_H
#include <Arduino.h>

inline int32_t predict_stress(const float* f) {
  if (f[0] > 1.8f && f[8] > 1.0f) return 1;   // Stress
  if (f[10] > 5.0f) return 2;               // Amusement
  return 0;                                 // Baseline
}
#endif




// // Auto-generated Random Forest model for TinyML
// // Generated using emlearn
// // Model: 100 trees, max_depth=None

// #ifndef STRESS_RF_MODEL_H
// #define STRESS_RF_MODEL_H

// #include <stdint.h>

// // Feature names for reference
// // 0: ECG_mean, 1: ECG_std, 2: ECG_min, 3: ECG_max
// // 4: HRV_MeanNN, 5: HRV_SDNN, 6: HRV_RMSSD, 7: HRV_pNN50
// // 8: EDA_tonic, 9: EDA_phasic, 10: SCR_peaks, 11: SCR_amp

// // Class labels
// // 0: Baseline, 1: Stress, 2: Amusement

// <emlearn.trees.Wrapper object at 0x0000027A6FEBCCD0>

// // Prediction function
// int32_t predict_stress(float features[12]) {
//     return model_predict(features, 12);
// }

// #endif  // STRESS_RF_MODEL_H
