#include "AI.h"

#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
#include <tensorflow/lite/version.h>
#include "model.h"
#include <Arduino.h>

#define byte uint8_t

// global variables used for TensorFlow Lite (Micro)
tflite::MicroErrorReporter tflErrorReporter;

// pull in all the TFLM ops, you can remove this line and
// only pull in the TFLM ops you need, if would like to reduce
// the compiled size of the sketch.
tflite::AllOpsResolver tflOpsResolver;

const tflite::Model* tflModel = nullptr;
tflite::MicroInterpreter* tflInterpreter = nullptr;
TfLiteTensor* tflInputTensor = nullptr;
TfLiteTensor* tflOutputTensor = nullptr;

// Create a static memory buffer for TFLM, the size may need to
// be adjusted based on the model you are using
const int tensorArenaSize = 8 * 1024;
byte tensorArena[tensorArenaSize];

// array to map gesture index to a name
const char* PLASTICS[] = {
  "PET",
  "PEHD",
  "PVC",
  "PEBD",
  "PP",
  "PS",
  "OTHERS"
};

#define NUM_PLASTICS (sizeof(PLASTICS) / sizeof(PLASTICS[0]))

void AI::setup() {

  tflModel = tflite::GetModel(model);
  if (tflModel->version() != TFLITE_SCHEMA_VERSION) {
    while (1);
  }

  // Create an interpreter to run the model
  tflInterpreter = new tflite::MicroInterpreter(tflModel, tflOpsResolver, tensorArena, tensorArenaSize, &tflErrorReporter);

  // Allocate memory for the model's input and output tensors
  tflInterpreter->AllocateTensors();

  // Get pointers for the model's input and output tensors
  tflInputTensor = tflInterpreter->input(0);
  tflOutputTensor = tflInterpreter->output(0);
}

//take normalised data and return the result from the plastics enum
const char* AI::triage(float inputArray[samples_Num]) {
  for (int i = 0; i < samples_Num; i++) {
    tflInputTensor->data.f[i] = inputArray[i];
  }

  TfLiteStatus invokeStatus = tflInterpreter->Invoke();
  if (invokeStatus != kTfLiteOk) {
    while (1);
    return "invoke failed";
  }
  int index = 0;
  for (int i = 0; i < NUM_PLASTICS; i++) {
    if(tflOutputTensor->data.f[i] >= tflOutputTensor->data.f[index]){
      index = i;
    }    
  }

  return PLASTICS[index];
}
