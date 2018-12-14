#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <ros/ros.h>

// utils
#include <cnn_utils.h>

// TensorRT
#include <NvInfer.h>
#include <NvUffParser.h>

using namespace nvinfer1;
using namespace nvuffparser;

class Logger : public ILogger {
  void log(Severity severity, const char * msg) override {
    if (severity != Severity::kINFO)
      ROS_INFO("[[infer.cu]] %s", msg);
  }
} gLogger;

// runtimes
IRuntime *runtime;
ICudaEngine *engine;
IExecutionContext *context;
int inputBindingIndex, outputBindingIndex;
int inputHeight, inputWidth;
Dims inputDims, outputDims;
bool is_initialized = false;
void *bindings[2];

// flags
bool use_mappedMemory;

// pointers
size_t numInput, numOutput;
float *inputDataHost, *outputDataHost;
float *inputDataDevice, *outputDataDevice;

int convert(std::string uffFilename, std::string planFilename, std::string inputName, std::string outputName) {
  IBuilder *builder = createInferBuilder(gLogger);
  INetworkDefinition *network = builder->createNetwork();
  IUffParser *parser = createUffParser();

  parser->registerInput(inputName.c_str(), DimsCHW(3, 224, 224));
  parser->registerOutput(outputName.c_str());
  parser->parse(uffFilename.c_str(), *network, DataType::kFLOAT);  // or, kHALF

  builder->setMaxBatchSize(1);
  builder->setMaxWorkspaceSize(1<<20);
  ICudaEngine *_engine = builder->buildCudaEngine(*network);

  ofstream f;
  f.open(planFilename.c_str());  // plan
  IHostMemory *serializedEngine = _engine->serialize();
  f.write((char *)serializedEngine->data(), serializedEngine->size());
  f.close();

  builder->destroy();
  parser->destroy();
  network->destroy();
  _engine->destroy();
  serializedEngine->destroy();

  return 0;
}

int setup(std::string planFilename, std::string inputName, std::string outputName, bool _use_mappedMemory) {
  ROS_INFO("setup");
  std::ifstream planFile(planFilename.c_str());
  if(!planFile.is_open()) {
    ROS_INFO("cannot get plan file");
    is_initialized = false;
    return -1;
  } else {
    std::stringstream planBuffer;
    planBuffer << planFile.rdbuf();
    std::string plan = planBuffer.str();

    use_mappedMemory = _use_mappedMemory;

    runtime = createInferRuntime(gLogger);
    engine  = runtime->deserializeCudaEngine((void*)plan.data(), plan.size(), nullptr);
    context = engine->createExecutionContext();
    ROS_INFO("load setup finished");

    inputBindingIndex = engine->getBindingIndex(inputName.c_str());
    outputBindingIndex = engine->getBindingIndex(outputName.c_str());
    inputDims = engine->getBindingDimensions(inputBindingIndex);
    outputDims = engine->getBindingDimensions(outputBindingIndex);
    inputHeight = inputDims.d[1];
    inputWidth = inputDims.d[2];
    ROS_INFO("input: h=%d, w=%d", inputHeight, inputWidth);

    numInput = numTensorElements(inputDims);
    numOutput = numTensorElements(outputDims);

    if (use_mappedMemory) {
      // host
      cudaHostAlloc(&inputDataHost, numInput * sizeof(float), cudaHostAllocMapped);
      cudaHostAlloc(&outputDataHost, numOutput * sizeof(float), cudaHostAllocMapped);
      // device
      cudaHostGetDevicePointer(&inputDataDevice, inputDataHost, 0);
      cudaHostGetDevicePointer(&outputDataDevice, outputDataHost, 0);
    } else {
      // host
      inputDataHost = (float*) malloc(numInput * sizeof(float));
      outputDataHost = (float*) malloc(numOutput * sizeof(float));
      // device
      cudaMalloc(&inputDataDevice, numInput * sizeof(float));
      cudaMalloc(&outputDataDevice, numOutput * sizeof(float));
    }
    bindings[inputBindingIndex] = (void*)inputDataDevice;
    bindings[outputBindingIndex] = (void*)outputDataDevice;

    is_initialized = true;
    ROS_INFO("initialize finished %d, %d", numInput, numOutput);

    return (int)numOutput;
  }
}

void destroy(void) {
  if(is_initialized) {
    runtime->destroy();
    engine->destroy();
    context->destroy();
    if(use_mappedMemory) {
      cudaFreeHost(inputDataHost);
      cudaFreeHost(outputDataHost);
    } else {
      free(inputDataHost);
      free(outputDataHost);
    }
    cudaFree(inputDataDevice);
    cudaFree(outputDataDevice);
  }
  is_initialized = false;
}

void infer(cv::Mat image, float* out) {
  // cvの画像からcnnを走らせる
  // preprocessing
  cv::resize(image, image, cv::Size(inputWidth, inputHeight));
  cvImageToTensor(image, inputDataHost, inputDims);
  preprocessVgg(inputDataHost, inputDims);

  // execute on cuda
  if (use_mappedMemory) {
    context->execute(1, bindings);
  } else {
    cudaMemcpy(inputDataDevice, inputDataHost, numInput * sizeof(float), cudaMemcpyHostToDevice);
    context->execute(1, bindings);
    cudaMemcpy(outputDataHost, outputDataDevice, numOutput * sizeof(float), cudaMemcpyDeviceToHost);
  }

  // output
  /* ROS_INFO("%f %f %f %f", outputDataHost[0], outputDataHost[1], outputDataHost[2], outputDataHost[3]); */
  for (int i = 0; i < (int)numOutput; i++) {
    out[i] = outputDataHost[i];
  }
}

void test(void) {
  ROS_INFO("inside cu");
  cudaDeviceSynchronize();
}

