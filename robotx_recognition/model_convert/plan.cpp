#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <NvInfer.h>
#include <NvUffParser.h>

using namespace std;
using namespace nvinfer1;
using namespace nvuffparser;


class Logger : public ILogger {
  void log(Severity severity, const char * msg) override
  {
    cout << msg << endl;
  }
} gLogger;

void usage() {
  cout << "Usage: plan <*.uff(input)> <*.plan(output)> <inputNodeName> <outputNodeName>" << endl;
}

int main(int argc, char *argv[]) {
  if (argc != 5) {
    usage();
    return -1;
  }
  IBuilder *builder = createInferBuilder(gLogger);
  INetworkDefinition *network = builder->createNetwork();
  IUffParser *parser = createUffParser();

  // argv[1] = *.uff
  parser->registerInput(argv[3], DimsCHW(3, 224, 224));
  parser->registerOutput(argv[4]);
  parser->parse(argv[1], *network, DataType::kFLOAT);  // or, kHALF

  builder->setMaxBatchSize(1);
  builder->setMaxWorkspaceSize(1<<20);
  ICudaEngine *engine = builder->buildCudaEngine(*network);

  ofstream f;
  f.open(argv[2]);  // plan
  IHostMemory *serializedEngine = engine->serialize();
  f.write((char *)serializedEngine->data(), serializedEngine->size());
  f.close();

  builder->destroy();
  parser->destroy();
  network->destroy();
  engine->destroy();
  serializedEngine->destroy();

  return 0;
}
