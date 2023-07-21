//
// Created by bismarck on 23-3-15.
//

#ifndef DIGITALRECOGNITION_INFER_H
#define DIGITALRECOGNITION_INFER_H

#include "InferResult.h"

#ifdef OPENVINO
#include "openvino/modelManager.h"
#endif
#ifdef TENSORRT
#include "tensorrt/modelManager.h"
#endif
#ifdef RKNN
#include "rknn/modelManager.h"
#endif
#ifdef TEMPLATE
#include "templateMatch/modelManager.h"
#endif

#endif //DIGITALRECOGNITION_INFER_H
