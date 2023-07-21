//
// Created by bismarck on 23-3-18.
//

#include "inferRequest.h"
#include "../InferResult.h"
#include "modelManager.h"

InferRequest::InferRequest(cudaStream_t _stream, int _idx, ModelManager* _mm) {
    idx = _idx;
    mm = _mm;
    _mm->memoryUsing[_idx] = true;
    stream = _stream;
    output = _mm->binding_p[_idx].data[1];
}

InferRequest::~InferRequest() {
    if (output != nullptr) {
        output = nullptr;
        mm->memoryUsing[idx] = false;
    }
    if (stream != nullptr) {
        CHECK(cudaStreamDestroy(stream));
        stream = nullptr;
    }
}

InferResult InferRequest::get() {
    if (stream == nullptr || output == nullptr) {
        return {0, false, 0};
    }
    CHECK(cudaStreamSynchronize(stream));
    return ModelManager::postprocess(output);
}

InferRequest& InferRequest::operator=(InferRequest &&other) noexcept {
    if (&other == this) {
        return *this;
    }
    stream = other.stream;
    other.stream = nullptr;
    output = other.output;
    other.output = nullptr;
    idx = other.idx;
    other.idx = -1;
    return *this;
}
