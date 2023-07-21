//
// Created by bismarck on 23-3-15.
//

#include "InferResult.h"
#include <iostream>

#ifdef OPENVINO
#include "openvino/modelManager.h"
#endif

InferResultAsync::InferResultAsync(InferResultAsync::T&& _req): req(_req) {
    callable = true;
}

InferResultAsync &InferResultAsync::operator=(const InferResultAsync &other) {
    if (&other == this) {
        return *this;
    }
    req = other.req;
    result = other.result;
    callable = other.callable;
    called = other.called;
    set = other.set;
    return *this;
}

InferResult InferResultAsync::operator()() {
    if (called) {
        return result;
    } else {
        if (callable) {
            bool _type;
            if(set) {
                _type = result.big;
            }
            result = get();
            if(set) {
                result.big = _type;
            }
            called = true;
            callable = false;
            return result;
        } else {
            throw std::runtime_error("Call invalid InferResultAsync!");
        }
    }
}

void InferResultAsync::setMarkerType(bool type) {
    result.big = type;
    set = true;
}

#ifdef OPENVINO
InferResult InferResultAsync::get() {
    req.wait();
    InferResult res = ModelManager::postprocess(req.get_tensor(output_name));
    return res;
}

#endif

#ifdef TENSORRT
InferResult InferResultAsync::get() {
    auto res = req->get();
    req.reset();
    return res;
}
#endif

#ifdef RKNN
InferResult InferResultAsync::get() {
    try {
        return req.get();
    }
    catch (std::exception &e) {
        std::cerr << "Task exception: " << e.what() << std::endl;
        return InferResult{0, false, 0};
    }
}
#endif

#ifdef TEMPLATE
InferResult InferResultAsync::get() {
    try {
        return req.get();
    }
    catch (std::exception &e) {
        std::cerr << "Task exception: " << e.what() << std::endl;
        return InferResult{0, false, 0};
    }
}
#endif
