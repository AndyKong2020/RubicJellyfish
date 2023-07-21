//
// Created by bismarck on 23-3-18.
//
#include "modelManager.h"
#include <fstream>
#include "logging.h"

Logger glogger;
InferResult datasetId2InferResult[] = {
        {1, true, 1},
        {2, false, 1},
        {3, false, 1},
        {4, false, 1},
        {5, false, 1},
        {6, false, 1},
        {7, false, 1},
        {8, true, 1},
        {0, false, 0},
};

InferResult ModelManager::postprocess(const float *res) {
    InferResult maxRes{255, false, 0};
    for(int i = 0; i < output_size - 1; i++) {
        if (maxRes.id == 255 || res[i] > maxRes.confidence) {
            maxRes = datasetId2InferResult[i];
            maxRes.confidence = res[i];
        }
    }
    return maxRes;
}

void ModelManager::preprocess(cv::Mat &img, int idx, cudaStream_t stream) {
    if(img.channels() > 1) {
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    }
    if (img.rows != height || img.cols != width) {
        cv::resize(img, img, cv::Size(width, height));
    }
    if (img.type() != INPUT_MAT_TYPE) {
        img.convertTo(img, INPUT_MAT_TYPE);
    }
    memcpy(input_p[idx], img.data, input_size * sizeof(INPUT_VAR_TYPE));
#ifdef JETSON
    CHECK(cuMemPrefetchAsync((CUdeviceptr)input_p[idx], input_size * sizeof(INPUT_VAR_TYPE), device, stream));
#endif
    cuda_preprocess(input_p[idx], binding_p[idx].data[0], input_size, stream);
}

void ModelManager::init() {
    cudaGetDevice(&device);
    char* trtModelStream{nullptr};
    size_t size{0};
    std::ifstream file(MODEL_PATH, std::ios::binary);
    if (file.good()) {
        file.seekg(0, std::ifstream::end);
        size = file.tellg();
        file.seekg(0, std::ifstream::beg);
        trtModelStream = new char[size];
        assert(trtModelStream);
        file.read(trtModelStream, (long)size);
        file.close();
    }
    runtime = nvinfer1::createInferRuntime(glogger);
    assert(runtime != nullptr);
    engine = runtime->deserializeCudaEngine(trtModelStream, size);
    assert(engine != nullptr);
    scaleMemoryPoll();
}

void ModelManager::scaleMemoryPoll(int size) {
    std::cerr << "Infer slots use out, malloc new!" << std::endl;
    for (int i = 0; i < size; i++) {
        memoryUsing.emplace_back(false);
        INPUT_VAR_TYPE* input_;
        CHECK(cudaMallocManaged((void**)&input_, input_size * sizeof(INPUT_VAR_TYPE)));
        input_p.push_back(input_);
        float* preprocess_;
        CHECK(cudaMallocManaged((void**)&preprocess_, input_size * sizeof(float)));
        float* output_;
        CHECK(cudaMallocManaged((void**)&output_, output_size * sizeof(float)));
        binding_p.push_back({preprocess_, output_});
        auto context_ = engine->createExecutionContext();
        assert(context_ != nullptr);
        context_p.push_back(context_);
    }
}

InferResult ModelManager::infer_sync(cv::Mat& img) {
    cudaStream_t stream;
    CHECK(cudaStreamCreate(&stream));
    int idx = getMemorySlot();
    preprocess(img, idx, stream);
    context_p[idx]->enqueueV2((void**)binding_p[idx].data, stream, nullptr);
#ifdef JETSON
    CHECK(cuMemPrefetchAsync((CUdeviceptr)binding_p[idx].data[1],
                             output_size * sizeof(INPUT_VAR_TYPE), cudaCpuDeviceId, stream));
#endif
    CHECK(cudaStreamSynchronize(stream));
    auto res = ModelManager::postprocess(binding_p[idx].data[1]);
    memoryUsing[idx] = false;
    CHECK(cudaStreamDestroy(stream));
    return res;
}

InferResultAsync ModelManager::infer_async(cv::Mat& img) {
    cudaStream_t stream;
    CHECK(cudaStreamCreate(&stream));
    int idx = getMemorySlot();
    InferRequestPtr ir = std::make_shared<InferRequest>(stream, idx, this);
    preprocess(img, idx, stream);
    context_p[idx]->enqueueV2((void**)binding_p[idx].data, stream, nullptr);
#ifdef JETSON
    CHECK(cuMemPrefetchAsync((CUdeviceptr)binding_p[idx].data[1],
                             output_size * sizeof(INPUT_VAR_TYPE), cudaCpuDeviceId, stream));
#endif
    return InferResultAsync(std::move(ir));
}

InferResultAsync ModelManager::infer_async(cv::Mat&& img) {
    return infer_async(img);
}

int ModelManager::getMemorySlot() {
    int size = (int)memoryUsing.size();
    for (int i = 0; i < size; i++) {
        bool _t = false;
        if (memoryUsing[i].compare_exchange_strong(_t, true)) {
            return i;
        }
    }
    scaleMemoryPoll();
    memoryUsing[size] = true;
    return size;
}

ModelManager::~ModelManager() {
    int size = (int)memoryUsing.size();
    for (int i = 0; i < size; i++) {
        cudaFree(input_p[i]);
        cudaFree(binding_p[i].data[0]);
        cudaFree(binding_p[i].data[1]);
//        delete context_p[i];
    }
//    delete runtime;
//    delete engine;
}
