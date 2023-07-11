//
// Created by bismarck on 23-3-19.
//

#ifndef DIGITALRECOGNITION_COMMON_H
#define DIGITALRECOGNITION_COMMON_H

constexpr int output_size = 13;
constexpr int height = 32;
constexpr int width = 32;
constexpr int input_size = height * width;
#ifdef MOBILE_NET
constexpr const char* output_name = "874";
#endif
#ifdef BP
constexpr const char* output_name = "14";
#endif

#endif //DIGITALRECOGNITION_COMMON_H
