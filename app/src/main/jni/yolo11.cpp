// Tencent is pleased to support the open source community by making ncnn available.
//
// Copyright (C) 2025 THL A29 Limited, a Tencent company. All rights reserved.
//
// Licensed under the BSD 3-Clause License (the "License"); you may not use this file except
// in compliance with the License. You may obtain a copy of the License at
//
// https://opensource.org/licenses/BSD-3-Clause
//
// Unless required by applicable law or agreed to in writing, software distributed
// under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

#include "yolo11.h"

YOLO11::~YOLO11()
{
    det_target_size = 320;
}

int YOLO11::load(const char* parampath, const char* modelpath, bool use_gpu)
{
    yolo11.clear();

    yolo11.opt = ncnn::Option();

#if NCNN_VULKAN
    yolo11.opt.use_vulkan_compute = use_gpu;
#endif

    yolo11.load_param(parampath);
    yolo11.load_model(modelpath);

    return 0;
}

int YOLO11::load(AAssetManager* mgr, const char* parampath, const char* modelpath, bool use_gpu)
{
    yolo11.clear();

    yolo11.opt = ncnn::Option();

#if NCNN_VULKAN
    yolo11.opt.use_vulkan_compute = use_gpu;
#endif

    yolo11.load_param(mgr, parampath);
    yolo11.load_model(mgr, modelpath);

    return 0;
}

void YOLO11::set_det_target_size(int target_size)
{
    det_target_size = target_size;
}
