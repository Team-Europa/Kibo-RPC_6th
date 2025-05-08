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

#include <android/asset_manager_jni.h>
#include <android/native_window_jni.h>
#include <android/native_window.h>

#include <android/log.h>

#include <jni.h>

#include <string>
#include <vector>

#include <platform.h>
#include <benchmark.h>

#include "yolo11.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#if __ARM_NEON
#include <arm_neon.h>
#endif // __ARM_NEON

static YOLO11* g_yolo11 = 0;
static ncnn::Mutex lock;

extern "C" {

JNIEXPORT jint JNI_OnLoad(JavaVM* vm, void* reserved)
{
    __android_log_print(ANDROID_LOG_DEBUG, "ncnn", "JNI_OnLoad");

    ncnn::create_gpu_instance();

    return JNI_VERSION_1_4;
}

JNIEXPORT void JNI_OnUnload(JavaVM* vm, void* reserved)
{
    __android_log_print(ANDROID_LOG_DEBUG, "ncnn", "JNI_OnUnload");

    {
        ncnn::MutexLockGuard g(lock);

        delete g_yolo11;
        g_yolo11 = 0;
    }

    ncnn::destroy_gpu_instance();
}

// public native boolean loadModel(AssetManager mgr, int taskid, int modelid, int cpugpu);
JNIEXPORT jboolean JNICALL
Java_jp_jaxa_iss_kibo_rpc_defaultapk_YOLO11Ncnn_loadModel(JNIEnv* env, jobject thiz, jobject assetManager, jint taskid, jint modelid, jint cpugpu)
{
    if (taskid < 0 || taskid > 4 || modelid < 0 || modelid > 5 || cpugpu < 0 || cpugpu > 2)
    {
        return JNI_FALSE;
    }

    AAssetManager* mgr = AAssetManager_fromJava(env, assetManager);

    __android_log_print(ANDROID_LOG_DEBUG, "ncnn", "loadModel %p", mgr);

    const char* tasknames[5] =
    {
        "",
        "_seg",
        "_pose",
        "_cls",
        "_obb"
    };

    const char* modeltypes[5] =
    {
        "n",
        "s",
        "m",
        "l",
        "x",
    };

    std::string parampath = std::string("yolo11") + modeltypes[(int)modelid] + tasknames[(int)taskid] + ".ncnn.param";
    std::string modelpath = std::string("yolo11") + modeltypes[(int)modelid] + tasknames[(int)taskid] + ".ncnn.bin";
    bool use_gpu = (int)cpugpu == 1;
    bool use_turnip = (int)cpugpu == 2;

    // reload
    {
        ncnn::MutexLockGuard g(lock);

        {
            static int old_taskid = 0;
            static int old_modelid = 0;
            static int old_cpugpu = 0;
            if (taskid != old_taskid || (modelid % 3) != old_modelid || cpugpu != old_cpugpu)
            {
                // taskid or model or cpugpu changed
                delete g_yolo11;
                g_yolo11 = 0;
            }
            old_taskid = taskid;
            old_modelid = modelid % 3;
            old_cpugpu = cpugpu;

            ncnn::destroy_gpu_instance();

            if (use_turnip)
            {
                ncnn::create_gpu_instance("libvulkan_freedreno.so");
            }
            else if (use_gpu)
            {
                ncnn::create_gpu_instance();
            }

            if (!g_yolo11)
            {
                if (taskid == 0) g_yolo11 = new YOLO11_det;
                if (taskid == 1) g_yolo11 = new YOLO11_seg;
                if (taskid == 2) g_yolo11 = new YOLO11_pose;
                if (taskid == 3) g_yolo11 = new YOLO11_cls;
                if (taskid == 4) g_yolo11 = new YOLO11_obb;

                g_yolo11->load(mgr, parampath.c_str(), modelpath.c_str(), use_gpu || use_turnip);
            }
            int target_size = 320;
            if ((int)modelid >= 3)
                target_size = 480;
            if ((int)modelid >= 6)
                target_size = 640;
            g_yolo11->set_det_target_size(target_size);
        }
    }

    return JNI_TRUE;
}

JNIEXPORT jobjectArray JNICALL
Java_jp_jaxa_iss_kibo_rpc_defaultapk_YOLO11Ncnn_detectObjects(JNIEnv* env, jobject thiz, jobject bitmapObj)
{
    cv::Mat input;
    AndroidBitmapInfo info;
    void* pixels = nullptr;

    // Get Bitmap Info
    if (AndroidBitmap_getInfo(env, bitmapObj, &info) != ANDROID_BITMAP_RESULT_SUCCESS){
        return nullptr;
    }

    // Set pixel buffer zone
    if (AndroidBitmap_lockPixels(env, bitmapObj, &pixels) != ANDROID_BITMAP_RESULT_SUCCESS){
        return nullptr;
    }

    // Create OpenCV Mat object
    if(info.format == ANDROID_BITMAP_FORMAT_RGBA_8888){
        cv::Mat temp(info.height, info.width, CV_8UC4, pixels);
        cv::cvtColor(temp, input, cv::COLOR_RGBA2RGB);
    }
    else {
        AndroidBitmap_unlockPixels(env, bitmapObj);
        return nullptr;
    }

    std::vector<Object> objects;

    {
        ncnn::MutexLockGuard g(lock);

        if (!g_yolo11) {
            AndroidBitmap_unlockPixels(env, bitmapObj);
            return nullptr;
        }
        g_yolo11->detect(input, objects);
    }

    jclass objClass = env->FindClass("jp/jaxa/iss/kibo/rpc/defaultapk/model/DetectionResult");
    if (!objClass) return nullptr;

    jmethodID constructor = env->GetMethodID(objClass, "<init>", "(IFIIII)V");
    if (!constructor) {
        env->DeleteLocalRef(objClass);
        return nullptr;
    }

    jobjectArray result = env->NewObjectArray(objects.size(), objClass, nullptr);

    env->DeleteLocalRef(objClass);

    for (size_t i = 0; i < objects.size(); i++) {
        const Object& obj = objects[i];

        jclass localObjClass = env->FindClass("jp/jaxa/iss/kibo/rpc/defaultapk/model/DetectionResult");
        if (!localObjClass) break;

        jobject objResult = env->NewObject(localObjClass, constructor,
                                           obj.label, obj.prob,
                                           (int)obj.rect.x, (int)obj.rect.y,
                                           (int)obj.rect.width, (int)obj.rect.height);
        if (objResult) {
            env->SetObjectArrayElement(result, i, objResult);
            env->DeleteLocalRef(objResult);
        }
        env->DeleteLocalRef(localObjClass);
    }

    return result;
}

JNIEXPORT jobjectArray JNICALL
Java_jp_jaxa_iss_kibo_rpc_defaultapk_YOLO11Ncnn_detectSegObjects(JNIEnv* env, jobject thiz, jobject bitmapObj)
{
    cv::Mat input;
    AndroidBitmapInfo info;
    void* pixels = nullptr;

    // Get Bitmap Info
    if (AndroidBitmap_getInfo(env, bitmapObj, &info) != ANDROID_BITMAP_RESULT_SUCCESS){
        return nullptr;
    }

    // Set pixel buffer zone
    if (AndroidBitmap_lockPixels(env, bitmapObj, &pixels) != ANDROID_BITMAP_RESULT_SUCCESS){
        return nullptr;
    }

    // Create OpenCV Mat object
    if(info.format == ANDROID_BITMAP_FORMAT_RGBA_8888){
        cv::Mat temp(info.height, info.width, CV_8UC4, pixels);
        cv::cvtColor(temp, input, cv::COLOR_RGBA2RGB);
    }
    else {
        AndroidBitmap_unlockPixels(env, bitmapObj);
        return nullptr;
    }

    std::vector<Object> objects;

    {
        ncnn::MutexLockGuard g(lock);

        if (!g_yolo11) {
            return nullptr;
        }
        g_yolo11->detect(input, objects);
    }

    jclass resultClass = env->FindClass("jp/jaxa/iss/kibo/rpc/defaultapk/model/SegDetectionResult");
    if (!resultClass) return nullptr;

    jmethodID constructor = env->GetMethodID(resultClass, "<init>", "(IFIIII[B)V");
    if (!constructor) {
        env->DeleteLocalRef(resultClass);
        return nullptr;
    }

    jobjectArray result = env->NewObjectArray(objects.size(), resultClass, nullptr);

    for (size_t i = 0; i < objects.size(); i++) {
        const Object& obj = objects[i];

        jbyteArray maskArray = nullptr;
        if (!obj.mask.empty()) {
            cv::Mat continuousMask;
            if (!obj.mask.isContinuous()) {
                continuousMask = obj.mask.clone();
            } else {
                continuousMask = obj.mask;
            }

            int maskSize = continuousMask.total() * continuousMask.elemSize();
            jbyte* maskData = (jbyte*)continuousMask.data;
            maskArray = env->NewByteArray(maskSize);
            env->SetByteArrayRegion(maskArray, 0, maskSize, maskData);
        }

        jobject objResult = env->NewObject(resultClass, constructor,
                                           obj.label, obj.prob,
                                           (int)obj.rect.x, (int)obj.rect.y,
                                           (int)obj.rect.width, (int)obj.rect.height,
                                           maskArray);

        if (objResult) {
            env->SetObjectArrayElement(result, i, objResult);
            env->DeleteLocalRef(objResult);
        }

        if (maskArray) {
            env->DeleteLocalRef(maskArray);
        }
    }

    env->DeleteLocalRef(resultClass);

    return result;
}
}
