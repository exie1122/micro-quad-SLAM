#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

extern "C" {
#include "cvi_sys.h"
#include "cviruntime.h"
#include "core/utils/vpss_helper.h"
#include "middleware_utils.h"
}

static volatile sig_atomic_t g_stop = 0;
static void on_signal(int) { g_stop = 1; }

struct Options {
    std::string model;
    std::string log_path = "/root/yaw_live.csv";
    std::string resize_mode = "center_crop";
    std::string save_dir;
    int input_size = 128;
    int capture_width = 320;
    int capture_height = 240;
    int stride = 4;
    int max_pairs = 0;
    int save_every = 0;
    int calibrate_pairs = 20;
    double stationary_threshold = 4.0;
    double offset_alpha = 0.05;
};

static void usage(const char *argv0) {
    std::printf(
        "Usage: %s MODEL [options]\n"
        "  --size N             model input size (default 128)\n"
        "  --stride N           camera frames between predictions (default 4)\n"
        "  --capture WxH        VPSS capture size (default 320x240)\n"
        "  --resize MODE        center_crop or stretch (default center_crop)\n"
        "  --pairs N            stop after N predictions; 0 is continuous\n"
        "  --calibrate N        stationary pairs used for visual zero offset (default 20)\n"
        "  --stationary-pixel N mean absolute pixel delta treated as still (default 4.0)\n"
        "  --offset-alpha N     still-frame offset adaptation rate (default 0.05)\n"
        "  --log PATH           CSV output (default /root/yaw_live.csv)\n"
        "  --save-dir DIR       optional PGM sample directory\n"
        "  --save-every N       save every Nth prediction (default disabled)\n",
        argv0);
}

static bool parse_options(int argc, char **argv, Options &o) {
    if (argc < 2) return false;
    o.model = argv[1];
    for (int i = 2; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--size" && i + 1 < argc) o.input_size = std::atoi(argv[++i]);
        else if (a == "--stride" && i + 1 < argc) o.stride = std::atoi(argv[++i]);
        else if (a == "--pairs" && i + 1 < argc) o.max_pairs = std::atoi(argv[++i]);
        else if (a == "--calibrate" && i + 1 < argc) o.calibrate_pairs = std::atoi(argv[++i]);
        else if (a == "--stationary-pixel" && i + 1 < argc)
            o.stationary_threshold = std::atof(argv[++i]);
        else if (a == "--offset-alpha" && i + 1 < argc) o.offset_alpha = std::atof(argv[++i]);
        else if (a == "--log" && i + 1 < argc) o.log_path = argv[++i];
        else if (a == "--resize" && i + 1 < argc) o.resize_mode = argv[++i];
        else if (a == "--save-dir" && i + 1 < argc) o.save_dir = argv[++i];
        else if (a == "--save-every" && i + 1 < argc) o.save_every = std::atoi(argv[++i]);
        else if (a == "--capture" && i + 1 < argc) {
            if (std::sscanf(argv[++i], "%dx%d", &o.capture_width, &o.capture_height) != 2)
                return false;
        } else return false;
    }
    return o.input_size > 0 && o.capture_width > 0 && o.capture_height > 0 && o.stride > 0 &&
           o.calibrate_pairs >= 0 && o.stationary_threshold >= 0.0 && o.offset_alpha >= 0.0 &&
           o.offset_alpha <= 1.0 &&
           (o.resize_mode == "center_crop" || o.resize_mode == "stretch");
}

static int setup_middleware(int width, int height, SAMPLE_TDL_MW_CONFIG_S *cfg,
                            SAMPLE_TDL_MW_CONTEXT *ctx) {
    CVI_S32 ret = SAMPLE_TDL_Get_VI_Config(&cfg->stViConfig);
    if (ret != CVI_SUCCESS || cfg->stViConfig.s32WorkingViNum <= 0) return -1;

    PIC_SIZE_E pic_size;
    SIZE_S sensor_size;
    ret = SAMPLE_COMM_VI_GetSizeBySensor(cfg->stViConfig.astViInfo[0].stSnsInfo.enSnsType,
                                         &pic_size);
    if (ret != CVI_SUCCESS || SAMPLE_COMM_SYS_GetPicSize(pic_size, &sensor_size) != CVI_SUCCESS)
        return -1;

    cfg->stVBPoolConfig.u32VBPoolCount = 1;
    cfg->stVBPoolConfig.astVBPoolSetup[0].enFormat = PIXEL_FORMAT_BGR_888;
    cfg->stVBPoolConfig.astVBPoolSetup[0].u32BlkCount = 3;
    cfg->stVBPoolConfig.astVBPoolSetup[0].u32Width = width;
    cfg->stVBPoolConfig.astVBPoolSetup[0].u32Height = height;
    cfg->stVBPoolConfig.astVBPoolSetup[0].bBind = true;
    cfg->stVBPoolConfig.astVBPoolSetup[0].u32VpssChnBinding = VPSS_CHN0;
    cfg->stVBPoolConfig.astVBPoolSetup[0].u32VpssGrpBinding = (VPSS_GRP)0;

    cfg->stVPSSPoolConfig.u32VpssGrpCount = 1;
#ifndef CV186X
    cfg->stVPSSPoolConfig.stVpssMode.aenInput[0] = VPSS_INPUT_MEM;
    cfg->stVPSSPoolConfig.stVpssMode.enMode = VPSS_MODE_DUAL;
    cfg->stVPSSPoolConfig.stVpssMode.ViPipe[0] = 0;
    cfg->stVPSSPoolConfig.stVpssMode.aenInput[1] = VPSS_INPUT_ISP;
    cfg->stVPSSPoolConfig.stVpssMode.ViPipe[1] = 0;
#endif
    SAMPLE_TDL_VPSS_CONFIG_S *v = &cfg->stVPSSPoolConfig.astVpssConfig[0];
    v->bBindVI = true;
    VPSS_GRP_DEFAULT_HELPER2(&v->stVpssGrpAttr, sensor_size.u32Width, sensor_size.u32Height,
                             VI_PIXEL_FORMAT, 1);
    v->u32ChnCount = 1;
    v->u32ChnBindVI = VPSS_CHN0;
    VPSS_CHN_DEFAULT_HELPER(&v->astVpssChnAttr[0], width, height, PIXEL_FORMAT_BGR_888, true);
    return SAMPLE_TDL_Init_WM_NO_RTSP(cfg, ctx) == CVI_SUCCESS ? 0 : -1;
}

static cv::Mat preprocess(const cv::Mat &bgr, int size, const std::string &mode) {
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    if (mode == "center_crop") {
        int side = std::min(gray.cols, gray.rows);
        int x = (gray.cols - side) / 2;
        int y = (gray.rows - side) / 2;
        gray = gray(cv::Rect(x, y, side, side));
    }
    cv::Mat resized;
    cv::resize(gray, resized, cv::Size(size, size), 0, 0, cv::INTER_AREA);
    return resized;
}

static bool get_frame(int size, const std::string &mode, cv::Mat &out) {
    VIDEO_FRAME_INFO_S frame = {};
    CVI_S32 ret = CVI_VPSS_GetChnFrame(0, VPSS_CHN0, &frame, 2000);
    if (ret != CVI_SUCCESS) return false;
    size_t bytes = frame.stVFrame.u32Length[0] + frame.stVFrame.u32Length[1] +
                   frame.stVFrame.u32Length[2];
    void *mapped = CVI_SYS_Mmap(frame.stVFrame.u64PhyAddr[0], bytes);
    if (!mapped) {
        CVI_VPSS_ReleaseChnFrame(0, VPSS_CHN0, &frame);
        return false;
    }
    CVI_SYS_IonInvalidateCache(frame.stVFrame.u64PhyAddr[0], mapped, bytes);
    cv::Mat bgr(frame.stVFrame.u32Height, frame.stVFrame.u32Width, CV_8UC3, mapped,
                frame.stVFrame.u32Stride[0]);
    out = preprocess(bgr, size, mode).clone();
    CVI_SYS_Munmap(mapped, bytes);
    CVI_VPSS_ReleaseChnFrame(0, VPSS_CHN0, &frame);
    return true;
}

static bool save_pgm(const std::string &path, const cv::Mat &gray) {
    FILE *f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    std::fprintf(f, "P5\n%d %d\n255\n", gray.cols, gray.rows);
    for (int y = 0; y < gray.rows; ++y)
        std::fwrite(gray.ptr(y), 1, gray.cols, f);
    std::fclose(f);
    return true;
}

int main(int argc, char **argv) {
    Options o;
    if (!parse_options(argc, argv, o)) {
        usage(argv[0]);
        return 2;
    }
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    SAMPLE_TDL_MW_CONFIG_S mw_cfg = {};
    SAMPLE_TDL_MW_CONTEXT mw_ctx = {};
    if (setup_middleware(o.capture_width, o.capture_height, &mw_cfg, &mw_ctx) != 0) {
        std::fprintf(stderr, "camera middleware initialization failed\n");
        return 3;
    }

    CVI_MODEL_HANDLE model = nullptr;
    CVI_TENSOR *inputs = nullptr, *outputs = nullptr;
    int32_t input_count = 0, output_count = 0;
    if (CVI_NN_RegisterModel(o.model.c_str(), &model) != CVI_RC_SUCCESS ||
        CVI_NN_GetInputOutputTensors(model, &inputs, &input_count, &outputs, &output_count) !=
            CVI_RC_SUCCESS || input_count < 1 || output_count < 1) {
        std::fprintf(stderr, "model initialization failed: %s\n", o.model.c_str());
        SAMPLE_TDL_Destroy_MW_NO_RTSP(&mw_ctx);
        return 4;
    }
    const size_t expected_bytes = (size_t)2 * o.input_size * o.input_size * sizeof(float);
    if (CVI_NN_TensorSize(&inputs[0]) != expected_bytes || CVI_NN_TensorCount(&outputs[0]) < 2) {
        std::fprintf(stderr, "unexpected model tensors: input=%zu expected=%zu output_count=%zu\n",
                     CVI_NN_TensorSize(&inputs[0]), expected_bytes,
                     CVI_NN_TensorCount(&outputs[0]));
        CVI_NN_CleanupModel(model);
        SAMPLE_TDL_Destroy_MW_NO_RTSP(&mw_ctx);
        return 5;
    }

    FILE *log = std::fopen(o.log_path.c_str(), "w");
    if (!log) {
        std::perror("open log");
        CVI_NN_CleanupModel(model);
        SAMPLE_TDL_Destroy_MW_NO_RTSP(&mw_ctx);
        return 6;
    }
    std::fprintf(log, "pair,timestamp_s,raw_yaw_deg,visual_offset_deg,relative_yaw_deg,accumulated_yaw_deg,pixel_mae,stationary_gate,npu_ms\n");
    std::fflush(log);
    if (!o.save_dir.empty()) {
        std::string command = "mkdir -p '" + o.save_dir + "'";
        std::system(command.c_str());
    }

    std::printf("YAW_LIVE model=%s input=%d capture=%dx%d stride=%d resize=%s calibrate=%d\n",
                o.model.c_str(), o.input_size, o.capture_width, o.capture_height, o.stride,
                o.resize_mode.c_str(), o.calibrate_pairs);
    if (o.calibrate_pairs > 0)
        std::printf("CALIBRATION: keep the camera stationary until VISUAL_OFFSET is printed.\n");
    cv::Mat previous, current;
    int frame_index = 0, pair_index = 0;
    int calibration_count = 0;
    double calibration_sum = 0.0;
    double visual_offset = 0.0;
    double accumulated = 0.0;
    const double rad_to_deg = 180.0 / 3.14159265358979323846;
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();

    while (!g_stop && (o.max_pairs <= 0 || pair_index < o.max_pairs)) {
        if (!get_frame(o.input_size, o.resize_mode, current)) {
            std::fprintf(stderr, "camera frame acquisition failed\n");
            break;
        }
        if ((frame_index++ % o.stride) != 0) continue;
        if (previous.empty()) {
            previous = current.clone();
            continue;
        }

        float *input = static_cast<float *>(CVI_NN_TensorPtr(&inputs[0]));
        const size_t pixels = (size_t)o.input_size * o.input_size;
        for (size_t i = 0; i < pixels; ++i) {
            input[i] = previous.data[i] / 255.0f;
            input[pixels + i] = current.data[i] / 255.0f;
        }
        auto npu_start = Clock::now();
        CVI_RC rc = CVI_NN_Forward(model, inputs, input_count, outputs, output_count);
        auto npu_end = Clock::now();
        if (rc != CVI_RC_SUCCESS) {
            std::fprintf(stderr, "NPU forward failed: %d\n", rc);
            break;
        }
        const float *output = static_cast<const float *>(CVI_NN_TensorPtr(&outputs[0]));
        double raw_yaw_deg = std::atan2((double)output[0], (double)output[1]) * rad_to_deg;
        cv::Mat pixel_delta;
        cv::absdiff(previous, current, pixel_delta);
        double pixel_mae = cv::mean(pixel_delta)[0];
        if (calibration_count < o.calibrate_pairs) {
            calibration_sum += raw_yaw_deg;
            ++calibration_count;
            previous = current.clone();
            std::printf("calibrating %d/%d raw=%+8.3f deg\n", calibration_count,
                        o.calibrate_pairs, raw_yaw_deg);
            if (calibration_count == o.calibrate_pairs) {
                visual_offset = calibration_sum / calibration_count;
                std::printf("VISUAL_OFFSET=%+.6f deg; movement logging starts now.\n",
                            visual_offset);
            }
            continue;
        }
        double yaw_deg = raw_yaw_deg - visual_offset;
        bool stationary = pixel_mae <= o.stationary_threshold;
        if (stationary) {
            visual_offset = (1.0 - o.offset_alpha) * visual_offset + o.offset_alpha * raw_yaw_deg;
            yaw_deg = 0.0;
        }
        accumulated += yaw_deg;
        double timestamp = std::chrono::duration<double>(npu_end - start).count();
        double npu_ms = std::chrono::duration<double, std::milli>(npu_end - npu_start).count();
        std::fprintf(log, "%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%d,%.3f\n", pair_index,
                     timestamp, raw_yaw_deg, visual_offset, yaw_deg, accumulated, pixel_mae,
                     stationary ? 1 : 0, npu_ms);
        std::fflush(log);
        std::printf("pair=%5d raw=%+8.3f yaw=%+8.3f deg accumulated=%+9.3f pixel=%.2f%s npu=%.3f ms\n",
                    pair_index, raw_yaw_deg, yaw_deg, accumulated, pixel_mae,
                    stationary ? " STILL" : "", npu_ms);

        if (!o.save_dir.empty() && o.save_every > 0 && pair_index % o.save_every == 0) {
            char pa[512], pb[512];
            std::snprintf(pa, sizeof(pa), "%s/pair_%05d_a.pgm", o.save_dir.c_str(), pair_index);
            std::snprintf(pb, sizeof(pb), "%s/pair_%05d_b.pgm", o.save_dir.c_str(), pair_index);
            save_pgm(pa, previous);
            save_pgm(pb, current);
        }
        previous = current.clone();
        ++pair_index;
    }

    std::printf("YAW_LIVE_DONE pairs=%d accumulated=%+.3f log=%s\n", pair_index, accumulated,
                o.log_path.c_str());
    std::fclose(log);
    CVI_NN_CleanupModel(model);
    SAMPLE_TDL_Destroy_MW_NO_RTSP(&mw_ctx);
    return 0;
}
