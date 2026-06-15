/*
 * pest_ai: thin C++ wrapper around ESP-DL dl::Model for YOLOv8n pest detection.
 *
 * The ONNX model was exported without DFL (see PestYOLO/export_onnx_esp32.py),
 * so DFL decode, bbox anchor decode, sigmoid, and NMS are all implemented here
 * in C++ on the ESP32-P4.
 *
 * Model I/O (INT8 quantized .espdl, 320x320):
 *   Input:  RGB float [1, 320, 320, 3] (NHWC interleaved), normalized to [0, 1]
 *   Outputs (6 independent tensors, per-scale, no cross-scale Concat):
 *     box0:   [1, 64, 1600]  stride 8   — bbox features (4 coords x reg_max=16)
 *     score0: [1, 15, 1600]  stride 8   — class logits
 *     box1:   [1, 64, 400]   stride 16
 *     score1: [1, 15, 400]   stride 16
 *     box2:   [1, 64, 100]   stride 32
 *     score2: [1, 15, 100]   stride 32
 *
 * Post-processing pipeline:
 *   1. Dequantize INT8 outputs → float
 *   2. DFL decode: softmax over 16 bins → 4 offsets per anchor
 *   3. Anchor decode: grid cell centre ± offset × stride → xyxy boxes
 *   4. Sigmoid on class logits
 *   5. Score threshold + per-class NMS
 *   6. Coordinate scaling: 320×320 → actual frame size
 *
 * Reference: face_ai.hpp for code patterns and namespace conventions.
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

// Forward-declare ESP-DL types so we don't drag heavy headers into every
// translation unit that includes us.
namespace dl {
class Model;
class TensorBase;
} // namespace dl

namespace p4fs {

/* ---- Data structures ------------------------------------------------ */

struct PestHit {
    int         x;          // top-left in RGB565 frame
    int         y;
    int         w;
    int         h;
    int         class_id;   // 0..14
    float       score;      // confidence [0, 1]
    std::string class_name; // Chinese name, e.g. "稻纵卷叶螟"
};

struct PestAiConfig {
    std::string model_path = "pest15";  // Flash partition label for .espdl
    float       score_thr  = 0.25f;     // minimum class confidence
    float       nms_thr    = 0.45f;     // IoU threshold for NMS
    float       cls_scale_restore = 1.0f; // restore ONNX Mul(scale) after dequantize
                                           // 1.0=normal; 4.0=ONNX had Mul(0.25) before cls
};

/* ---- PestAi class --------------------------------------------------- */

class PestAi {
public:
    explicit PestAi(const PestAiConfig &cfg);
    ~PestAi();

    /**
     * @brief Load the .espdl model from flash partition and allocate tensors.
     * @return true on success.
     */
    bool init();

    /**
     * @brief Run a self-test with an embedded pest image to verify the model.
     *
     * Feeds a synthetic 320×320 RGB565 image (aphids on leaf) through the
     * full detection pipeline and logs the results.  Called once at startup.
     *
     * @return true if at least one pest was detected.
     */
    bool self_test();

    /**
     * @brief Run pest detection on an RGB565 frame.
     *
     * Internally converts RGB565 → float RGB, resizes to 320×320,
     * runs the model, and post-processes outputs.
     *
     * @param rgb565  Pointer to RGB565LE pixel data (w * h * 2 bytes).
     * @param width   Frame width in pixels.
     * @param height  Frame height in pixels.
     * @return        Detected pests with coordinates in the input frame space.
     */
    std::vector<PestHit> process(const uint8_t *rgb565, int width, int height);

    // Stats for /api/info.
    uint32_t frames_processed() const { return frames_processed_; }
    uint32_t frames_with_pests() const { return frames_with_pests_; }

    // Latest detection summary as compact JSON.
    std::string last_event_json() const;

    // 15-class name table. Index matches model class ID.
    static const char *class_name_en(int id);
    static const char *class_name_cn(int id);
    static constexpr int kNumClasses = 15;

private:
    /* ---- YOLOv8 anchor constants (320×320 input) -------------------- */
    static constexpr int kModelSize  = 320;   // input width/height
    static constexpr int kRegMax     = 16;    // DFL bin count
    static constexpr int kNumScales  = 3;
    static constexpr int kStrides[3] = {8, 16, 32};
    static constexpr int kGridHW[3]  = {40, 20, 10};  // kModelSize / stride
    // Total anchors = 40² + 20² + 10² = 2100
    static constexpr int kNumAnchors = 40*40 + 20*20 + 10*10;

    PestAiConfig cfg_;
    dl::Model   *model_ = nullptr;

    // Pre-allocated float buffers for inference + post-processing.
    // All live in PSRAM; never on the stack — the camera callback task has
    // a small stack (4-8 KB) and 67 KB of stack arrays would overflow it.
    //
    // MEMORY LAYOUT (after the auto-transpose in process()):
    // All downstream code uses CHANNEL-MAJOR indexing:
    //   bbox_feat_[ch * kNumAnchors + a]   where ch in [0, 64)
    //   cls_logits_[c * kNumAnchors + a]   where c in [0, 15)
    //
    // The ESP-PPQ export may produce either NHWC (anchor-major) or NCHW
    // (channel-major) layout in the .espdl flatbuffer.  At runtime,
    // process() detects the actual tensor shape and auto-transposes from
    // NHWC to channel-major if needed, so the indexing above always works.
    float *input_float_   = nullptr;  // [1, 3, 320, 320] NHWC interleaved
    float *bbox_feat_     = nullptr;  // [64, kNumAnchors]  channel-major — dequantized bbox features
    float *cls_logits_    = nullptr;  // [kNumClasses, kNumAnchors]  class-major — dequantized logits
    float *dfl_buf_       = nullptr;  // [kNumAnchors, 4]    DFL decode output (also used as transpose temp)
    float *boxes_buf_     = nullptr;  // [kNumAnchors, 4]    anchor decode output
    float *scratch_buf_   = nullptr;  // [64*1600]           max single-scale dequant buffer

    uint32_t frames_processed_ = 0;
    uint32_t frames_with_pests_ = 0;
    mutable std::string last_event_json_;

    /* ---- Internal helpers ------------------------------------------- */

    /**
     * @brief Convert RGB565LE pixel data to float RGB planar input.
     * Also does letterbox/aspect-ratio handling if needed (currently
     * simple resize via nearest-neighbour for speed).
     */
    void prepare_input(const uint8_t *rgb565, int w, int h);

    /**
     * @brief Dequantize an INT8/INT16 tensor output into float buffer.
     * Preserves the ESP-DL memory layout (channel-major C-order).
     *
     * Supports both per-tensor and per-channel quantization.
     * When elems_per_channel > 0 and the tensor uses per-channel exponents,
     * each channel is dequantized with its own scale factor.
     *
     * @param tensor        ESP-DL output tensor (INT8 or INT16 quantized).
     * @param dst           Destination float buffer (n elements).
     * @param n             Total number of elements.
     * @param elems_per_channel  Elements per channel; set to 0 for per-tensor only.
     */
    void dequantize(const dl::TensorBase *tensor, float *dst, int n,
                    int elems_per_channel = 0);

    /**
     * @brief DFL decode: softmax + weighted sum over reg_max bins.
     * @param bbox_feat  [64, n_anchors] raw bbox features, channel-major.
     * @param dfl_out    [n_anchors, 4]  decoded offsets (l, t, r, b).
     */
    void dfl_decode(const float *bbox_feat, float *dfl_out);

    /**
     * @brief Convert DFL offsets + anchor grid → image-coordinate boxes.
     * @param dfl    [n_anchors, 4] DFL offsets.
     * @param boxes  [n_anchors, 4] output xyxy boxes in 320×320 space.
     */
    void anchor_decode(const float *dfl, float *boxes);

    /**
     * @brief Get candidates above score threshold.
     * @param cls_logits  [n_anchors, 15] class logits.
     * @param boxes       [n_anchors, 4]  xyxy boxes in 320×320 space.
     * @return            Raw candidates (unsorted, unfiltered except by score).
     */
    std::vector<PestHit> get_candidates(const float *cls_logits, const float *boxes);

    /**
     * @brief Per-class NMS, in-place sort + filter.
     * @param hits  Candidate list; survivors are kept, others removed.
     */
    void apply_nms(std::vector<PestHit> &hits);

    /**
     * @brief Scale 320×320 coordinates to frame coordinates.
     * @param hits     Hits with boxes in model space.
     * @param fw, fh   Actual frame dimensions.
     */
    static void scale_to_frame(std::vector<PestHit> &hits, int fw, int fh);
};

} // namespace p4fs
