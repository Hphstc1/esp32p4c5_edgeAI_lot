/*
 * face_ai implementation.
 *
 * Notes on the espressif/human_face_* APIs:
 *   - HumanFaceDetect::run() returns std::list<dl::detect::result_t>
 *     where each result has box[4] = {x, y, x+w, y+h} and score.
 *   - HumanFaceRecognizer::recognize() returns std::vector<recognition_result_t>
 *     with id, similarity, etc. Empty vector means "no match".
 *   - recognize() / enroll() both need a face detection result because they
 *     crop the face from the original image.
 */
#include "face_ai.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <list>
#include <mutex>
#include <vector>

#include "esp_log.h"
#include "human_face_detect.hpp"
#include "human_face_recognition.hpp"

namespace p4fs {

static const char *TAG = "face_ai";

FaceAi::FaceAi(const FaceAiConfig &cfg) : cfg_(cfg) {}

FaceAi::~FaceAi() {
    delete recognizer_;
    delete detector_;
}

bool FaceAi::init() {
    detector_ = new HumanFaceDetect();
    recognizer_ = new HumanFaceRecognizer(cfg_.db_path.c_str());

    /* Apply user-configured score thresholds to the detector.
     * cfg_.detect_min_score is 0..100; convert to 0.0..1.0.
     * MSRMNP has two stages: idx=0 (MSR), idx=1 (MNP). */
    float thr = (float)cfg_.detect_min_score / 100.0f;
    detector_->set_score_thr(thr, 0);  // MSR stage
    detector_->set_score_thr(thr, 1);  // MNP stage

    /* Diagnostic: check if model was actually loaded and inspect quantization params */
    auto *raw = detector_->get_raw_model(0);  // try to get MSR model
    auto *raw2 = detector_->get_raw_model(1); // try to get MNP model
    if (raw) {
        auto *inp = raw->get_input();
        ESP_LOGI(TAG, "FaceAi: MSR input: [%d,%d,%d,%d] dtype=%d exponent=%d",
                 (int)inp->shape[0], (int)inp->shape[1], (int)inp->shape[2], (int)inp->shape[3],
                 (int)inp->dtype, (int)inp->exponent);
    }
    if (raw2) {
        auto *inp2 = raw2->get_input();
        ESP_LOGI(TAG, "FaceAi: MNP input: [%d,%d,%d,%d] dtype=%d exponent=%d",
                 (int)inp2->shape[0], (int)inp2->shape[1], (int)inp2->shape[2], (int)inp2->shape[3],
                 (int)inp2->dtype, (int)inp2->exponent);
    }
    ESP_LOGI(TAG, "FaceAi ready, db=%s, enrolled=%d, detect_thr=%.2f, "
             "msr_model=%p mnp_model=%p",
             cfg_.db_path.c_str(), recognizer_->get_num_feats(), thr,
             (void*)raw, (void*)raw2);
    return true;
}

static uint64_t now_ms() {
    return (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

static FaceHit make_unknown(int x, int y, int w, int h, float score) {
    FaceHit h_{};
    h_.x = x; h_.y = y; h_.w = w; h_.h = h;
    h_.id = -1;
    h_.score = score;
    h_.sim = NAN;
    h_.name = "unknown";
    return h_;
}

std::vector<FaceHit> FaceAi::process(const uint8_t *rgb565, int width, int height) {
    std::vector<FaceHit> out;
    if (!detector_ || !recognizer_ || !rgb565) return out;

    dl::image::img_t img;
    img.data   = const_cast<uint8_t *>(rgb565);
    img.width  = width;
    img.height = height;
    img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE;

    /* Diagnostic: dump first 8 pixels to verify RGB565LE data */
    if (frames_processed_ == 0) {
        const uint16_t *px = (const uint16_t *)rgb565;
        ESP_LOGI(TAG, "first 8 pixels (RGB565LE): %04x %04x %04x %04x %04x %04x %04x %04x",
                 (unsigned)px[0], (unsigned)px[1], (unsigned)px[2], (unsigned)px[3],
                 (unsigned)px[4], (unsigned)px[5], (unsigned)px[6], (unsigned)px[7]);
        /* Find min/max pixel value to check dynamic range */
        uint16_t pmin = 0xFFFF, pmax = 0;
        int nonzero = 0;
        for (int i = 0; i < width * height; i++) {
            if (px[i] < pmin) pmin = px[i];
            if (px[i] > pmax) pmax = px[i];
            if (px[i] != 0) nonzero++;
        }
        ESP_LOGI(TAG, "pixel range: min=%04x max=%04x nonzero=%d/%d",
                 (unsigned)pmin, (unsigned)pmax, nonzero, width*height);
        ESP_LOGI(TAG, "pixel[center]: %04x %04x %04x %04x",
                 (unsigned)px[height/2 * width + width/2],
                 (unsigned)px[height/2 * width + width/2 + 1],
                 (unsigned)px[height/2 * width + width/2 + 2],
                 (unsigned)px[height/2 * width + width/2 + 3]);
    }

    auto det_results = detector_->run(img);
    frames_processed_++;

    /* On very first run, check if model returned anything at all */
    if (frames_processed_ == 1) {
        ESP_LOGI(TAG, "first detection: %zu results, thr=%.2f",
                 det_results.size(), (float)cfg_.detect_min_score / 100.0f);
    }
    if (det_results.empty()) {
        /* Diagnostic: log every 10th empty result so we know detector is running */
        if ((frames_processed_ % 10) == 0) {
            ESP_LOGI(TAG, "process: frame %ux%u, processed=%u, 0 faces (thr=%.2f)",
                     (unsigned)width, (unsigned)height,
                     (unsigned)frames_processed_, (float)cfg_.detect_min_score / 100.0f);
        }
        return out;
    }
    frames_with_face_++;
    out.reserve(det_results.size());

    /* Diagnostic: log every detection with individual score */
    int di = 0;
    for (auto &d : det_results) {
        ESP_LOGI(TAG, "  det[%d]: box=[%d,%d %d,%d] score=%.4f",
                 di, (int)d.box[0], (int)d.box[1], (int)d.box[2], (int)d.box[3], d.score);
        di++;
    }
    ESP_LOGI(TAG, "process: %ux%u, %zu face(s), thr=%.2f",
             (unsigned)width, (unsigned)height,
             det_results.size(), (float)cfg_.detect_min_score / 100.0f);

    for (auto &d : det_results) {
        // dl::detect::result_t.box is {x0, y0, x1, y1} in input coords.
        int x0 = std::max(0, (int)d.box[0]);
        int y0 = std::max(0, (int)d.box[1]);
        int x1 = std::min(width,  (int)d.box[2]);
        int y1 = std::min(height, (int)d.box[3]);
        if (x1 <= x0 || y1 <= y0) continue;

        /* ESPDET doesn't output landmarks, but the recognizer needs
         * 5 facial keypoints (10 values: x,y pairs) for face alignment.
         * Synthesize from bounding box: left_eye, right_eye, nose,
         * left_mouth, right_mouth. */
        auto dd = d;
        if (dd.keypoint.size() != 10) {
            int cx = x0 + (x1 - x0) / 2;
            int cy = y0 + (y1 - y0) / 2;
            int w = x1 - x0, h = y1 - y0;
            dd.keypoint = {
                cx - w * 15 / 100, cy - h * 10 / 100,  // left eye
                cx + w * 15 / 100, cy - h * 10 / 100,  // right eye
                cx,                cy,                  // nose
                cx - w * 12 / 100, cy + h * 15 / 100,  // left mouth
                cx + w * 12 / 100, cy + h * 15 / 100   // right mouth
            };
        }

        // Build a one-element list to recognize just this face.
        std::list<dl::detect::result_t> one = { dd };
        auto rec = recognizer_->recognize(img, one);
        if (rec.empty()) {
            out.push_back(make_unknown(x0, y0, x1 - x0, y1 - y0, d.score));
        } else {
            FaceHit h_{};
            h_.x = x0; h_.y = y0;
            h_.w = x1 - x0; h_.h = y1 - y0;
            h_.id = rec[0].id;
            h_.sim = rec[0].similarity;
            h_.score = d.score;
            char buf[64];
            std::snprintf(buf, sizeof(buf), "id:%d", h_.id);
            h_.name = buf;
            out.push_back(h_);
        }
    }

    // Render a compact JSON of the highest-similarity recognized face (or
    // the first face if none recognized) as the "latest event" snapshot.
    FaceHit *best = nullptr;
    for (auto &h_ : out) {
        if (h_.id >= 0 && (!best || h_.sim > best->sim)) {
            best = &h_;
        }
    }
    if (!best && !out.empty()) best = &out[0];

    char json[256];
    if (best) {
        if (best->id >= 0) {
            std::snprintf(json, sizeof(json),
                "{\"ts\":%llu,\"id\":%d,\"name\":\"%s\",\"sim\":%.3f,"
                "\"score\":%.3f,\"x\":%d,\"y\":%d,\"w\":%d,\"h\":%d,"
                "\"faces\":%u}",
                (unsigned long long)now_ms(), best->id, best->name.c_str(),
                best->sim, best->score, best->x, best->y, best->w, best->h,
                (unsigned)out.size());
        } else {
            std::snprintf(json, sizeof(json),
                "{\"ts\":%llu,\"name\":\"unknown\",\"score\":%.3f,"
                "\"x\":%d,\"y\":%d,\"w\":%d,\"h\":%d,\"faces\":%u}",
                (unsigned long long)now_ms(), best->score,
                best->x, best->y, best->w, best->h, (unsigned)out.size());
        }
        last_event_json_ = json;
    }
    return out;
}

int FaceAi::enroll_largest(const uint8_t *rgb565, int width, int height) {
    if (!detector_ || !recognizer_) return -1;
    dl::image::img_t img;
    img.data   = const_cast<uint8_t *>(rgb565);
    img.width  = width;
    img.height = height;
    img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE;
    auto det = detector_->run(img);
    if (det.empty()) {
        ESP_LOGW(TAG, "enroll: no face in frame (%ux%u, thr=%.2f)",
                 (unsigned)width, (unsigned)height,
                 (float)cfg_.detect_min_score / 100.0f);
        return -1;
    }
    /* Log every candidate with score */
    int di = 0;
    for (auto &d : det) {
        ESP_LOGI(TAG, "enroll cand[%d]: box=[%d,%d %d,%d] score=%.4f",
                 di, (int)d.box[0], (int)d.box[1], (int)d.box[2], (int)d.box[3], d.score);
        di++;
    }
    ESP_LOGI(TAG, "enroll: %zu face candidate(s) in frame %ux%u",
             det.size(), (unsigned)width, (unsigned)height);
    auto biggest = std::max_element(det.begin(), det.end(),
        [](const dl::detect::result_t &a, const dl::detect::result_t &b) {
            const int aw = (a.box[2] - a.box[0]);
            const int ah = (a.box[3] - a.box[1]);
            const int bw = (b.box[2] - b.box[0]);
            const int bh = (b.box[3] - b.box[1]);
            return aw * ah < bw * bh;
        });
    int bx0 = (int)biggest->box[0], by0 = (int)biggest->box[1];
    int bx1 = (int)biggest->box[2], by1 = (int)biggest->box[3];
    ESP_LOGI(TAG, "enroll: biggest face [%d,%d %dx%d] score=%.3f",
             bx0, by0, bx1-bx0, by1-by0, biggest->score);
    /* ESPDET doesn't output landmarks; synthesize 5 facial keypoints */
    auto big = *biggest;
    if (big.keypoint.size() != 10) {
        int cx = bx0 + (bx1 - bx0) / 2;
        int cy = by0 + (by1 - by0) / 2;
        int w = bx1 - bx0, h = by1 - by0;
        big.keypoint = {
            cx - w * 15 / 100, cy - h * 10 / 100,
            cx + w * 15 / 100, cy - h * 10 / 100,
            cx,                cy,
            cx - w * 12 / 100, cy + h * 15 / 100,
            cx + w * 12 / 100, cy + h * 15 / 100
        };
    }
    std::list<dl::detect::result_t> one = { big };
    if (recognizer_->enroll(img, one) != ESP_OK) {
        ESP_LOGE(TAG, "enroll failed (recognizer)");
        return -1;
    }
    int id = recognizer_->get_num_feats();
    ESP_LOGI(TAG, "enrolled new face, total=%d", id);
    return id;
}

int FaceAi::delete_last() {
    if (!recognizer_) return -1;
    if (recognizer_->delete_last_feat() != ESP_OK) return -1;
    return 0;
}

int FaceAi::num_enrolled() const {
    return recognizer_ ? (int)recognizer_->get_num_feats() : 0;
}

std::string FaceAi::last_event_json() const {
    return last_event_json_;
}

} // namespace p4fs
