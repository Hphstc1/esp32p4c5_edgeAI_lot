/*
 * face_ai: thin C++ wrapper around espressif/human_face_recognition.
 *
 * The model component is what esp-who's who_recognition example uses under
 * the hood; calling it directly keeps our pipeline single-threaded and lets
 * us share a single RGB565 frame buffer with the JPEG annotator.
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

// Forward-declare the model classes so we don't have to drag the heavy
// esp-dl headers into every translation unit that includes us.
class HumanFaceDetect;
class HumanFaceRecognizer;
struct dl_detect_result_t;  // not actually needed; keep for doxygen

namespace p4fs {

struct FaceHit {
    int      x;        // top-left in RGB565 frame
    int      y;
    int      w;
    int      h;
    int      id;       // -1 if not recognized
    float    score;    // detection score
    float    sim;      // recognition similarity, NaN if N/A
    std::string name;  // "id:N" if recognized, "unknown" otherwise
};

struct FaceAiConfig {
    std::string db_path = "/spiffs/face.db";
    int         detect_min_score = 20;        // 0..100 (further lowered for debug)
    int         recog_min_score  = 20;        // 0..100 (further lowered for debug)
};

class FaceAi {
public:
    explicit FaceAi(const FaceAiConfig &cfg);
    ~FaceAi();

    bool init();

    // Run detect + recognize on an RGB565 frame (QVGA-ish size, w*h*2 bytes).
    // Returns the recognized/seen faces, with coordinates in the input frame.
    std::vector<FaceHit> process(const uint8_t *rgb565, int width, int height);

    // Add the largest face in the frame to the database. Returns new id (>=1)
    // on success or -1 on failure.
    int enroll_largest(const uint8_t *rgb565, int width, int height);

    // Drop the most recently enrolled face. Returns 0 on success.
    int delete_last();

    int num_enrolled() const;

    // Latest event as a compact JSON string (id, name, sim, score, x, y, w, h, ts_ms).
    std::string last_event_json() const;

    // Stats for /api/info.
    uint32_t frames_processed() const { return frames_processed_; }
    uint32_t frames_with_face() const { return frames_with_face_; }

private:
    FaceAiConfig cfg_;
    HumanFaceDetect    *detector_  = nullptr;
    HumanFaceRecognizer *recognizer_ = nullptr;
    std::string last_event_json_;
    uint32_t frames_processed_ = 0;
    uint32_t frames_with_face_ = 0;
};

} // namespace p4fs
