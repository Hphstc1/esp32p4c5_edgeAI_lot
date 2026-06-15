/*
 * jpeg_annotate: draw rectangles + 5x7 ASCII labels into an RGB565 frame.
 *
 * Self-contained, no FreeType / LVGL / PNG dependency, so the MJPEG stream
 * gets visual face boxes at zero extra component cost.
 */
#pragma once

#include <cstdint>
#include <vector>

namespace p4fs {
struct FaceHit;  // from face_ai.hpp

namespace draw {

// RGB565 colour helpers.
static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

void hline(uint8_t *buf, int w, int h, int x, int y, int len, uint16_t c);
void vline(uint8_t *buf, int w, int h, int x, int y, int len, uint16_t c);
// Hollow rectangle, t pixels thick.
void rect(uint8_t *buf, int w, int h, int x, int y, int rw, int rh,
          int t, uint16_t c);
// Filled rectangle (for label background).
void fill_rect(uint8_t *buf, int w, int h, int x, int y, int rw, int rh, uint16_t c);
// 5x7 ASCII text. Returns width drawn. '?' is rendered for unsupported glyphs.
int  text_5x7(uint8_t *buf, int w, int h, int x, int y, const char *s, uint16_t fg, uint16_t bg);

// High-level: draws boxes + labels for all hits.
void annotate_faces(uint8_t *rgb565, int w, int h,
                    const std::vector<FaceHit> &hits);

} // namespace draw
} // namespace p4fs
