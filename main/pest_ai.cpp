1	/*
2	 * pest_ai implementation.
3	 *
4	 * YOLOv8n pest detection pipeline for ESP32-P4:
5	 *   RGB565 frame → float planar RGB → ESP-DL Model (INT8 .espdl)
6	 *   → DFL decode → anchor decode → sigmoid → NMS → frame coords.
7	 *
8	 * Model outputs (320×320, reg_max=16, 15 classes, 6 independent tensors):
9	 *   box0/score0  — stride 8  (40×40=1600 anchors)
10	 *   box1/score1  — stride 16 (20×20=400 anchors)
11	 *   box2/score2  — stride 32 (10×10=100 anchors)
12	 *
13	 * DFL was stripped during ONNX export; implemented here in C++.
14	 *
15	 * References:
16	 *   export_onnx_esp32.py  — model export logic
17	 *   face_ai.cpp           — ESP-DL integration patterns
18	 *   dl_model_base.hpp     — dl::Model API
19	 */
20	
21	#include "pest_ai.hpp"
22	
23	#include <algorithm>
24	#include <cmath>
25	#include <cstdio>
26	#include <cstring>
27	
28	#include "esp_log.h"
29	#include "esp_heap_caps.h"
30	#include "esp_task_wdt.h"
31	#include "esp_timer.h"
32	#include "dl_model_base.hpp"
33	
34	namespace p4fs {
35	
36	static const char *TAG = "pest_ai";
37	
38	/* ---- 15-class name tables ------------------------------------------- */
39	
40	static const char *kEnNames[PestAi::kNumClasses] = {
41	    "rice leaf roller",        // 0
42	    "yellow rice borer",       // 1
43	    "brown plant hopper",      // 2
44	    "white backed plant hopper", // 3
45	    "small brown plant hopper",  // 4
46	    "rice leafhopper",         // 5
47	    "corn borer",              // 6
48	    "army worm",               // 7
49	    "aphids",                  // 8
50	    "black cutworm",           // 9
51	    "red spider",              // 10
52	    "flea beetle",             // 11
53	    "cabbage army worm",       // 12
54	    "beet army worm",          // 13
55	    "Prodenia litura",         // 14
56	};
57	
58	static const char *kCnNames[PestAi::kNumClasses] = {
59	    "\xe7\xa8\xbb\xe7\xba\xb5\xe5\x8d\xb7\xe5\x8f\xb6\xe8\x9e\x9e",  // 稻纵卷叶螟
60	    "\xe9\xbb\x84\xe7\xa8\xbb\xe8\x9e\x9e",                          // 黄稻螟
61	    "\xe8\xa4\x90\xe9\xa3\x9e\xe8\x99\xb1",                          // 褐飞虱
62	    "\xe7\x99\xbd\xe8\x83\x8c\xe9\xa3\x9e\xe8\x99\xb1",              // 白背飞虱
63	    "\xe7\x81\xb0\xe9\xa3\x9e\xe8\x99\xb1",                          // 灰飞虱
64	    "\xe7\xa8\xbb\xe5\x8f\xb6\xe8\x9d\x89",                          // 稻叶蝉
65	    "\xe7\x8e\x89\xe7\xb1\xb3\xe8\x9e\x9e",                          // 玉米螟
66	    "\xe9\xbb\x8f\xe8\x99\xab",                                       // 黏虫
67	    "\xe8\x9a\x9c\xe8\x99\xab",                                       // 蚜虫
68	    "\xe9\xbb\x91\xe5\x88\x87\xe6\xa0\xb9\xe8\x99\xab",              // 黑切根虫
69	    "\xe7\xba\xa2\xe8\x9c\x98\xe8\x9b\x9b",                          // 红蜘蛛
70	    "\xe8\xb7\xb3\xe7\x94\xb2",                                       // 跳甲
71	    "\xe8\x8f\x9c\xe9\x9d\x92\xe8\x99\xab",                          // 菜青虫
72	    "\xe7\x94\x9c\xe8\x8f\x9c\xe5\xa4\x9c\xe8\x9b\xbe",              // 甜菜夜蛾
73	    "\xe6\x96\x9c\xe7\xba\xb9\xe5\xa4\x9c\xe8\x9b\xbe",              // 斜纹夜蛾
74	};
75	
76	const char *PestAi::class_name_en(int id) {
77	    if (id < 0 || id >= kNumClasses) return "unknown";
78	    return kEnNames[id];
79	}
80	
81	const char *PestAi::class_name_cn(int id) {
82	    if (id < 0 || id >= kNumClasses) return "\xe6\x9c\xaa\xe7\x9f\xa5"; // 未知
83	    return kCnNames[id];
84	}
85	
86	/* ---- Construction / destruction ------------------------------------- */
87	
88	PestAi::PestAi(const PestAiConfig &cfg) : cfg_(cfg) {}
89	
90	PestAi::~PestAi() {
91	    if (model_) {
92	        delete model_;
93	        model_ = nullptr;
94	    }
95	    if (input_float_) {
96	        heap_caps_free(input_float_);
97	        input_float_ = nullptr;
98	    }
99	    if (bbox_feat_) {
100	        heap_caps_free(bbox_feat_);
101	        bbox_feat_ = nullptr;
102	    }
103	    if (cls_logits_) {
104	        heap_caps_free(cls_logits_);
105	        cls_logits_ = nullptr;
106	    }
107	    if (dfl_buf_) {
108	        heap_caps_free(dfl_buf_);
109	        dfl_buf_ = nullptr;
110	    }
111	    if (boxes_buf_) {
112	        heap_caps_free(boxes_buf_);
113	        boxes_buf_ = nullptr;
114	    }
115	    if (scratch_buf_) {
116	        heap_caps_free(scratch_buf_);
117	        scratch_buf_ = nullptr;
118	    }
119	}
120	
121	/* ---- Initialisation ------------------------------------------------- */
122	
123	bool PestAi::init() {
124	    /* --- Load .espdl model from flash partition --- */
125	    model_ = new dl::Model(
126	        cfg_.model_path.c_str(),
127	        fbs::MODEL_LOCATION_IN_FLASH_PARTITION
128	    );
129	
130	    if (!model_) {
131	        ESP_LOGE(TAG, "Failed to create dl::Model");
132	        return false;
133	    }
134	
135	    /* --- Allocate float buffers in PSRAM --- */
136	    constexpr int input_sz  = 1 * 3 * kModelSize * kModelSize;  // 307,200 floats
137	    constexpr int bbox_sz   = kNumAnchors * kRegMax * 4;         // 134,400 floats
138	    constexpr int cls_sz    = kNumAnchors * kNumClasses;          // 31,500 floats
139	    constexpr int dfl_sz    = kNumAnchors * 4;                    // 8,400 floats
140	    constexpr int boxes_sz  = kNumAnchors * 4;                    // 8,400 floats
141	    constexpr int scratch_sz = 64 * 1600;                         // 102,400 floats (max single-scale)
142	
143	    input_float_ = (float *)heap_caps_malloc(input_sz * sizeof(float),
144	                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
145	    bbox_feat_   = (float *)heap_caps_malloc(bbox_sz * sizeof(float),
146	                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
147	    cls_logits_  = (float *)heap_caps_malloc(cls_sz * sizeof(float),
148	                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
149	    dfl_buf_     = (float *)heap_caps_malloc(dfl_sz * sizeof(float),
150	                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
151	    boxes_buf_   = (float *)heap_caps_malloc(boxes_sz * sizeof(float),
152	                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
153	    scratch_buf_ = (float *)heap_caps_malloc(scratch_sz * sizeof(float),
154	                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
155	
156	    if (!input_float_ || !bbox_feat_ || !cls_logits_ || !dfl_buf_ || !boxes_buf_ || !scratch_buf_) {
157	        ESP_LOGE(TAG, "PSRAM alloc failed: input=%p bbox=%p cls=%p dfl=%p boxes=%p scratch=%p",
158	                 (void *)input_float_, (void *)bbox_feat_, (void *)cls_logits_,
159	                 (void *)dfl_buf_, (void *)boxes_buf_, (void *)scratch_buf_);
160	        return false;
161	    }
162	
163	    /* --- Diagnostic: inspect model I/O (6 independent outputs) --- */
164	    auto *inp = model_->get_input("images");
165	    if (inp) {
166	        ESP_LOGI(TAG, "Model input [images]: shape=[%d,%d,%d,%d] dtype=%d exponent=%d",
167	                 (int)inp->shape[0], (int)inp->shape[1],
168	                 (int)inp->shape[2], (int)inp->shape[3],
169	                 (int)inp->dtype, (int)inp->exponent);
170	    }
171	    if (inp && inp->exponent.is_per_channel()) {
172	        ESP_LOGW(TAG, "Model input [images] uses PER-CHANNEL quantization: %d channels",
173	                 inp->exponent.channel_size());
174	    }
175	    const char *out_names[] = {"box0", "score0", "box1", "score1", "box2", "score2"};
176	    for (int i = 0; i < 6; i++) {
177	        auto *out = model_->get_output(out_names[i]);
178	        if (out) {
179	            ESP_LOGI(TAG, "Model output [%s]: shape=[%d,%d,%d] dtype=%d exponent=%d%s",
180	                     out_names[i],
181	                     (int)out->shape[0],
182	                     (int)out->shape.size() > 1 ? (int)out->shape[1] : -1,
183	                     (int)out->shape.size() > 2 ? (int)out->shape[2] : -1,
184	                     (int)out->dtype, (int)out->exponent,
185	                     out->exponent.is_per_channel() ? " PER-CHANNEL" : "");
186	        } else {
187	            ESP_LOGW(TAG, "Model output [%s]: NOT FOUND — check ONNX export names", out_names[i]);
188	        }
189	    }
190	
191	    ESP_LOGI(TAG, "PestAi ready: model=%s score_thr=%.2f nms_thr=%.2f bufs=%.0f KB",
192	             cfg_.model_path.c_str(), cfg_.score_thr, cfg_.nms_thr,
193	             (input_sz + bbox_sz + cls_sz + dfl_sz + boxes_sz + scratch_sz) * 4.0 / 1024.0);
194	
195	    return true;
196	}
197	
198	/* ---- Self-test with embedded pest image ------------------------------- */
199	/*
200	 * The embedded image (test_pest_image.rgb565) is a synthetic 320×320 scene
201	 * with a green leaf-like background and multiple dark insect-shaped blobs.
202	 * It is NOT a real photograph — a real IP102 training image would be better,
203	 * but network restrictions prevent downloading one at build time.
204	 *
205	 * Even a synthetic image is diagnostically useful: if the model's class head
206	 * is alive, the logit distribution should CHANGE when we swap the camera
207	 * input for a completely different synthetic image.  If it stays identical,
208	 * the model is fundamentally broken (not processing input at all).
209	 *
210	 * Linked via COMPONENT_EMBED_FILES; the build system auto-generates the
211	 * _binary_... symbols.
212	 */
213	extern "C" {
214	extern const uint8_t _binary_test_pest_image_rgb565_start[];
215	extern const uint8_t _binary_test_pest_image_rgb565_end[];
216	}
217	
218	bool PestAi::self_test() {
219	    if (!model_) {
220	        ESP_LOGE(TAG, "SELF_TEST: model not loaded, skip");
221	        return false;
222	    }
223	
224	    const uint8_t *img = _binary_test_pest_image_rgb565_start;
225	    ptrdiff_t sz = _binary_test_pest_image_rgb565_end
226	                 - _binary_test_pest_image_rgb565_start;
227	
228	    if (sz != 204800) {
229	        ESP_LOGE(TAG, "SELF_TEST: unexpected image size %d, expected 204800", (int)sz);
230	        return false;
231	    }
232	
233	    constexpr int kTestW = 320, kTestH = 320;
234	    ESP_LOGI(TAG, "══════════════ SELF-TEST START (synthetic pest image) ══════════════");
235	    ESP_LOGI(TAG, "SELF_TEST: image %d×%d RGB565LE, %d bytes",
236	             kTestW, kTestH, (int)sz);
237	
238	    // Run detection at configured threshold — self-test must reflect real behaviour.
239	    // We save and restore frames_processed_ so self-test doesn't consume the
240	    // debug-log quota for real camera frames.
241	    uint32_t saved_frames = frames_processed_;
242	    frames_processed_ = 0;  // Trick process() into thinking these are the first frames
243	
244	    auto hits = process(img, kTestW, kTestH);
245	
246	    frames_processed_ = saved_frames;  // Restore
247	
248	    ESP_LOGI(TAG, "SELF_TEST: %zu detection(s) at thr=%.2f",
249	             hits.size(), cfg_.score_thr);
250	
251	    if (hits.empty()) {
252	        ESP_LOGE(TAG, "SELF_TEST: ⛔ NO PESTS DETECTED — model classification head "
253	                       "is not responding");
254	        ESP_LOGI(TAG, "══════════════ SELF-TEST END (FAIL) ══════════════");
255	        return false;
256	    }
257	
258	    // Log each detection
259	    for (size_t i = 0; i < hits.size(); i++) {
260	        ESP_LOGI(TAG, "SELF_TEST:   [%d] cls=%d(%s) score=%.3f box=[%d,%d %d×%d]",
261	                 (int)i, hits[i].class_id, hits[i].class_name.c_str(),
262	                 hits[i].score, hits[i].x, hits[i].y, hits[i].w, hits[i].h);
263	    }
264	
265	    ESP_LOGI(TAG, "SELF_TEST: ✅ Model works! %zu pest(s) detected on synthetic image",
266	             hits.size());
267	    ESP_LOGI(TAG, "══════════════ SELF-TEST END (PASS) ══════════════");
268	    return true;
269	}
270	
271	/* ---- Main processing loop ------------------------------------------- */
272	
273	std::vector<PestHit> PestAi::process(const uint8_t *rgb565, int width, int height) {
274	    std::vector<PestHit> out;
275	    if (!model_ || !rgb565 || width <= 0 || height <= 0) return out;
276	
277	    uint64_t t0 = esp_timer_get_time();
278	
279	    // 1. Convert RGB565 → float planar RGB (320×320)
280	    prepare_input(rgb565, width, height);
281	
282	    // Debug: input float statistics (first 5 frames)
283	    if (frames_processed_ < 5) {
284	        float imin = input_float_[0], imax = input_float_[0];
285	        double isum = 0.0;
286	        constexpr int input_sz = 1 * 3 * kModelSize * kModelSize;
287	        for (int i = 0; i < input_sz; i++) {
288	            float v = input_float_[i];
289	            if (v < imin) imin = v;
290	            if (v > imax) imax = v;
291	            isum += v;
292	        }
293	        ESP_LOGI(TAG, "frame %u: input float min=%.3f max=%.3f mean=%.3f",
294	                 (unsigned)(frames_processed_ + 1), imin, imax,
295	                 (float)(isum / (double)input_sz));
296	    }
297	
298	    // 2. Feed input to model
299	    auto *input_tensor = model_->get_input("images");
300	    if (!input_tensor) {
301	        ESP_LOGE(TAG, "Missing input tensor");
302	        return out;
303	    }
304	
305	    // Input tensor is INT8 (dtype=3, exponent=-7).  get_element_ptr<T>() is
306	    // a raw (T*)data cast — writing float to an INT8 tensor would:
307	    //   1. Write 4× the data (buffer overflow past 307,200 bytes)
308	    //   2. Feed the model random noise (first byte of float mantissa)
309	    // Quantize each pixel: INT8 = round( float * 2^(-exponent) )
310	    constexpr int input_sz = 1 * 3 * kModelSize * kModelSize;
311	    int8_t *input_ptr = input_tensor->get_element_ptr<int8_t>();
312	    float inv_scale = ldexpf(1.0f, -input_tensor->exponent.get());  // 128.0f
313	    for (int i = 0; i < input_sz; i++) {
314	        input_ptr[i] = dl::quantize<int8_t, float>(input_float_[i], inv_scale);
315	    }
316	
317	    // Debug: quantized INT8 input statistics (first 5 frames)
318	    if (frames_processed_ < 5) {
319	        int8_t qmin = input_ptr[0], qmax = input_ptr[0];
320	        int buckets[5] = {0};  // [-128,-1], 0, [1,32], [33,96], [97,127]
321	        for (int i = 0; i < input_sz; i++) {
322	            int8_t v = input_ptr[i];
323	            if (v < qmin) qmin = v;
324	            if (v > qmax) qmax = v;
325	            if (v < 0) buckets[0]++;
326	            else if (v == 0) buckets[1]++;
327	            else if (v <= 32) buckets[2]++;
328	            else if (v <= 96) buckets[3]++;
329	            else buckets[4]++;
330	        }
331	        ESP_LOGI(TAG, "frame %u: INT8 in  min=%d max=%d  hist: <0=%d 0=%d 1-32=%d 33-96=%d 97-127=%d",
332	                 (unsigned)(frames_processed_ + 1), (int)qmin, (int)qmax,
333	                 buckets[0], buckets[1], buckets[2], buckets[3], buckets[4]);
334	    }
335	
336	    // 3. Run inference — pest model takes ~3 s on ESP32-P4.
337	    // Unsubscribe from the task watchdog while blocked inside the ESP-DL
338	    // inference engine; otherwise IDLE1 can't run and TWDT fires.
339	    esp_task_wdt_delete(NULL);
340	    model_->run();
341	    esp_task_wdt_add(NULL);
342	
343	    frames_processed_++;
344	
345	    // 4. Dequantize 6 independent outputs → 2 channel-major buffers
346	    //    Old: output_bbox [1,64,2100] + output_cls [1,15,2100] (concatenated)
347	    //    New: box0/score0 (1600a) + box1/score1 (400a) + box2/score2 (100a)
348	    constexpr int kAnchorOffsets[4] = {0, 1600, 2000, 2100};  // cumulative anchors
349	    constexpr int kScaleAnchors[3] = {1600, 400, 100};
350	    const char *box_names[]  = {"box0",  "box1",  "box2"};
351	    const char *score_names[]= {"score0","score1","score2"};
352	
353	    bool any_missing = false;
354	    for (int s = 0; s < kNumScales; s++) {
355	        auto *b = model_->get_output(box_names[s]);
356	        auto *c = model_->get_output(score_names[s]);
357	        if (!b || !c) {
358	            ESP_LOGE(TAG, "Missing output: %s or %s", box_names[s], score_names[s]);
359	            any_missing = true;
360	            continue;
361	        }
362	
363	        int n_a = kScaleAnchors[s];  // anchors this scale
364	        int a_off = kAnchorOffsets[s]; // start index in 2100-buffer
365	
366	        // --- Dequantize box and copy into bbox_feat_ [64, 2100] channel-major ---
367	        {
368	            int box_elems = 1;
369	            for (auto d : b->shape) box_elems *= (int)d;
370	            bool is_nchw = (b->shape.size() >= 3 && b->shape[1] == 64 && b->shape[2] == (uint32_t)n_a);
371	            bool is_nhwc = (b->shape.size() >= 3 && b->shape[1] == (uint32_t)n_a && b->shape[2] == 64);
372	
373	            // Use pre-allocated scratch_buf_ (102,400 floats = 400 KB)
374	            dequantize(b, scratch_buf_, box_elems);
375	
376	            if (is_nchw) {
377	                // NCHW: [64, N_a] → copy each channel's contiguous block
378	                for (int ch = 0; ch < 64; ch++) {
379	                    const float *src = scratch_buf_ + ch * n_a;
380	                    float *dst = bbox_feat_ + ch * kNumAnchors + a_off;
381	                    memcpy(dst, src, n_a * sizeof(float));
382	                }
383	            } else if (is_nhwc) {
384	                // NHWC: [N_a, 64] → transpose
385	                for (int a = 0; a < n_a; a++) {
386	                    for (int ch = 0; ch < 64; ch++) {
387	                        bbox_feat_[ch * kNumAnchors + a_off + a] = scratch_buf_[a * 64 + ch];
388	                    }
389	                }
390	            } else {
391	                ESP_LOGW(TAG, "Unexpected %s shape, assuming [64,%d]", box_names[s], n_a);
392	                memcpy(bbox_feat_ + a_off, scratch_buf_, box_elems * sizeof(float));
393	            }
394	        }
395	
396	        // --- Dequantize score and copy into cls_logits_ [15, 2100] class-major ---
397	        {
398	            int cls_elems = 1;
399	            for (auto d : c->shape) cls_elems *= (int)d;
400	            bool is_nchw = (c->shape.size() >= 3 && c->shape[1] == kNumClasses && c->shape[2] == (uint32_t)n_a);
401	            bool is_nhwc = (c->shape.size() >= 3 && c->shape[1] == (uint32_t)n_a && c->shape[2] == kNumClasses);
402	
403	            // Reuse scratch_buf_ (15*1600=24,000 floats, well within 102,400 limit)
404	            dequantize(c, scratch_buf_, cls_elems);
405	
406	            if (is_nchw) {
407	                // NCHW: [15, N_a] → copy each class's contiguous block
408	                for (int cl = 0; cl < kNumClasses; cl++) {
409	                    const float *src = scratch_buf_ + cl * n_a;
410	                    float *dst = cls_logits_ + cl * kNumAnchors + a_off;
411	                    memcpy(dst, src, n_a * sizeof(float));
412	                }
413	            } else if (is_nhwc) {
414	                // NHWC: [N_a, 15] → transpose
415	                for (int a = 0; a < n_a; a++) {
416	                    for (int cl = 0; cl < kNumClasses; cl++) {
417	                        cls_logits_[cl * kNumAnchors + a_off + a] = scratch_buf_[a * kNumClasses + cl];
418	                    }
419	                }
420	            } else {
421	                ESP_LOGW(TAG, "Unexpected %s shape, assuming [15,%d]", score_names[s], n_a);
422	                memcpy(cls_logits_ + a_off, scratch_buf_, cls_elems * sizeof(float));
423	            }
424	        }
425	    }
426	
427	    if (any_missing) return out;
428	
429	    /* Diagnostic: log output value ranges on first few frames */
430	    if (frames_processed_ <= 3) {
431	        constexpr int bbox_sz = kNumAnchors * kRegMax * 4;
432	        constexpr int cls_sz  = kNumAnchors * kNumClasses;
433	        float bmin = bbox_feat_[0], bmax = bbox_feat_[0];
434	        float cmin = cls_logits_[0], cmax = cls_logits_[0];
435	        for (int i = 1; i < bbox_sz; i++) {
436	            if (bbox_feat_[i] < bmin) bmin = bbox_feat_[i];
437	            if (bbox_feat_[i] > bmax) bmax = bbox_feat_[i];
438	        }
439	        for (int i = 1; i < cls_sz; i++) {
440	            if (cls_logits_[i] < cmin) cmin = cls_logits_[i];
441	            if (cls_logits_[i] > cmax) cmax = cls_logits_[i];
442	        }
443	        ESP_LOGI(TAG, "frame %u: bbox [%.4f,%.4f] cls [%.4f,%.4f]",
444	                 (unsigned)frames_processed_, bmin, bmax, cmin, cmax);
445	
446	        // Per-coordinate-group DFL feature ranges (4 groups × 16 bins each)
447	        // bbox_feat_ is [64, 2100] channel-major; each coord group = 16 consecutive channels
448	        for (int cg = 0; cg < 4; cg++) {
449	            float gmin = bbox_feat_[cg * 16 * kNumAnchors];
450	            float gmax = gmin;
451	            for (int bin = 0; bin < 16; bin++) {
452	                int ch = cg * 16 + bin;
453	                for (int a = 0; a < kNumAnchors; a++) {
454	                    float v = bbox_feat_[ch * kNumAnchors + a];
455	                    if (v < gmin) gmin = v;
456	                    if (v > gmax) gmax = v;
457	                }
458	            }
459	            ESP_LOGI(TAG, "frame %u:   bbox grp%d (ch%d-%d) [%.4f,%.4f]",
460	                     (unsigned)frames_processed_, cg, cg*16, cg*16+15, gmin, gmax);
461	        }
462	
463	        // Count anchors with at least one positive class logit (first frame only)
464	        if (frames_processed_ == 1) {
465	            int pos_any = 0;
466	            int pos_strong = 0;  // max logit > 2.0 → sigmoid > 0.88
467	            for (int a = 0; a < kNumAnchors; a++) {
468	                float best = cls_logits_[0 * kNumAnchors + a];
469	                for (int c = 1; c < kNumClasses; c++) {
470	                    float v = cls_logits_[c * kNumAnchors + a];
471	                    if (v > best) best = v;
472	                }
473	                if (best > 0.0f) pos_any++;
474	                if (best > 2.0f) pos_strong++;
475	            }
476	            ESP_LOGI(TAG, "frame %u: cls  positive-logit anchors: %d/2100  strong(>2.0): %d",
477	                     (unsigned)frames_processed_, pos_any, pos_strong);
478	        }
479	    }
480	
481	    // 5. DFL decode: [64, 2100] channel-major → [2100, 4]
482	    dfl_decode(bbox_feat_, dfl_buf_);
483	
484	    // Debug: DFL offset range (first 5 frames)
485	    if (frames_processed_ <= 5) {
486	        float dmin[4] = {dfl_buf_[0], dfl_buf_[1], dfl_buf_[2], dfl_buf_[3]};
487	        float dmax[4] = {dmin[0], dmin[1], dmin[2], dmin[3]};
488	        for (int a = 1; a < kNumAnchors; a++) {
489	            for (int c = 0; c < 4; c++) {
490	                float v = dfl_buf_[a * 4 + c];
491	                if (v < dmin[c]) dmin[c] = v;
492	                if (v > dmax[c]) dmax[c] = v;
493	            }
494	        }
495	        ESP_LOGI(TAG, "frame %u: DFL offsets  L[%.2f,%.2f] T[%.2f,%.2f] R[%.2f,%.2f] B[%.2f,%.2f]",
496	                 (unsigned)frames_processed_,
497	                 dmin[0], dmax[0], dmin[1], dmax[1], dmin[2], dmax[2], dmin[3], dmax[3]);
498	    }
499	
500	    // 6. Anchor decode: [2100, 4] → [2100, 4] xyxy boxes in 320×320 space
501	    anchor_decode(dfl_buf_, boxes_buf_);
502	
503	    // Debug: box coordinate range (first 5 frames)
504	    if (frames_processed_ <= 5) {
505	        float x1min = boxes_buf_[0], y1min = boxes_buf_[1];
506	        float x2min = boxes_buf_[2], y2min = boxes_buf_[3];
507	        float x1max = x1min, y1max = y1min, x2max = x2min, y2max = y2min;
508	        for (int a = 1; a < kNumAnchors; a++) {
509	            float x1 = boxes_buf_[a * 4 + 0];
510	            float y1 = boxes_buf_[a * 4 + 1];
511	            float x2 = boxes_buf_[a * 4 + 2];
512	            float y2 = boxes_buf_[a * 4 + 3];
513	            if (x1 < x1min) x1min = x1;
514	            if (x1 > x1max) x1max = x1;
515	            if (y1 < y1min) y1min = y1;
516	            if (y1 > y1max) y1max = y1;
517	            if (x2 < x2min) x2min = x2;
518	            if (x2 > x2max) x2max = x2;
519	            if (y2 < y2min) y2min = y2;
520	            if (y2 > y2max) y2max = y2;
521	        }
522	        ESP_LOGI(TAG, "frame %u: boxes  x1[%.1f,%.1f] y1[%.1f,%.1f] x2[%.1f,%.1f] y2[%.1f,%.1f]",
523	                 (unsigned)frames_processed_,
524	                 x1min, x1max, y1min, y1max, x2min, x2max, y2min, y2max);
525	    }
526	
527	    // 7. Score threshold + sigmoid → candidates
528	    out = get_candidates(cls_logits_, boxes_buf_);
529	
530	    // 8. Per-class NMS
531	    apply_nms(out);
532	
533	    // 9. Scale to frame coordinates
534	    scale_to_frame(out, width, height);
535	
536	    uint32_t elapsed = (uint32_t)((esp_timer_get_time() - t0) / 1000);
537	
538	    if (!out.empty()) {
539	        frames_with_pests_++;
540	        ESP_LOGI(TAG, "DETECT frame=%u: %zu pest(s) in %ums",
541	                 (unsigned)frames_processed_, out.size(), (unsigned)elapsed);
542	        for (size_t i = 0; i < out.size(); i++) {
543	            ESP_LOGI(TAG, "  [%d] cls=%d(%s) score=%.3f box=[%d,%d %dx%d]",
544	                     (int)i, out[i].class_id, out[i].class_name.c_str(),
545	                     out[i].score, out[i].x, out[i].y, out[i].w, out[i].h);
546	        }
547	    } else {
548	        // silent: no pest on this frame
549	    }
550	
551	    // 10. Build JSON summary from the highest-score hit
552	    const PestHit *best = nullptr;
553	    for (auto &h : out) {
554	        if (!best || h.score > best->score) best = &h;
555	    }
556	    if (best) {
557	        char json[256];
558	        std::snprintf(json, sizeof(json),
559	            "{\"pests\":%u,\"top\":{\"cls\":%d,\"name\":\"%s\","
560	            "\"score\":%.3f,\"x\":%d,\"y\":%d,\"w\":%d,\"h\":%d}}",
561	            (unsigned)out.size(), best->class_id, best->class_name.c_str(),
562	            best->score, best->x, best->y, best->w, best->h);
563	        last_event_json_ = json;
564	    }
565	
566	    return out;
567	}
568	
569	/* ---- Input preparation: RGB565LE → float interleaved RGB (NHWC) ----- */
570	/*
571	 * ESP-DL officially requires NHWC (interleaved) memory layout.
572	 * ESP-PPQ converts PyTorch NCHW → NHWC during .espdl export, inserting
573	 * transpose nodes and resetting convolution parameter layouts.
574	 *
575	 * Model input shape: [1, 320, 320, 3] → NHWC.
576	 * Memory layout:  R,G,B for pixel (0,0), then R,G,B for pixel (0,1), ...
577	 *   offset(y, x, c) = (y * 320 + x) * 3 + c
578	 */
579	
580	void PestAi::prepare_input(const uint8_t *rgb565, int w, int h) {
581	    const uint16_t *src = reinterpret_cast<const uint16_t *>(rgb565);
582	
583	    // Nearest-neighbour resize + RGB565→float conversion in one pass.
584	    for (int y = 0; y < kModelSize; y++) {
585	        int src_y = y * h / kModelSize;
586	        if (src_y >= h) src_y = h - 1;
587	
588	        for (int x = 0; x < kModelSize; x++) {
589	            int src_x = x * w / kModelSize;
590	            if (src_x >= w) src_x = w - 1;
591	
592	            uint16_t px = src[src_y * w + src_x];
593	            float r = (float)((px >> 11) & 0x1F) / 31.0f;
594	            float g = (float)((px >> 5)  & 0x3F) / 63.0f;
595	            float b = (float)( px        & 0x1F) / 31.0f;
596	
597	            int idx = (y * kModelSize + x) * 3;
598	            input_float_[idx + 0] = r;
599	            input_float_[idx + 1] = g;
600	            input_float_[idx + 2] = b;
601	        }
602	    }
603	}
604	
605	/* ---- Dequantization ------------------------------------------------- */
606	/*
607	 * Dequantizes an ESP-DL output tensor to float.
608	 *
609	 * Handles both per-tensor and per-channel quantization:
610	 *   Per-tensor:  one scale for all elements (exponent.get())
611	 *   Per-channel: each channel has its own exponent (exponent.get(ch))
612	 *
613	 * When elems_per_channel > 0 and per-channel is detected, each channel
614	 * of elems_per_channel contiguous elements is scaled independently.
615	 * This is critical for classification heads where each class may have
616	 * a different quantization scale.
617	 */
618	
619	void PestAi::dequantize(const dl::TensorBase *tensor, float *dst, int n,
620	                        int elems_per_channel) {
621	    if (!tensor || !dst) return;
622	
623	    dl::dtype_t dt = tensor->dtype;
624	    bool per_ch = tensor->exponent.is_per_channel() && (elems_per_channel > 0);
625	    // get_element_ptr() is non-const; const_cast is safe here (we only read)
626	    const void *src = const_cast<dl::TensorBase *>(tensor)->get_element_ptr();
627	
628	    if (per_ch) {
629	        int nch = n / elems_per_channel;
630	        switch (dt) {
631	        case dl::DATA_TYPE_INT8: {
632	            const int8_t *s = static_cast<const int8_t *>(src);
633	            for (int ch = 0; ch < nch; ch++) {
634	                float scale = ldexpf(1.0f, tensor->exponent.get(ch));
635	                int base = ch * elems_per_channel;
636	                for (int i = 0; i < elems_per_channel; i++) {
637	                    dst[base + i] = (float)s[base + i] * scale;
638	                }
639	            }
640	            break;
641	        }
642	        case dl::DATA_TYPE_INT16: {
643	            const int16_t *s = static_cast<const int16_t *>(src);
644	            for (int ch = 0; ch < nch; ch++) {
645	                float scale = ldexpf(1.0f, tensor->exponent.get(ch));
646	                int base = ch * elems_per_channel;
647	                for (int i = 0; i < elems_per_channel; i++) {
648	                    dst[base + i] = (float)s[base + i] * scale;
649	                }
650	            }
651	            break;
652	        }
653	        default:
654	            ESP_LOGW(TAG, "Per-channel dequant unsupported dtype=%d, falling back",
655	                     (int)dt);
656	            per_ch = false;  // fall through to per-tensor below
657	            break;
658	        }
659	    }
660	
661	    if (!per_ch) {
662	        // Per-tensor: single scale for all elements
663	        float scale = ldexpf(1.0f, (int)tensor->exponent);
664	
665	        switch (dt) {
666	        case dl::DATA_TYPE_FLOAT:
667	            std::memcpy(dst, src, (size_t)n * sizeof(float));
668	            break;
669	        case dl::DATA_TYPE_INT8: {
670	            const int8_t *s = static_cast<const int8_t *>(src);
671	            for (int i = 0; i < n; i++) {
672	                dst[i] = (float)s[i] * scale;
673	            }
674	            break;
675	        }
676	        case dl::DATA_TYPE_UINT8: {
677	            const uint8_t *s = static_cast<const uint8_t *>(src);
678	            for (int i = 0; i < n; i++) {
679	                dst[i] = (float)s[i] * scale;
680	            }
681	            break;
682	        }
683	        case dl::DATA_TYPE_INT16: {
684	            const int16_t *s = static_cast<const int16_t *>(src);
685	            for (int i = 0; i < n; i++) {
686	                dst[i] = (float)s[i] * scale;
687	            }
688	            break;
689	        }
690	        default:
691	            ESP_LOGW(TAG, "Unsupported dequant dtype: %d", (int)dt);
692	            break;
693	        }
694	    }
695	}
696	
697	/* ---- DFL decode ----------------------------------------------------- */
698	/*
699	 * Input layout (after dequantize → float):
700	 *   bbox_feat has the same memory layout as the ESP-DL tensor.
701	 *   ESP-DL uses C-order (row-major): for shape [1, 64, 2100],
702	 *   axis_offset = [134400, 2100, 1], so element (0, channel, anchor)
703	 *   is at offset  channel * 2100 + anchor  (channel-major).
704	 *
705	 * The 64 channels are: 4 coords × 16 bins, packed as:
706	 *   c0_b0..c0_b15, c1_b0..c1_b15, c2_b0..c2_b15, c3_b0..c3_b15
707	 *
708	 * Output: dfl_out [kNumAnchors, 4] — (left, top, right, bottom) offsets.
709	 */
710	void PestAi::dfl_decode(const float *bbox_feat, float *dfl_out) {
711	    for (int a = 0; a < kNumAnchors; a++) {
712	
713	        for (int c = 0; c < 4; c++) {
714	            // 16 bins for coordinate c, anchor a.
715	            // Index: (c*16 + bin) * kNumAnchors + a  (channel-major)
716	#define BBOX_IDX(bin) ((c * 16 + (bin)) * kNumAnchors + a)
717	
718	            // Numerically stable softmax over 16 bins
719	            float max_val = bbox_feat[BBOX_IDX(0)];
720	            for (int i = 1; i < 16; i++) {
721	                float v = bbox_feat[BBOX_IDX(i)];
722	                if (v > max_val) max_val = v;
723	            }
724	            float sum = 0.0f;
725	            float soft[16];
726	            for (int i = 0; i < 16; i++) {
727	                soft[i] = expf(bbox_feat[BBOX_IDX(i)] - max_val);
728	                sum += soft[i];
729	            }
730	            // Weighted sum: offset = Σ(i * softmax(bin_i))
731	            float offset = 0.0f;
732	            for (int i = 0; i < 16; i++) {
733	                offset += (float)i * soft[i] / sum;
734	            }
735	            dfl_out[a * 4 + c] = offset;
736	
737	#undef BBOX_IDX
738	        }
739	    }
740	}
741	
742	/* ---- Anchor decode -------------------------------------------------- */
743	/*
744	 * For anchor index a, determine scale + grid position:
745	 *   a <  1600 → scale 0 (stride 8,  40×40)
746	 *   a <  2000 → scale 1 (stride 16, 20×20)
747	 *   a >= 2000 → scale 2 (stride 32, 10×10)
748	 *
749	 * Anchor centre (feature-map space): cx = col + 0.5, cy = row + 0.5
750	 * Box in feature-map space:
751	 *   x1_fm = cx - left,  y1_fm = cy - top
752	 *   x2_fm = cx + right, y2_fm = cy + bottom
753	 * Box in image space:
754	 *   [x1, y1, x2, y2] = [x1_fm, y1_fm, x2_fm, y2_fm] * stride
755	 */
756	void PestAi::anchor_decode(const float *dfl, float *boxes) {
757	    constexpr int cum0 = 40 * 40;          // 1600
758	    constexpr int cum1 = cum0 + 20 * 20;   // 2000
759	
760	    for (int a = 0; a < kNumAnchors; a++) {
761	        float cx, cy;
762	        int stride;
763	
764	        if (a < cum0) {
765	            // Scale 0: 40×40, stride 8
766	            int row = a / 40;
767	            int col = a % 40;
768	            cx = (float)col + 0.5f;
769	            cy = (float)row + 0.5f;
770	            stride = 8;
771	        } else if (a < cum1) {
772	            // Scale 1: 20×20, stride 16
773	            int idx = a - cum0;
774	            int row = idx / 20;
775	            int col = idx % 20;
776	            cx = (float)col + 0.5f;
777	            cy = (float)row + 0.5f;
778	            stride = 16;
779	        } else {
780	            // Scale 2: 10×10, stride 32
781	            int idx = a - cum1;
782	            int row = idx / 10;
783	            int col = idx % 10;
784	            cx = (float)col + 0.5f;
785	            cy = (float)row + 0.5f;
786	            stride = 32;
787	        }
788	
789	        float left   = dfl[a * 4 + 0];
790	        float top    = dfl[a * 4 + 1];
791	        float right  = dfl[a * 4 + 2];
792	        float bottom = dfl[a * 4 + 3];
793	
794	        float s = (float)stride;
795	        boxes[a * 4 + 0] = (cx - left)   * s;   // x1
796	        boxes[a * 4 + 1] = (cy - top)    * s;   // y1
797	        boxes[a * 4 + 2] = (cx + right)  * s;   // x2
798	        boxes[a * 4 + 3] = (cy + bottom) * s;   // y2
799	    }
800	}
801	
802	/* ---- Score threshold + sigmoid -------------------------------------- */
803	
804	std::vector<PestHit> PestAi::get_candidates(const float *cls_logits, const float *boxes) {
805	    std::vector<PestHit> hits;
806	    hits.reserve(64);  // Typical per-frame pests << 64
807	
808	    // cls_logits layout: [kNumClasses, kNumAnchors] class-major (ESP-DL C-order).
809	    // Element (class, anchor) is at offset: class * kNumAnchors + anchor.
810	    for (int a = 0; a < kNumAnchors; a++) {
811	
812	        // Find max score and class
813	        float max_score = -1.0f;
814	        int   best_cls  = -1;
815	
816	        for (int c = 0; c < kNumClasses; c++) {
817	            // Sigmoid: 1 / (1 + exp(-x))
818	            // Use numerically stable version
819	            float logit = cls_logits[c * kNumAnchors + a];
820	            float prob;
821	            if (logit >= 0) {
822	                prob = 1.0f / (1.0f + expf(-logit));
823	            } else {
824	                float exp_x = expf(logit);
825	                prob = exp_x / (1.0f + exp_x);
826	            }
827	
828	            if (prob > max_score) {
829	                max_score = prob;
830	                best_cls  = c;
831	            }
832	        }
833	
834	        if (max_score >= cfg_.score_thr) {
835	            float x1 = boxes[a * 4 + 0];
836	            float y1 = boxes[a * 4 + 1];
837	            float x2 = boxes[a * 4 + 2];
838	            float y2 = boxes[a * 4 + 3];
839	
840	            // Clip to model bounds
841	            if (x1 < 0) x1 = 0;
842	            if (y1 < 0) y1 = 0;
843	            if (x2 > kModelSize) x2 = kModelSize;
844	            if (y2 > kModelSize) y2 = kModelSize;
845	
846	            float bw = x2 - x1;
847	            float bh = y2 - y1;
848	            if (bw < 2 && bh < 2) continue;  // skip degenerate boxes
849	

            // Reject boxes that cover >75% of the model-input area.
            // YOLO sometimes fires whole-image false positives on blank /
            // featureless backgrounds (e.g. dark wall, desk surface).
            // A real pest never fills the entire frame — it's always a
            // region of interest, typically <30% of the image.
            // Threshold: 75% = 0.75 × 320×320 = 76800 px².
            float box_area = bw * bh;
            if (box_area > 0.75f * kModelSize * kModelSize) {
                static int large_box_reject_count = 0;
                if (++large_box_reject_count <= 3) {
                    ESP_LOGW(TAG, "Rejected oversized box: cls=%d score=%.3f "
                             "box=[%.0f,%.0f %.0fx%.0f] area=%.0f (%.0f%% of frame)",
                             best_cls, max_score, x1, y1, bw, bh, box_area,
                             100.0f * box_area / (kModelSize * kModelSize));
                }
                continue;
            }
850	            PestHit h;
851	            h.x         = (int)x1;
852	            h.y         = (int)y1;
853	            h.w         = (int)bw;
854	            h.h         = (int)bh;
855	            h.class_id  = best_cls;
856	            h.score     = max_score;
857	            h.class_name = kCnNames[best_cls];
858	            hits.push_back(h);
859	        }
860	    }
861	
862	    // Debug: score distribution (every 30 frames)
863	    if ((frames_processed_ % 30) == 1) {
864	        int buckets[6] = {0};  // [0,0.1),[0.1,0.25),[0.25,0.5),[0.5,0.75),[0.75,0.9),[0.9,1.0]
865	        // Scan all anchors to find max score distribution
866	        for (int a = 0; a < kNumAnchors; a++) {
867	            float best = 0.0f;
868	            for (int c = 0; c < kNumClasses; c++) {
869	                float logit = cls_logits[c * kNumAnchors + a];
870	                float prob;
871	                if (logit >= 0) prob = 1.0f / (1.0f + expf(-logit));
872	                else { float ex = expf(logit); prob = ex / (1.0f + ex); }
873	                if (prob > best) best = prob;
874	            }
875	            if (best < 0.1f) buckets[0]++;
876	            else if (best < 0.25f) buckets[1]++;
877	            else if (best < 0.5f) buckets[2]++;
878	            else if (best < 0.75f) buckets[3]++;
879	            else if (best < 0.9f) buckets[4]++;
880	            else buckets[5]++;
881	        }
882	        ESP_LOGI(TAG, "frame %u: score dist [0,.1)=%d [.1,.25)=%d [.25,.5)=%d [.5,.75)=%d [.75,.9)=%d [.9,1]=%d  above thr=%.2f: %zu",
883	                 (unsigned)frames_processed_,
884	                 buckets[0], buckets[1], buckets[2], buckets[3], buckets[4], buckets[5],
885	                 cfg_.score_thr, hits.size());
886	    }
887	
888	    return hits;
889	}
890	
891	/* ---- Per-class NMS -------------------------------------------------- */
892	
893	static float compute_iou(const PestHit &a, const PestHit &b) {
894	    float ax1 = (float)a.x, ay1 = (float)a.y;
895	    float ax2 = ax1 + (float)a.w, ay2 = ay1 + (float)a.h;
896	    float bx1 = (float)b.x, by1 = (float)b.y;
897	    float bx2 = bx1 + (float)b.w, by2 = by1 + (float)b.h;
898	
899	    float ix1 = (ax1 > bx1) ? ax1 : bx1;
900	    float iy1 = (ay1 > by1) ? ay1 : by1;
901	    float ix2 = (ax2 < bx2) ? ax2 : bx2;
902	    float iy2 = (ay2 < by2) ? ay2 : by2;
903	
904	    float iw = ix2 - ix1;
905	    float ih = iy2 - iy1;
906	    if (iw <= 0.0f || ih <= 0.0f) return 0.0f;
907	
908	    float inter = iw * ih;
909	    float area_a = (float)a.w * (float)a.h;
910	    float area_b = (float)b.w * (float)b.h;
911	    return inter / (area_a + area_b - inter + 1e-6f);
912	}
913	
914	void PestAi::apply_nms(std::vector<PestHit> &hits) {
915	    if (hits.size() <= 1) return;
916	
917	    // Sort by score descending
918	    std::sort(hits.begin(), hits.end(),
919	              [](const PestHit &a, const PestHit &b) { return a.score > b.score; });
920	
921	    std::vector<bool> suppressed(hits.size(), false);
922	
923	    for (size_t i = 0; i < hits.size(); i++) {
924	        if (suppressed[i]) continue;
925	        for (size_t j = i + 1; j < hits.size(); j++) {
926	            if (suppressed[j]) continue;
927	            if (hits[i].class_id != hits[j].class_id) continue;
928	            float iou = compute_iou(hits[i], hits[j]);
929	            if (iou > cfg_.nms_thr) {
930	                suppressed[j] = true;
931	            }
932	        }
933	    }
934	
935	    // Remove suppressed hits in-place
936	    size_t w = 0;
937	    for (size_t i = 0; i < hits.size(); i++) {
938	        if (!suppressed[i]) {
939	            if (w != i) hits[w] = hits[i];
940	            w++;
941	        }
942	    }
943	    hits.resize(w);
944	}
945	
946	/* ---- Coordinate scaling --------------------------------------------- */
947	
948	void PestAi::scale_to_frame(std::vector<PestHit> &hits, int fw, int fh) {
949	    float sx = (float)fw / (float)kModelSize;
950	    float sy = (float)fh / (float)kModelSize;
951	
952	    for (auto &h : hits) {
953	        int x1 = (int)((float)h.x * sx);
954	        int y1 = (int)((float)h.y * sy);
955	        int x2 = (int)((float)(h.x + h.w) * sx);
956	        int y2 = (int)((float)(h.y + h.h) * sy);
957	
958	        // Clamp to frame
959	        if (x1 < 0) x1 = 0;
960	        if (y1 < 0) y1 = 0;
961	        if (x2 > fw) x2 = fw;
962	        if (y2 > fh) y2 = fh;
963	
964	        h.x = x1;
965	        h.y = y1;
966	        h.w = x2 - x1;
967	        h.h = y2 - y1;
968	        if (h.w < 1) h.w = 1;
969	        if (h.h < 1) h.h = 1;
970	    }
971	}
972	
973	/* ---- JSON helpers --------------------------------------------------- */
974	
975	std::string PestAi::last_event_json() const {
976	    return last_event_json_;
977	}
978	
979	} // namespace p4fs
980	