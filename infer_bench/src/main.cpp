#include <Arduino.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <esp_timer.h>

#ifndef BENCH_TARGET_HZ
#define BENCH_TARGET_HZ 100
#endif

#ifndef BENCH_BURST_ITERATIONS
#define BENCH_BURST_ITERATIONS 2000
#endif

#ifndef BENCH_SUSTAINED_CYCLES
#define BENCH_SUSTAINED_CYCLES 300
#endif

#ifndef BENCH_ONLY_CUSTOM_CASE
#define BENCH_ONLY_CUSTOM_CASE 0
#endif

#ifndef BENCH_CUSTOM_FRAME_DIM
#define BENCH_CUSTOM_FRAME_DIM 8
#endif

#ifndef BENCH_CUSTOM_HISTORY_STEPS
#define BENCH_CUSTOM_HISTORY_STEPS 25
#endif

#ifndef BENCH_CUSTOM_H1_DIM
#define BENCH_CUSTOM_H1_DIM 128
#endif

#ifndef BENCH_CUSTOM_H2_DIM
#define BENCH_CUSTOM_H2_DIM 64
#endif

namespace {

constexpr float kActionLow = -0.8f;
constexpr float kActionHigh = 0.8f;
constexpr float kClipMean = 2.0f;
constexpr uint32_t kTargetPeriodUs = 1000000UL / BENCH_TARGET_HZ;

volatile float g_output_sink = 0.0f;

inline float relu(float x) {
    return (x > 0.0f) ? x : 0.0f;
}

inline uint32_t xorshift32(uint32_t& state) {
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

inline float next_uniform_signed(uint32_t& state, float scale) {
    const uint32_t raw = xorshift32(state);
    const float unit = static_cast<float>(raw & 0x00FFFFFFu) / static_cast<float>(0x01000000u);
    return (unit * 2.0f - 1.0f) * scale;
}

template <size_t InputDim, size_t OutputDim>
class DenseLayer {
public:
    DenseLayer() : weights_(nullptr), biases_(nullptr) {}

    ~DenseLayer() {
        if (weights_ != nullptr) {
            free(weights_);
        }
        if (biases_ != nullptr) {
            free(biases_);
        }
    }

    bool init(uint32_t seed, float weight_scale, float bias_scale) {
        if (!allocate()) {
            return false;
        }

        uint32_t rng = seed;
        for (size_t idx = 0; idx < InputDim * OutputDim; ++idx) {
            weights_[idx] = next_uniform_signed(rng, weight_scale);
        }
        for (size_t idx = 0; idx < OutputDim; ++idx) {
            biases_[idx] = next_uniform_signed(rng, bias_scale);
        }
        return true;
    }

    void forward(const float* input, float* output) const {
        for (size_t out_idx = 0; out_idx < OutputDim; ++out_idx) {
            const float* row = weights_ + out_idx * InputDim;
            float sum = biases_[out_idx];
            for (size_t in_idx = 0; in_idx < InputDim; ++in_idx) {
                sum += row[in_idx] * input[in_idx];
            }
            output[out_idx] = sum;
        }
    }

private:
    bool allocate() {
        if (weights_ == nullptr) {
            weights_ = static_cast<float*>(ps_malloc(InputDim * OutputDim * sizeof(float)));
            if (weights_ == nullptr) {
                weights_ = static_cast<float*>(malloc(InputDim * OutputDim * sizeof(float)));
            }
            if (weights_ == nullptr) {
                return false;
            }
        }

        if (biases_ == nullptr) {
            biases_ = static_cast<float*>(ps_malloc(OutputDim * sizeof(float)));
            if (biases_ == nullptr) {
                biases_ = static_cast<float*>(malloc(OutputDim * sizeof(float)));
            }
            if (biases_ == nullptr) {
                return false;
            }
        }
        return true;
    }

    float* weights_;
    float* biases_;
};

template <size_t InputDim, size_t H1Dim, size_t H2Dim>
class BenchmarkPolicy {
public:
    static constexpr size_t kParameterCount =
        (InputDim * H1Dim) + H1Dim +
        (H1Dim * H2Dim) + H2Dim +
        (H2Dim * 1) + 1;

    static constexpr size_t kParameterBytes = kParameterCount * sizeof(float);

    bool init(uint32_t seed) {
        return fc1_.init(seed ^ 0x13579BDFu, 0.08f, 0.02f) &&
               fc2_.init(seed ^ 0x2468ACE0u, 0.08f, 0.02f) &&
               fc3_.init(seed ^ 0xA5A5A5A5u, 0.08f, 0.02f);
    }

    float forward(std::array<float, InputDim>& input, size_t iteration) const {
        std::array<float, H1Dim> h1{};
        std::array<float, H2Dim> h2{};
        float out[1] = {0.0f};

        const size_t mutate_idx = iteration % InputDim;
        input[mutate_idx] += 0.001f;
        if (input[mutate_idx] > 1.0f) {
            input[mutate_idx] = -1.0f;
        }

        fc1_.forward(input.data(), h1.data());
        for (size_t idx = 0; idx < H1Dim; ++idx) {
            h1[idx] = relu(h1[idx]);
        }

        fc2_.forward(h1.data(), h2.data());
        for (size_t idx = 0; idx < H2Dim; ++idx) {
            h2[idx] = relu(h2[idx]);
        }

        fc3_.forward(h2.data(), out);

        float mean = out[0];
        if (mean < -kClipMean) {
            mean = -kClipMean;
        } else if (mean > kClipMean) {
            mean = kClipMean;
        }

        const float squashed = tanhf(mean);
        return kActionLow + (squashed + 1.0f) * 0.5f * (kActionHigh - kActionLow);
    }

private:
    DenseLayer<InputDim, H1Dim> fc1_;
    DenseLayer<H1Dim, H2Dim> fc2_;
    DenseLayer<H2Dim, 1> fc3_;
};

struct BurstStats {
    uint32_t iterations = 0;
    uint32_t min_us = UINT32_MAX;
    uint32_t max_us = 0;
    uint64_t total_us = 0;
};

struct SustainStats {
    uint32_t cycles = 0;
    uint32_t misses = 0;
    uint32_t worst_us = 0;
    int64_t total_slack_us = 0;
};

template <size_t InputDim>
void seed_input(std::array<float, InputDim>& input, uint32_t seed) {
    uint32_t rng = seed;
    for (size_t idx = 0; idx < InputDim; ++idx) {
        input[idx] = next_uniform_signed(rng, 0.5f);
    }
}

template <size_t InputDim, size_t H1Dim, size_t H2Dim>
BurstStats run_burst(BenchmarkPolicy<InputDim, H1Dim, H2Dim>& policy) {
    std::array<float, InputDim> input{};
    seed_input(input, 0xC0FFEE11u ^ static_cast<uint32_t>(InputDim));

    for (size_t warmup = 0; warmup < 64; ++warmup) {
        g_output_sink += policy.forward(input, warmup);
    }

    BurstStats stats;
    stats.iterations = BENCH_BURST_ITERATIONS;
    for (uint32_t iter = 0; iter < BENCH_BURST_ITERATIONS; ++iter) {
        const int64_t start_us = esp_timer_get_time();
        g_output_sink += policy.forward(input, iter);
        const int64_t elapsed_us = esp_timer_get_time() - start_us;
        const uint32_t sample_us = static_cast<uint32_t>(elapsed_us);

        if (sample_us < stats.min_us) {
            stats.min_us = sample_us;
        }
        if (sample_us > stats.max_us) {
            stats.max_us = sample_us;
        }
        stats.total_us += sample_us;
    }
    return stats;
}

inline void wait_until(int64_t target_us) {
    while (true) {
        const int64_t now_us = esp_timer_get_time();
        const int64_t remaining_us = target_us - now_us;
        if (remaining_us <= 0) {
            return;
        }
        if (remaining_us > 2000) {
            delay(1);
        } else if (remaining_us > 200) {
            delayMicroseconds(100);
        }
    }
}

template <size_t InputDim, size_t H1Dim, size_t H2Dim>
SustainStats run_sustained(BenchmarkPolicy<InputDim, H1Dim, H2Dim>& policy) {
    std::array<float, InputDim> input{};
    seed_input(input, 0xBADC0DEu ^ static_cast<uint32_t>(H1Dim));

    SustainStats stats;
    stats.cycles = BENCH_SUSTAINED_CYCLES;

    int64_t next_release_us = esp_timer_get_time() + 50000;
    for (uint32_t cycle = 0; cycle < BENCH_SUSTAINED_CYCLES; ++cycle) {
        wait_until(next_release_us);

        const int64_t start_us = esp_timer_get_time();
        g_output_sink += policy.forward(input, cycle);
        const uint32_t elapsed_us = static_cast<uint32_t>(esp_timer_get_time() - start_us);

        if (elapsed_us > stats.worst_us) {
            stats.worst_us = elapsed_us;
        }
        if (elapsed_us > kTargetPeriodUs) {
            ++stats.misses;
            stats.total_slack_us -= static_cast<int64_t>(elapsed_us - kTargetPeriodUs);
        } else {
            stats.total_slack_us += static_cast<int64_t>(kTargetPeriodUs - elapsed_us);
        }

        next_release_us += kTargetPeriodUs;
    }
    return stats;
}

template <size_t FrameDim, size_t HistorySteps, size_t H1Dim, size_t H2Dim>
void run_case(const char* label) {
    constexpr size_t kInputDim = FrameDim * HistorySteps;
    BenchmarkPolicy<kInputDim, H1Dim, H2Dim> policy;

    Serial.println();
    Serial.printf("[bench] %s\n", label);
    Serial.printf("  dims: frame_dim=%u history=%u input=%u hidden=(%u,%u)\n",
                  static_cast<unsigned>(FrameDim),
                  static_cast<unsigned>(HistorySteps),
                  static_cast<unsigned>(kInputDim),
                  static_cast<unsigned>(H1Dim),
                  static_cast<unsigned>(H2Dim));
    Serial.printf("  params: %u floats, %u bytes (%.1f KB)\n",
                  static_cast<unsigned>(BenchmarkPolicy<kInputDim, H1Dim, H2Dim>::kParameterCount),
                  static_cast<unsigned>(BenchmarkPolicy<kInputDim, H1Dim, H2Dim>::kParameterBytes),
                  BenchmarkPolicy<kInputDim, H1Dim, H2Dim>::kParameterBytes / 1024.0f);

    if (!policy.init(0x12345678u ^ static_cast<uint32_t>(kInputDim) ^ (static_cast<uint32_t>(H1Dim) << 8))) {
        Serial.println("  ERROR: parameter allocation failed");
        return;
    }

    const BurstStats burst = run_burst(policy);
    const SustainStats sustained = run_sustained(policy);

    const float avg_us = static_cast<float>(burst.total_us) / static_cast<float>(burst.iterations);
    const float budget_pct = (avg_us * 100.0f) / static_cast<float>(kTargetPeriodUs);
    const float est_max_hz = 1000000.0f / avg_us;
    const float avg_slack_us = static_cast<float>(sustained.total_slack_us) / static_cast<float>(sustained.cycles);

    Serial.printf("  burst: avg=%.1f us min=%u us max=%u us est_max_hz=%.1f budget@%uHz=%.2f%%\n",
                  avg_us,
                  burst.min_us,
                  burst.max_us,
                  est_max_hz,
                  static_cast<unsigned>(BENCH_TARGET_HZ),
                  budget_pct);
    Serial.printf("  sustain: cycles=%u misses=%u worst=%u us avg_slack=%.1f us\n",
                  sustained.cycles,
                  sustained.misses,
                  sustained.worst_us,
                  avg_slack_us);
}

void print_banner() {
    Serial.println();
    Serial.println("=== ESP32 Inference Benchmark ===");
    Serial.printf("chip=%s cpu=%u MHz target=%u Hz period=%u us\n",
                  ESP.getChipModel(),
                  static_cast<unsigned>(ESP.getCpuFreqMHz()),
                  static_cast<unsigned>(BENCH_TARGET_HZ),
                  static_cast<unsigned>(kTargetPeriodUs));
    Serial.printf("heap_free=%u heap_min=%u psram_found=%s psram_free=%u\n",
                  static_cast<unsigned>(ESP.getFreeHeap()),
                  static_cast<unsigned>(ESP.getMinFreeHeap()),
                  psramFound() ? "yes" : "no",
                  static_cast<unsigned>(ESP.getFreePsram()));
    Serial.printf("burst_iterations=%u sustained_cycles=%u sink=%.4f\n",
                  static_cast<unsigned>(BENCH_BURST_ITERATIONS),
                  static_cast<unsigned>(BENCH_SUSTAINED_CYCLES),
                  g_output_sink);
}

void run_suite() {
#if BENCH_ONLY_CUSTOM_CASE
    run_case<BENCH_CUSTOM_FRAME_DIM, BENCH_CUSTOM_HISTORY_STEPS, BENCH_CUSTOM_H1_DIM, BENCH_CUSTOM_H2_DIM>(
        "custom");
#else
    run_case<8, 5, 128, 64>("current_policy_shape");
    // run_case<8, 10, 128, 64>("history_10_same_hidden");
    // run_case<8, 20, 128, 64>("history_20_same_hidden");
    // run_case<12, 20, 128, 64>("frame12_history20_same_hidden");
    // run_case<16, 20, 128, 64>("frame16_history20_same_hidden");
    run_case<8, 40, 128, 64>("history_40_same_hidden");
    run_case<8, 50, 128, 64>("history_50_same_hidden");
    run_case<8, 20, 256, 128>("history_20_wider_hidden");
    run_case<8, 40, 256, 128>("history_40_wider_hidden");
#endif
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);
    print_banner();
    run_suite();
    Serial.println();
    Serial.printf("benchmark complete; final_sink=%.6f\n", g_output_sink);
}

void loop() {
    delay(1000);
}
