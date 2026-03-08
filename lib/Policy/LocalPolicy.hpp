#ifndef LOCAL_POLICY_HPP
#define LOCAL_POLICY_HPP

#include <Arduino.h>
#include <array>
#include <FS.h>
#include <LittleFS.h>
#include "deploy_config.h"
#include "local_obs_config.h"

// ===== Local policy network dimensions =====
// Input: latent_cmd(LOCAL_LATENT_DIM) + local_obs(LOCAL_OBS_DIM)
// Architecture: Linear(input->128) -> ReLU -> Linear(128->64) -> ReLU -> Linear(64->1)
// Output: 1 (mean only; log_std head not deployed)
constexpr size_t LOCAL_INPUT_DIM   = LOCAL_LATENT_DIM + LOCAL_OBS_DIM;
constexpr size_t LOCAL_H1_DIM      = 128;
constexpr size_t LOCAL_H2_DIM      = 64;
constexpr size_t LOCAL_ACT_DIM     = 1;

// Action bounds: [-0.8, 0.8]
constexpr float LOCAL_ACTION_LOW  = -0.8f;
constexpr float LOCAL_ACTION_HIGH =  0.8f;

// Mean clamp range (from training: clip_mean=2.0)
constexpr float LOCAL_CLIP_MEAN = 2.0f;

// ===== Fully-connected (dense) layer =====
// weights layout: row-major per output neuron i:
//   weights[i * InputSize + j] == w_{i,j}
template <size_t InputSize, size_t OutputSize>
class LocalFC {
public:
    LocalFC() : weights(nullptr), biases(nullptr) {}

    ~LocalFC() {
        if (weights) free(weights);
        if (biases) free(biases);
    }

    bool load(const char* weights_path, const char* biases_path) {
        // Allocate weights
        if (!weights) {
            weights = (float*)ps_malloc(InputSize * OutputSize * sizeof(float));
            if (!weights) {
                Serial.println("    WARNING: PSRAM alloc failed, trying heap...");
                weights = (float*)malloc(InputSize * OutputSize * sizeof(float));
                if (!weights) {
                    Serial.printf("    ERROR: Failed to allocate %u bytes for weights\n",
                                  (unsigned)(InputSize * OutputSize * sizeof(float)));
                    return false;
                }
            }
        }

        // Allocate biases
        if (!biases) {
            biases = (float*)ps_malloc(OutputSize * sizeof(float));
            if (!biases) {
                biases = (float*)malloc(OutputSize * sizeof(float));
                if (!biases) {
                    Serial.printf("    ERROR: Failed to allocate %u bytes for biases\n",
                                  (unsigned)(OutputSize * sizeof(float)));
                    return false;
                }
            }
        }

        // Load weights
        {
            File f = LittleFS.open(weights_path, "r");
            if (!f) {
                Serial.printf("    ERROR: Cannot open %s\n", weights_path);
                return false;
            }
            const size_t nbytes = InputSize * OutputSize * sizeof(float);
            if (f.size() != nbytes) {
                Serial.printf("    ERROR: %s size %u != expected %u\n",
                              weights_path, (unsigned)f.size(), (unsigned)nbytes);
                f.close();
                return false;
            }
            int rd = f.readBytes(reinterpret_cast<char*>(weights), nbytes);
            f.close();
            if (rd != (int)nbytes) return false;
        }

        // Load biases
        {
            File f = LittleFS.open(biases_path, "r");
            if (!f) {
                Serial.printf("    ERROR: Cannot open %s\n", biases_path);
                return false;
            }
            const size_t nbytes = OutputSize * sizeof(float);
            if (f.size() != nbytes) {
                Serial.printf("    ERROR: %s size %u != expected %u\n",
                              biases_path, (unsigned)f.size(), (unsigned)nbytes);
                f.close();
                return false;
            }
            int rd = f.readBytes(reinterpret_cast<char*>(biases), nbytes);
            f.close();
            if (rd != (int)nbytes) return false;
        }

        return true;
    }

    // y[i] = dot(x, W[i,:]) + b[i]
    template <size_t N>
    void forward(const std::array<float, InputSize>& x,
                 std::array<float, N>& y) const {
        static_assert(N == OutputSize, "Output size mismatch");
        for (size_t i = 0; i < OutputSize; ++i) {
            float sum = biases[i];
            const float* row = weights + i * InputSize;
            for (size_t j = 0; j < InputSize; ++j) {
                sum += row[j] * x[j];
            }
            y[i] = sum;
        }
    }

private:
    float* weights;
    float* biases;
};

// ===== Local Policy =====
//
// Full action pipeline (matches training exactly):
//   1. NN forward: input=[latent_cmd(8), local_obs(40)] -> raw mean
//   2. Clamp mean to [-2, 2]
//   3. Tanh squash -> [-1, 1]
//   4. Scale to [-0.8, 0.8] (NN action output)
//   5. Add default_dof_pos[module_idx] (per-module joint offset)
//   -> motor_target (position setpoint for PD controller)
//
// The motor_task then adds its own mechanical offset before sending to motor.
//
class LocalPolicy {
public:
    LocalPolicy() : loaded(false), module_idx(0) {}

    // Set which module this ESP32 controls (0, 1, or 2)
    // This determines which default_dof_pos offset is applied.
    void set_module_index(int idx) {
        if (idx != module_idx) {
            module_idx = idx;
            Serial.printf("[LocalPolicy] Module index set to %d, default_dof_pos = %.4f rad\n",
                          idx, DEPLOY_DEFAULT_DOF_POS[idx]);
        }
    }

    int get_module_index() const { return module_idx; }

    bool load_from_littlefs() {
        Serial.println("[LocalPolicy] Mounting LittleFS...");
        if (!LittleFS.begin(true)) {
            Serial.println("[LocalPolicy] ERROR: LittleFS.begin() failed!");
            return false;
        }

        // List files
        Serial.println("[LocalPolicy] Files in LittleFS:");
        File root = LittleFS.open("/");
        File file = root.openNextFile();
        while (file) {
            Serial.printf("  %s (%u bytes)\n", file.name(), (unsigned)file.size());
            file = root.openNextFile();
        }

        Serial.println("[LocalPolicy] Loading layer 1 (48 -> 128)...");
        if (!fc1.load("/w1.bin", "/b1.bin")) {
            Serial.println("[LocalPolicy] ERROR: fc1 load failed");
            LittleFS.end();
            return false;
        }

        Serial.println("[LocalPolicy] Loading layer 2 (128 -> 64)...");
        if (!fc2.load("/w2.bin", "/b2.bin")) {
            Serial.println("[LocalPolicy] ERROR: fc2 load failed");
            LittleFS.end();
            return false;
        }

        Serial.println("[LocalPolicy] Loading layer 3 (64 -> 1)...");
        if (!fc3.load("/w3.bin", "/b3.bin")) {
            Serial.println("[LocalPolicy] ERROR: fc3 load failed");
            LittleFS.end();
            return false;
        }

        LittleFS.end();
        loaded = true;
        Serial.println("[LocalPolicy] All weights loaded successfully!");
        return true;
    }

    bool is_loaded() const { return loaded; }

    // Run the NN forward pass only (no default_dof_pos offset).
    // Returns the raw NN action in [-0.8, 0.8].
    float forward_nn(const std::array<float, LOCAL_LATENT_DIM>& latent_cmd,
                     const std::array<float, LOCAL_OBS_DIM>& local_obs) const {
        // Concatenate input: [latent_cmd, local_obs]
        std::array<float, LOCAL_INPUT_DIM> input{};
        for (size_t i = 0; i < LOCAL_LATENT_DIM; ++i)
            input[i] = latent_cmd[i];
        for (size_t i = 0; i < LOCAL_OBS_DIM; ++i)
            input[LOCAL_LATENT_DIM + i] = local_obs[i];

        // Layer 1: Linear(48 -> 128) + ReLU
        std::array<float, LOCAL_H1_DIM> h1{};
        fc1.forward(input, h1);
        for (auto& v : h1) v = (v > 0.0f) ? v : 0.0f;

        // Layer 2: Linear(128 -> 64) + ReLU
        std::array<float, LOCAL_H2_DIM> h2{};
        fc2.forward(h1, h2);
        for (auto& v : h2) v = (v > 0.0f) ? v : 0.0f;

        // Layer 3: Linear(64 -> 1) = mean
        std::array<float, LOCAL_ACT_DIM> out{};
        fc3.forward(h2, out);
        float mean = out[0];

        // Clamp mean (matches training clip_mean=2.0)
        if (mean < -LOCAL_CLIP_MEAN) mean = -LOCAL_CLIP_MEAN;
        if (mean >  LOCAL_CLIP_MEAN) mean =  LOCAL_CLIP_MEAN;

        // Tanh squashing + scale to [-0.8, 0.8]
        float a = tanhf(mean);  // in [-1, 1]
        float action = LOCAL_ACTION_LOW + (a + 1.0f) * 0.5f * (LOCAL_ACTION_HIGH - LOCAL_ACTION_LOW);

        return action;
    }

    // Full action pipeline: NN output + default_dof_pos offset.
    // Returns the motor position target (to be sent to motor_task).
    //
    // This matches the training pipeline:
    //   motor_target = policy_action + default_dof_pos[module_idx]
    //
    // The motor_task will then add its own mechanical offset before
    // sending to the motor driver.
    float select_action(const std::array<float, LOCAL_LATENT_DIM>& latent_cmd,
                        const std::array<float, LOCAL_OBS_DIM>& local_obs) const {
        float nn_action = forward_nn(latent_cmd, local_obs);
        float motor_target = nn_action + DEPLOY_DEFAULT_DOF_POS[module_idx];
        return motor_target;
    }

    // Run sanity check with known test vector
    void run_sanity_check(float expected_mean, float expected_nn_action) const {
        Serial.println("\n[LocalPolicy] === Sanity Check ===");

        // Test vector: latent[0]=1.0, local_obs[0]=0.5, rest zeros
        std::array<float, LOCAL_LATENT_DIM> latent{};
        std::array<float, LOCAL_OBS_DIM> local_obs{};
        latent[0] = 1.0f;
        local_obs[0] = 0.5f;

        // Compute raw mean (before clamp/tanh) for comparison
        std::array<float, LOCAL_INPUT_DIM> input{};
        for (size_t i = 0; i < LOCAL_LATENT_DIM; ++i) input[i] = latent[i];
        for (size_t i = 0; i < LOCAL_OBS_DIM; ++i) input[LOCAL_LATENT_DIM + i] = local_obs[i];

        std::array<float, LOCAL_H1_DIM> h1{};
        fc1.forward(input, h1);
        for (auto& v : h1) v = (v > 0.0f) ? v : 0.0f;

        std::array<float, LOCAL_H2_DIM> h2{};
        fc2.forward(h1, h2);
        for (auto& v : h2) v = (v > 0.0f) ? v : 0.0f;

        std::array<float, LOCAL_ACT_DIM> out{};
        fc3.forward(h2, out);
        float raw_mean = out[0];

        float nn_action = forward_nn(latent, local_obs);
        float motor_target = select_action(latent, local_obs);

        Serial.printf("  Raw mean:       %.8f (expected: %.8f, err: %.2e)\n",
                       raw_mean, expected_mean, fabsf(raw_mean - expected_mean));
        Serial.printf("  NN action:      %.8f (expected: %.8f, err: %.2e)\n",
                       nn_action, expected_nn_action, fabsf(nn_action - expected_nn_action));
        Serial.printf("  + default_dof:  %.4f (module %d)\n",
                       DEPLOY_DEFAULT_DOF_POS[module_idx], module_idx);
        Serial.printf("  Motor target:   %.8f\n", motor_target);

        bool ok = (fabsf(raw_mean - expected_mean) < 1e-3f) &&
                  (fabsf(nn_action - expected_nn_action) < 1e-3f);
        Serial.printf("  Result: %s\n", ok ? "PASS" : "FAIL");

        // Show all module targets for reference
        Serial.println("\n  Per-module motor targets for this test input:");
        for (int m = 0; m < DEPLOY_NUM_MODULES; ++m) {
            Serial.printf("    Module %d: nn_action(%.4f) + default_dof(%.4f) = motor_target(%.4f)\n",
                          m, nn_action, DEPLOY_DEFAULT_DOF_POS[m],
                          nn_action + DEPLOY_DEFAULT_DOF_POS[m]);
        }
        Serial.println("[LocalPolicy] === End Sanity Check ===\n");
    }

private:
    LocalFC<LOCAL_INPUT_DIM, LOCAL_H1_DIM> fc1;
    LocalFC<LOCAL_H1_DIM, LOCAL_H2_DIM>    fc2;
    LocalFC<LOCAL_H2_DIM, LOCAL_ACT_DIM>   fc3;
    bool loaded;
    int module_idx;
};

#endif // LOCAL_POLICY_HPP
