# ESP32 Inference Benchmark

Standalone ESP32 firmware for measuring how much of a `100 Hz` control-loop budget is consumed by a policy-shaped MLP.

The benchmark mirrors the current onboard local policy structure:

- Input: `frame_dim * history_steps`
- Network: `Linear(input -> H1) -> ReLU -> Linear(H1 -> H2) -> ReLU -> Linear(H2 -> 1)`
- Output shaping: clamp to `[-2, 2]`, then `tanh`, then scale to `[-0.8, 0.8]`

## Build

```bash
~/.platformio/penv/bin/pio run -d capy_esp32/infer_bench -e jw_esp32_infer_bench
```

## Preset sweep

The default `jw_esp32_infer_bench` environment runs several preset cases that vary:

- observation history (`8 x 5`, `8 x 10`, `8 x 20`, `8 x 40`)
- per-frame observation size (`12 x 20`, `16 x 20`)
- hidden width (`128/64` and `256/128`)

Each case reports:

- parameter count and memory footprint
- burst-mode inference timing
- estimated max frequency from mean inference time
- whether any inferences miss the `10 ms` budget in a sustained `100 Hz` loop

## Custom case

The `jw_esp32_infer_custom` environment is meant for quick sizing checks. Override dimensions at build time:

```bash
~/.platformio/penv/bin/pio run -d capy_esp32/infer_bench -e jw_esp32_infer_custom \
  --project-option "build_flags=-DBOARD_HAS_PSRAM -DBENCH_TARGET_HZ=100 -DBENCH_BURST_ITERATIONS=2000 -DBENCH_SUSTAINED_CYCLES=300 -DBENCH_ONLY_CUSTOM_CASE=1 -DBENCH_CUSTOM_FRAME_DIM=8 -DBENCH_CUSTOM_HISTORY_STEPS=50 -DBENCH_CUSTOM_H1_DIM=128 -DBENCH_CUSTOM_H2_DIM=64"
```

For a policy with the same structure as the current deployed one, `frame_dim = 8` and `history_steps` directly controls `LOCAL_OBS_DIM = 8 * history_steps`.
