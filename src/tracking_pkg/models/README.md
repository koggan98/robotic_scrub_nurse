# `robot.onnx` wake-word model

## Current asset status

`robot.onnx` is a required external training artifact and is intentionally not
present in this directory yet. Do not create an empty, copied, or synthetic
placeholder with that name: the ASR node must fail closed while the real model
is missing.

The repository contains:

- `robot.training.yaml`: the production training configuration.
- `robot.model.yaml`: the asset contract and provenance record. Its current
  `state: required_external_asset` is deliberate.

At the time this integration was prepared (2026-07-23), the development
workstation could not produce a trustworthy model:

- PyTorch reported no CUDA device.
- The openWakeWord training dependencies and datasets were not installed.
- The official negative feature file is 17.3 GB and the validation feature file
  is 185 MB.
- openWakeWord 0.6.0's `full` training extra pins TensorFlow CPU 2.8.1.
  TensorFlow publishes a CPython 3.10 wheel for x86_64, but not for the Jetson's
  AArch64 platform. The official training environment therefore cannot be
  installed unchanged on the Jetson.

The Jetson Orin was checked on 2026-07-24: its CUDA-enabled PyTorch 2.8.0
installation works and is suitable for running the ONNX detector. Production
training is still best performed in an isolated x86_64 Linux environment with
Python 3.10, an NVIDIA GPU, and enough fast disk space for the roughly 18 GB of
feature data plus generated clips. A current standard hosted Colab runtime uses
a newer Python version and does not reproduce the pinned 0.6.0 notebook
unchanged. Colab is usable when connected to a compatible local runtime.

An ONNX-only modification of the upstream trainer can perform a reduced
pipeline smoke test on the Jetson because the TensorFlow dependency is only
needed for the final TFLite conversion. Such a smoke test does not replace the
100,000-example production training and acoustic validation described below.

## Pinned training sources

The configuration follows the official openWakeWord 0.6.0 automatic training
pipeline:

- openWakeWord repository:
  `https://github.com/dscripka/openWakeWord.git`
- openWakeWord tag/commit:
  `v0.6.0` / `c8ef6912c5feccf1037b852d9bc6c7ed644135ba`
- Official notebook:
  `notebooks/automatic_model_training.ipynb`
- Piper sample generator repository:
  `https://github.com/rhasspy/piper-sample-generator.git`
- Piper tag/commit:
  `v2.0.0` / `195e3bd967d54589c2137c9de2b22ad526ba6b6f`

The official documentation recommends at least 20,000 positive examples and
notes that 100,000 or more often performs better. The production configuration
therefore uses 100,000 training examples for the short, single-word trigger
`robot`. A quick notebook run with reduced sample counts is useful only as a
pipeline smoke test and must not be deployed.

## Training inputs

Prepare a separate training directory according to the pinned official
notebook. Relative paths in `robot.training.yaml` are resolved from that
directory. At minimum it must contain:

- `piper-sample-generator/`
- `piper-sample-generator/models/en_US-libritts_r-medium.pt`
- `mit_rirs/`
- `background_clips/` with representative speech, music, room noise, and robot
  operating noise
- `openwakeword_features_ACAV100M_2000_hrs_16bit.npy`
- `validation_set_features.npy`

Verify the immutable inputs before training:

| Input | Size | SHA-256 |
| --- | ---: | --- |
| `en_US-libritts_r-medium.pt` | 204,089,915 bytes | `e95ee53770bf598c354a6e6dbfc95ccb259aeeb501d35a86be8a767429ab0ff6` |
| `embedding_model.onnx` | 1,326,578 bytes | `70d164290c1d095d1d4ee149bc5e00543250a7316b59f31d056cff7bd3075c1f` |
| `melspectrogram.onnx` | 1,087,958 bytes | `ba2b0e0f8b7b875369a2c89cb13360ff53bac436f2895cced9f479fa65eb176f` |
| `openwakeword_features_ACAV100M_2000_hrs_16bit.npy` | 17.3 GB | `721a66d0682c65a1b5c1da0aa109409cede1d20e28b15235c344b000cbb7654f` |
| `validation_set_features.npy` | 185 MB | `a56a8a0f8e0efb91900acc6de4c0cdf4c564842e8475a7d49b36c039e17a690f` |

The two openWakeWord feature-extractor models are downloaded by the official
0.6.0 notebook from the openWakeWord 0.5.1 release. The large feature datasets
come from:
`https://huggingface.co/datasets/davidscripka/openwakeword_features`.

## Reproducible training workflow

For a university GPU Hub, use the upload-ready PyTorch/ONNX training kit and
its German step-by-step guide:
[`ros_unrelated_scripts/wakeword_training_kit/START_HERE_DE.md`](../../../ros_unrelated_scripts/wakeword_training_kit/START_HERE_DE.md).
It pins both upstream repositories, downloads and verifies all large assets,
handles interrupted stages, validates the ONNX candidate, and packages only
the small result archive for return to this workspace. The kit targets
PyTorch 2.10 with Python 3.10 and a CUDA 12.6/12.8 build. If the university
VPN is available only on macOS, follow the separate
[`MAC_TRANSFER_DE.md`](../../../ros_unrelated_scripts/wakeword_training_kit/MAC_TRANSFER_DE.md)
bridge guide.

1. Prepare an isolated x86_64 Linux environment with Python 3.10,
   `torch==2.10.0`, a CUDA 12.6/12.8 PyTorch build, and an NVIDIA GPU. Do not
   install the training stack into the robot's ROS Python environment.
2. Check out the pinned commits listed above. Do not train from moving `main`
   branches.
3. Prepare and checksum the inputs listed above.
4. Copy `robot.training.yaml` into the training directory.
5. Run the three official stages:

   ```bash
   python openWakeWord/openwakeword/train.py \
     --training_config robot.training.yaml --generate_clips
   python openWakeWord/openwakeword/train.py \
     --training_config robot.training.yaml --augment_clips
   python openWakeWord/openwakeword/train.py \
     --training_config robot.training.yaml --train_model
   ```

6. The expected output is `robot_training/robot.onnx`. Confirm it is a
   valid ONNX graph before any microphone test:

   ```bash
   python -c "import onnxruntime as ort; ort.InferenceSession(
       'robot_training/robot.onnx',
       providers=['CPUExecutionProvider'])"
   sha256sum robot_training/robot.onnx
   ```

7. Test the candidate with openWakeWord 0.6.0 at 16 kHz using the actual Samson
   and Jieli microphone paths. Start with threshold `0.5` and test thresholds
   through `0.9` in `0.05` increments.
8. Accept a candidate only when each microphone achieves at least 19 detections
   in 20 deliberate `Robot` trials and zero activations in at least 60 minutes
   of representative background conversation. Also test the configured
   adversarial phrases explicitly.
9. Copy the accepted file to this directory as `robot.onnx`, then update
   `robot.model.yaml` with its byte size, SHA-256, UTC creation time, selected
   threshold, and measured validation results. Review that metadata in the same
   commit as the binary.
10. Rebuild and source the ROS workspace so CMake copies the accepted artifact
    to the path used by `jetson_launch.py`:

    ```bash
    colcon build --packages-select tracking_pkg
    source install/setup.bash
    ```

## Licensing and provenance

The openWakeWord source code is Apache-2.0, but its official pretrained models
and the recommended precomputed feature dataset are CC BY-NC-SA 4.0. Preserve
all source notices and treat the generated model as CC BY-NC-SA 4.0 unless a
documented review of every training input establishes another license. Do not
label the resulting model Apache-2.0 merely because the training code uses that
license.
