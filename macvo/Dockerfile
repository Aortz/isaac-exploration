# MAC-VO runtime image for isaac-exploration (RTX 5080 / Blackwell sm_120).
#
# Dependencies only: MAC-VO source (third_party/macvo, incl. Model/ weights and the
# FlowFormer submodule) and our glue (macvo/) are bind-mounted at run time by
# macvo/run_macvo.sh, so code edits never need an image rebuild.
#
#   docker build -t isaac-exploration/macvo:latest macvo/
#
# Upstream third_party/macvo/Docker/Dockerfile targets CUDA 12.4 + xformers 0.0.27,
# which has no sm_120 kernels; we use the official torch 2.7.1 / CUDA 12.8 image instead.
# Deliberately omitted from upstream requirements.txt (not used by the MAC-VO
# stereo pipeline): xformers, cupy (PWC-Net only), wandb, h5py, onnx, tensorrt.
FROM pytorch/pytorch:2.7.1-cuda12.8-cudnn9-runtime

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility

# gcc/libc6-dev: triton (used by torch.compile in MAC-VO's OnCallCompiler) builds its launcher at runtime.
RUN apt-get update \
 && apt-get install -y --no-install-recommends libgl1 libglib2.0-0 gcc libc6-dev \
 && rm -rf /var/lib/apt/lists/*

# torch / torchvision come from the base image; pin numpy<2 as upstream does
# (hence opencv<4.12 and pyarrow<18, whose newer wheels require numpy 2).
# evo<1.34: newer evo (1.34.3+) raises on align=True + align_origin=True, which MAC-VO's
# Evaluation/MetricsSeq.py passes.
RUN pip install \
      "numpy<2.0.0" \
      "pypose>=0.6.8" \
      "opencv-python-headless<4.12" \
      "evo<1.34" \
      matplotlib==3.8.4 \
      tabulate tqdm rich einops \
      timm==0.9.12 \
      rerun-sdk==0.21.0 "pyarrow<18" \
      yacs pyyaml pillow scipy flow_vis \
      jaxtyping typeguard==2.13.3 \
 && python -c "import torch, pypose, cv2, evo, timm, rerun, flow_vis, torchvision; print('torch', torch.__version__, 'cuda', torch.version.cuda, 'archs', torch.cuda.get_arch_list())"

WORKDIR /macvo
CMD ["/bin/bash"]
