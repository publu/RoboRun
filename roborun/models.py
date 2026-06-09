"""Model wrappers for RoboRun — YOLO, CLIP, JEPA, Cosmos."""

from __future__ import annotations

import time
from dataclasses import dataclass
from threading import RLock
from typing import Any

import numpy as np


@dataclass
class Detection:
    bbox: tuple[float, float, float, float]
    label: str
    confidence: float
    track_id: int = -1
    clip_score: float | None = None

    def area(self) -> float:
        x1, y1, x2, y2 = self.bbox
        return max(0.0, x2 - x1) * max(0.0, y2 - y1)

    def center(self) -> tuple[float, float]:
        x1, y1, x2, y2 = self.bbox
        return ((x1 + x2) / 2, (y1 + y2) / 2)

    def to_dict(self) -> dict:
        d = {
            "bbox": list(self.bbox),
            "label": self.label,
            "confidence": round(self.confidence, 3),
            "track_id": self.track_id,
        }
        if self.clip_score is not None:
            d["clip_score"] = round(self.clip_score, 3)
        return d


class YOLODetector:
    """YOLO object detector with tracking."""

    def __init__(self, model_name: str = "yolo11n.pt") -> None:
        self._model_name = model_name
        self._model = None
        self._lock = RLock()

    def _ensure_loaded(self) -> None:
        if self._model is None:
            from ultralytics import YOLO
            self._model = YOLO(self._model_name)

    def detect(self, frame: np.ndarray, track: bool = True) -> list[Detection]:
        with self._lock:
            self._ensure_loaded()
            if track:
                results = self._model.track(frame, persist=True, verbose=False)
            else:
                results = self._model(frame, verbose=False)

        detections = []
        for r in results:
            boxes = r.boxes
            if boxes is None:
                continue
            for i in range(len(boxes)):
                x1, y1, x2, y2 = boxes.xyxy[i].tolist()
                conf = float(boxes.conf[i])
                cls_id = int(boxes.cls[i])
                label = r.names.get(cls_id, str(cls_id))
                tid = int(boxes.id[i]) if boxes.id is not None else -1
                detections.append(Detection(
                    bbox=(x1, y1, x2, y2),
                    label=label,
                    confidence=conf,
                    track_id=tid,
                ))
        return detections


class CLIPMatcher:
    """OpenCLIP text-image matcher for zero-shot classification and search."""

    def __init__(self, model_name: str = "ViT-B-32", pretrained: str = "laion2b_s34b_b79k") -> None:
        self._model_name = model_name
        self._pretrained = pretrained
        self._model = None
        self._preprocess = None
        self._tokenizer = None
        self._lock = RLock()

    def _ensure_loaded(self) -> None:
        if self._model is not None:
            return
        import open_clip
        import torch
        self._device = "mps" if torch.backends.mps.is_available() else "cuda" if torch.cuda.is_available() else "cpu"
        self._model, _, self._preprocess = open_clip.create_model_and_transforms(
            self._model_name, pretrained=self._pretrained, device=self._device,
        )
        self._tokenizer = open_clip.get_tokenizer(self._model_name)
        self._model.eval()

    def embed_text(self, text: str) -> np.ndarray:
        with self._lock:
            self._ensure_loaded()
            import torch
            tokens = self._tokenizer([text]).to(self._device)
            with torch.no_grad():
                emb = self._model.encode_text(tokens)
                emb = emb / emb.norm(dim=-1, keepdim=True)
            return emb.cpu().numpy()[0]

    def embed_image(self, image: np.ndarray) -> np.ndarray:
        with self._lock:
            self._ensure_loaded()
            import torch
            from PIL import Image
            pil = Image.fromarray(image[..., ::-1]) if image.shape[-1] == 3 else Image.fromarray(image)
            tensor = self._preprocess(pil).unsqueeze(0).to(self._device)
            with torch.no_grad():
                emb = self._model.encode_image(tensor)
                emb = emb / emb.norm(dim=-1, keepdim=True)
            return emb.cpu().numpy()[0]

    def score(self, text: str, image: np.ndarray) -> float:
        t = self.embed_text(text)
        i = self.embed_image(image)
        return float(t @ i)

    def match_detections(
        self, text: str, frame: np.ndarray, detections: list[Detection], threshold: float = 0.2,
    ) -> list[Detection]:
        if not detections:
            return []
        text_emb = self.embed_text(text)
        scored = []
        for det in detections:
            x1, y1, x2, y2 = [int(v) for v in det.bbox]
            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(frame.shape[1], x2)
            y2 = min(frame.shape[0], y2)
            if x2 <= x1 or y2 <= y1:
                continue
            crop = frame[y1:y2, x1:x2]
            img_emb = self.embed_image(crop)
            s = float(text_emb @ img_emb)
            if s >= threshold:
                scored.append(Detection(
                    bbox=det.bbox, label=det.label, confidence=det.confidence,
                    track_id=det.track_id, clip_score=s,
                ))
        scored.sort(key=lambda d: d.clip_score or 0, reverse=True)
        return scored


class JEPAEncoder:
    """V-JEPA / I-JEPA visual encoder for self-supervised representations.

    Produces dense feature maps useful for world modeling and planning.
    Requires `timm` and a compatible checkpoint.
    """

    def __init__(self, model_name: str = "vit_base_patch16_224") -> None:
        self._model_name = model_name
        self._model = None
        self._transform = None
        self._lock = RLock()

    def _ensure_loaded(self) -> None:
        if self._model is not None:
            return
        import timm
        import torch
        self._device = "mps" if torch.backends.mps.is_available() else "cuda" if torch.cuda.is_available() else "cpu"
        self._model = timm.create_model(self._model_name, pretrained=True).to(self._device)
        self._model.eval()
        data_cfg = timm.data.resolve_data_config(self._model.pretrained_cfg)
        self._transform = timm.data.create_transform(**data_cfg)

    def encode(self, frame: np.ndarray) -> np.ndarray:
        with self._lock:
            self._ensure_loaded()
            import torch
            from PIL import Image
            pil = Image.fromarray(frame[..., ::-1]) if frame.shape[-1] == 3 else Image.fromarray(frame)
            tensor = self._transform(pil).unsqueeze(0).to(self._device)
            with torch.no_grad():
                features = self._model.forward_features(tensor)
            return features.cpu().numpy()[0]

    def encode_batch(self, frames: list[np.ndarray]) -> np.ndarray:
        with self._lock:
            self._ensure_loaded()
            import torch
            from PIL import Image
            tensors = []
            for f in frames:
                pil = Image.fromarray(f[..., ::-1]) if f.shape[-1] == 3 else Image.fromarray(f)
                tensors.append(self._transform(pil))
            batch = torch.stack(tensors).to(self._device)
            with torch.no_grad():
                features = self._model.forward_features(batch)
            return features.cpu().numpy()


class CosmosWorldModel:
    """Cosmos 3 Nano world model via MLX on Apple Silicon.

    Generates a predicted image from a text prompt using the 4-bit
    quantized Cosmos 3 Nano model. Points at a local cosmos-mac checkout.
    """

    _COSMOS_MAC_DIR = "/Users/dao/Documents/GitHub/cosmos_mac"

    def __init__(self, cosmos_dir: str | None = None) -> None:
        self._cosmos_dir = cosmos_dir or self._COSMOS_MAC_DIR
        self._pipe = None
        self._lock = RLock()

    def _ensure_loaded(self) -> None:
        if self._pipe is not None:
            return
        import sys
        import os
        from pathlib import Path

        cosmos_path = Path(self._cosmos_dir)
        model_dir = cosmos_path / "models" / "Cosmos3-Nano-MLX-4bit"
        if not (model_dir / "transformer").exists():
            raise RuntimeError(f"Cosmos model not found at {model_dir}")

        os.environ.setdefault("PYTORCH_MPS_HIGH_WATERMARK_RATIO", "0.0")
        os.environ.setdefault("TOKENIZERS_PARALLELISM", "false")

        if str(cosmos_path) not in sys.path:
            sys.path.insert(0, str(cosmos_path))

        import torch
        from diffusers import Cosmos3OmniPipeline, AutoencoderKLWan
        from diffusers.schedulers.scheduling_unipc_multistep import UniPCMultistepScheduler
        from transformers import AutoTokenizer
        from mlx_pipeline import MLXCosmos3Transformer

        repo = str(model_dir)
        vae = AutoencoderKLWan.from_pretrained(repo, subfolder="vae", torch_dtype=torch.float32).eval()
        sched = UniPCMultistepScheduler.from_pretrained(repo, subfolder="scheduler")
        tok = AutoTokenizer.from_pretrained(repo, subfolder="text_tokenizer")
        transformer = MLXCosmos3Transformer(str(model_dir / "transformer"), config_dir=repo)

        self._pipe = Cosmos3OmniPipeline(
            transformer=transformer,
            text_tokenizer=tok,
            vae=vae,
            scheduler=sched,
            sound_tokenizer=None,
            enable_safety_checker=False,
        )

    def generate(self, prompt: str = "What happens next in this scene",
                 steps: int = 12, resolution: int = 256, seed: int = 1234) -> np.ndarray:
        """Generate a single image and return it as a BGR numpy array."""
        with self._lock:
            self._ensure_loaded()
            import torch
            generator = torch.Generator().manual_seed(seed)
            result = self._pipe(
                prompt=prompt,
                num_frames=1,
                height=resolution,
                width=resolution,
                num_inference_steps=steps,
                guidance_scale=6.0,
                enable_sound=False,
                add_resolution_template=False,
                add_duration_template=False,
                generator=generator,
                enable_safety_check=False,
            )
            img = result.video[0][0] if isinstance(result.video[0], list) else result.video[0]
            frame = np.array(img)
            if frame.ndim == 3 and frame.shape[2] == 3:
                frame = frame[:, :, ::-1]  # RGB -> BGR for OpenCV
            return frame


class DepthEstimator:
    """Monocular depth estimation via Depth Anything V2 Small.

    Requires `transformers>=4.30` (optional dep: pip install ros-agent[depth]).
    Returns relative depth as a float32 ndarray normalized to [0, 1].
    """

    def __init__(self, model_name: str = "depth-anything/Depth-Anything-V2-Small-hf") -> None:
        self._model_name = model_name
        self._pipe = None
        self._lock = RLock()

    def _ensure_loaded(self) -> None:
        if self._pipe is not None:
            return
        from transformers import pipeline
        import torch
        device = "mps" if torch.backends.mps.is_available() else "cuda" if torch.cuda.is_available() else "cpu"
        self._pipe = pipeline(
            "depth-estimation",
            model=self._model_name,
            device=device,
            dtype=torch.float32,
        )

    def estimate(self, frame: np.ndarray) -> np.ndarray:
        """Return depth map as float32 ndarray with shape (H, W), values in [0, 1]."""
        with self._lock:
            self._ensure_loaded()
            from PIL import Image
            pil = Image.fromarray(frame[..., ::-1]) if frame.ndim == 3 and frame.shape[2] == 3 else Image.fromarray(frame)
            result = self._pipe(pil)
            depth = np.array(result["depth"], dtype=np.float32)
            lo, hi = depth.min(), depth.max()
            if hi > lo:
                depth = (depth - lo) / (hi - lo)
            else:
                depth = np.zeros_like(depth)
            return depth

    @staticmethod
    def is_available() -> bool:
        try:
            import transformers  # noqa: F401
            return True
        except ImportError:
            return False


# Keep old name as alias for backwards compat
CosmosTokenizer = CosmosWorldModel


MODEL_REGISTRY: dict[str, type] = {
    "yolo": YOLODetector,
    "clip": CLIPMatcher,
    "jepa": JEPAEncoder,
    "cosmos": CosmosWorldModel,
    "depth": DepthEstimator,
}
