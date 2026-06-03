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


class CosmosTokenizer:
    """NVIDIA Cosmos discrete visual tokenizer.

    Tokenizes video frames into discrete tokens for world model training.
    Requires `cosmos-tokenizer` package.
    """

    def __init__(self, encoder_name: str = "Cosmos-Tokenizer-DV4x8x8") -> None:
        self._encoder_name = encoder_name
        self._encoder = None
        self._lock = RLock()

    def _ensure_loaded(self) -> None:
        if self._encoder is not None:
            return
        try:
            from cosmos_tokenizer.video_lib import CausalVideoTokenizer
            self._encoder = CausalVideoTokenizer(
                checkpoint_enc=f"pretrained_ckpts/{self._encoder_name}/encoder.jit",
            )
        except ImportError:
            raise ImportError(
                "cosmos-tokenizer not installed. Run: pip install cosmos-tokenizer"
            )
        except Exception:
            self._encoder = "stub"

    def tokenize(self, frames: np.ndarray) -> Any:
        with self._lock:
            self._ensure_loaded()
            if self._encoder == "stub":
                return {"tokens": [], "note": "Cosmos checkpoint not found — download from NVIDIA"}
            import torch
            tensor = torch.from_numpy(frames).float()
            if tensor.ndim == 3:
                tensor = tensor.unsqueeze(0).unsqueeze(0)
            elif tensor.ndim == 4:
                tensor = tensor.unsqueeze(0)
            tensor = tensor.permute(0, 1, 4, 2, 3) / 255.0
            indices, codes = self._encoder.encode(tensor)
            return {
                "indices": indices.cpu().numpy().tolist(),
                "shape": list(indices.shape),
            }


MODEL_REGISTRY: dict[str, type] = {
    "yolo": YOLODetector,
    "clip": CLIPMatcher,
    "jepa": JEPAEncoder,
    "cosmos": CosmosTokenizer,
}
