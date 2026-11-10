"""Optional real-camera effects for the simulated colour image.

MuJoCo renders a sharp, noise-free, perfectly exposed frame. A walking robot's
camera gives none of that. With every setting at its default the image is
returned unchanged. Effects, in the order applied:

  motion blur  from the camera's angular velocity during `exposure_s`
               (pixels = focal_px * |omega| * exposure_s, along the image flow)
  defocus      Gaussian blur, sigma `defocus_px`
  exposure     gain `exposure_gain` times a per-frame jitter in
               [1 - exposure_jitter, 1 + exposure_jitter]; clipping washes out
               white surfaces such as the ball
  noise        Gaussian sensor noise, `noise_std` on the 0-255 scale
  compression  JPEG round trip at `jpeg_quality` (0 = off)

Only colour is degraded; depth stays the ideal rendered depth.
"""
from __future__ import annotations

from dataclasses import dataclass, fields

import numpy as np


@dataclass
class DegradationSettings:
    exposure_s: float = 0.0
    defocus_px: float = 0.0
    exposure_gain: float = 1.0
    exposure_jitter: float = 0.0
    noise_std: float = 0.0
    jpeg_quality: int = 0
    seed: int = 0

    @property
    def enabled(self) -> bool:
        return (self.exposure_s > 0 or self.defocus_px > 0 or self.exposure_gain != 1.0
                or self.exposure_jitter > 0 or self.noise_std > 0 or self.jpeg_quality > 0)

    @classmethod
    def names(cls) -> list[str]:
        return [f.name for f in fields(cls)]


def motion_kernel(dx: float, dy: float) -> np.ndarray | None:
    """Normalised line kernel for an image-plane smear of (dx, dy) pixels."""
    length = float(np.hypot(dx, dy))
    if length < 1.0:
        return None
    import cv2
    size = int(np.ceil(length)) | 1
    kernel = np.zeros((size, size), np.float32)
    c = size // 2
    ux, uy = dx / length * (size - 1) / 2, dy / length * (size - 1) / 2
    cv2.line(kernel, (int(round(c - ux)), int(round(c - uy))),
             (int(round(c + ux)), int(round(c + uy))), 1.0, 1, cv2.LINE_AA)
    return kernel / kernel.sum()


class ImageDegrader:

    def __init__(self, settings: DegradationSettings, focal_px: float) -> None:
        self.s = settings
        self.focal_px = float(focal_px)
        self.rng = np.random.default_rng(settings.seed)
        self._noise = None
        if settings.noise_std > 0:
            import cv2
            cv2.setRNGSeed(int(settings.seed))   # cv2.randn uses OpenCV's global RNG

    def blur_vector(self, omega_cam: np.ndarray) -> tuple[float, float]:
        """Image smear in pixels for MuJoCo camera-frame angular velocity (x right, y up, z back).

        Rotation about camera x moves the image vertically, about y horizontally;
        roll about z is ignored (small for this head and centred subjects).
        """
        t = self.s.exposure_s
        return (self.focal_px * float(omega_cam[1]) * t, self.focal_px * float(omega_cam[0]) * t)

    def apply(self, rgb: np.ndarray, omega_cam=None) -> np.ndarray:
        if not self.s.enabled:
            return rgb
        # uint8 OpenCV calls only: they release the GIL, which the physics
        # loop in this process needs (numpy float math here cost ~27 ms/frame).
        import cv2
        image = rgb
        if self.s.exposure_s > 0 and omega_cam is not None:
            kernel = motion_kernel(*self.blur_vector(np.asarray(omega_cam, float)))
            if kernel is not None:
                image = cv2.filter2D(image, -1, kernel, borderType=cv2.BORDER_REFLECT)
        if self.s.defocus_px > 0:
            image = cv2.GaussianBlur(image, (0, 0), self.s.defocus_px)
        gain = self.s.exposure_gain
        if self.s.exposure_jitter > 0:
            gain *= 1.0 + self.rng.uniform(-self.s.exposure_jitter, self.s.exposure_jitter)
        if gain != 1.0:
            image = cv2.convertScaleAbs(image, alpha=gain)
        if self.s.noise_std > 0:
            if self._noise is None or self._noise.shape != image.shape:
                self._noise = np.empty(image.shape, np.int16)
            cv2.randn(self._noise, 0, self.s.noise_std)
            image = cv2.add(image, self._noise, dtype=cv2.CV_8U)
        if self.s.jpeg_quality > 0:
            ok, buf = cv2.imencode('.jpg', cv2.cvtColor(image, cv2.COLOR_RGB2BGR),
                                   [cv2.IMWRITE_JPEG_QUALITY, int(self.s.jpeg_quality)])
            if ok:
                image = cv2.cvtColor(cv2.imdecode(buf, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
        return image if image is not rgb else rgb.copy()
