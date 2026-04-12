"""VLM client for target pixel detection (no ROS2 dependency).

Sends an image + text prompt to a vision-language model and parses the
response to extract a target pixel coordinate (u, v).

Supported providers:
- ``groq``  — Llama 4 Scout via Groq API (fast, cheap)
- ``gemini`` — Google Gemini via google-genai SDK

The provider is selected at construction time via the ``provider``
parameter and can be switched via the config YAML at launch time.
"""

import base64
import json
import re
from typing import Callable

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Prompt template
# ---------------------------------------------------------------------------
_SYSTEM_PROMPT = (
    "You are a robotic vision assistant. The user will show you an image "
    "and ask you to identify an object. Respond with ONLY the pixel "
    "coordinates of the center of the requested object in the format: "
    '{"u": <column>, "v": <row>}\n'
    "Do not include any other text."
)


def _encode_bgr_as_jpeg_b64(bgr: np.ndarray, quality: int = 85) -> str:
    """Encode a BGR image as a base64 JPEG string."""
    ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError('Failed to encode image as JPEG')
    return base64.b64encode(buf.tobytes()).decode('ascii')


def _parse_pixel_response(text: str) -> tuple[int, int]:
    """Extract (u, v) from the model response text.

    Tries JSON first, then falls back to regex for ``u=123, v=456``
    style responses.
    """
    # Try JSON
    try:
        data = json.loads(text)
        return int(data['u']), int(data['v'])
    except (json.JSONDecodeError, KeyError, TypeError):
        pass

    # Try JSON embedded in markdown code block
    m = re.search(r'\{[^}]*"u"\s*:\s*(\d+)[^}]*"v"\s*:\s*(\d+)[^}]*\}', text)
    if m:
        return int(m.group(1)), int(m.group(2))

    # Try plain number pairs like "u: 320, v: 240" or "(320, 240)"
    m = re.search(r'(\d+)\s*[,;]\s*(\d+)', text)
    if m:
        return int(m.group(1)), int(m.group(2))

    raise ValueError(f'Could not parse pixel coordinates from VLM response: {text!r}')


# ---------------------------------------------------------------------------
# Provider implementations
# ---------------------------------------------------------------------------

def _call_groq(
    image_b64: str,
    prompt: str,
    model: str,
    api_key: str,
) -> str:
    """Call Groq API with an image + text prompt."""
    from groq import Groq

    client = Groq(api_key=api_key)
    response = client.chat.completions.create(
        model=model,
        messages=[
            {'role': 'system', 'content': _SYSTEM_PROMPT},
            {
                'role': 'user',
                'content': [
                    {
                        'type': 'image_url',
                        'image_url': {
                            'url': f'data:image/jpeg;base64,{image_b64}',
                        },
                    },
                    {'type': 'text', 'text': prompt},
                ],
            },
        ],
        temperature=0.0,
        max_tokens=100,
    )
    return response.choices[0].message.content


def _call_gemini(
    image_b64: str,
    prompt: str,
    model: str,
    api_key: str,
) -> str:
    """Call Google Gemini API with an image + text prompt."""
    from google import genai

    client = genai.Client(api_key=api_key)
    response = client.models.generate_content(
        model=model,
        contents=[
            {
                'parts': [
                    {'text': f'{_SYSTEM_PROMPT}\n\n{prompt}'},
                    {
                        'inline_data': {
                            'mime_type': 'image/jpeg',
                            'data': image_b64,
                        },
                    },
                ],
            },
        ],
    )
    return response.text


_PROVIDERS = {
    'groq': _call_groq,
    'gemini': _call_gemini,
}


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

class VLMClient:
    """Vision-language model client for target pixel detection.

    Parameters
    ----------
    provider : str
        ``"groq"`` or ``"gemini"``.
    model : str
        Model identifier for the chosen provider.
    api_key : str
        API key for the provider.
    log_fn : callable, optional
        Logging function.
    """

    def __init__(
        self,
        provider: str,
        model: str,
        api_key: str,
        log_fn: Callable[[str], None] | None = None,
    ) -> None:
        if provider not in _PROVIDERS:
            raise ValueError(
                f'Unknown VLM provider {provider!r}. '
                f'Supported: {list(_PROVIDERS.keys())}')

        self._provider = provider
        self._model = model
        self._api_key = api_key
        self._call_fn = _PROVIDERS[provider]
        self._log = log_fn or print

        self._log(f'VLM client: provider={provider}, model={model}')

    def detect_target(
        self,
        bgr: np.ndarray,
        prompt: str,
    ) -> tuple[int, int]:
        """Detect the target object in the image.

        Parameters
        ----------
        bgr : np.ndarray
            Input image in BGR format (H×W×3, uint8).
        prompt : str
            Natural-language description of the target, e.g.
            ``"the red cup"``.

        Returns
        -------
        tuple[int, int]
            Pixel coordinates ``(u, v)`` of the target center.
        """
        image_b64 = _encode_bgr_as_jpeg_b64(bgr)
        self._log(f'VLM query: "{prompt}" ({len(image_b64) // 1024} kB)')

        raw_response = self._call_fn(
            image_b64, prompt, self._model, self._api_key)
        self._log(f'VLM response: {raw_response!r}')

        u, v = _parse_pixel_response(raw_response)
        self._log(f'VLM target pixel: u={u}, v={v}')
        return u, v
