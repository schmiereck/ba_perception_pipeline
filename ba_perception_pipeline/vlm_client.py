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
    "and ask you to identify an object.\n"
    "Locate the center of the requested object using normalized coordinates "
    "where [0, 0] is the top-left and [1000, 1000] is the bottom-right corner "
    "of the visible image area.\n"
    "Respond ONLY with a JSON object in this format: "
    '{"u": <0-1000>, "v": <0-1000>}\n'
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
        """Detect the target object in the image using direct scaling."""
        h, w = bgr.shape[:2]
        image_b64 = _encode_bgr_as_jpeg_b64(bgr)
        self._log(f'VLM query: "{prompt}" ({w}x{h})')

        raw_response = self._call_fn(
            image_b64, prompt, self._model, self._api_key)
        self._log(f'VLM response: {raw_response!r}')

        # 1. Parse normalized coordinates (0-1000)
        u_norm, v_norm = _parse_pixel_response(raw_response)
        
        # 2. Convert to actual pixels (independent scaling for u and v)
        u = int(u_norm * w / 1000.0)
        v = int(v_norm * h / 1000.0)
        
        # Clamp to original image bounds
        u = max(0, min(w - 1, u))
        v = max(0, min(h - 1, v))
        
        self._log(f'VLM normalized: ({u_norm}, {v_norm}) -> pixel: u={u}, v={v}')
        return u, v
