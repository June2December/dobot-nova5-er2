# Gemini Robotics ER 2 helpers. Coordinate convention matches GR-ER2:
# image points are normalized 0..1000 in [y, x] order (not [x, y]).
import json
import math
import os
import time
import uuid

PICK_PLACE_TOOL = {
    'type': 'function',
    'name': 'pick_place',
    'description': (
        'Pick the object at pick and place it at place. '
        'Coordinates are normalized [y,x] in 0..1000 on the current camera image. '
        'One call per observation. Both pixels refer to this image only.'
    ),
    'parameters': {
        'type': 'object',
        'properties': {
            'pick': {
                'type': 'array', 'items': {'type': 'number'},
                'minItems': 2, 'maxItems': 2,
            },
            'place': {
                'type': 'array', 'items': {'type': 'number'},
                'minItems': 2, 'maxItems': 2,
            },
            'observation_id': {'type': 'string'},
        },
        'required': ['pick', 'place', 'observation_id'],
        'additionalProperties': False,
    },
}


def new_observation_id():
    return uuid.uuid4().hex[:16]


def load_gemini_env():
    for key in ('GEMINI_API_KEY', 'GOOGLE_API_KEY'):
        if os.getenv(key):
            return
    env_path = os.path.expanduser('~/.config/dobot_er2.env')
    for path in (env_path, os.path.join(os.getcwd(), '.env')):
        if not os.path.isfile(path):
            continue
        with open(path, encoding='utf-8') as handle:
            for line in handle:
                line = line.strip()
                if not line or line.startswith('#') or '=' not in line:
                    continue
                name, value = line.split('=', 1)
                name = name.strip()
                if name in ('GEMINI_API_KEY', 'GOOGLE_API_KEY'):
                    os.environ.setdefault(name, value.strip().strip('"\''))


def point_yx(value):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError('좌표는 [y, x] 두 값이어야 합니다.')
    yx = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ValueError('좌표는 숫자여야 합니다.')
        item = float(item)
        if not math.isfinite(item) or not 0.0 <= item <= 1000.0:
            raise ValueError('좌표는 0~1000이어야 합니다.')
        yx.append(item)
    return yx


def norm_to_pixel(yx, width, height):
    y_n, x_n = point_yx(yx)
    u = x_n * (width - 1) / 1000.0
    v = y_n * (height - 1) / 1000.0
    return int(round(u)), int(round(v))


def parse_json_maybe(text):
    text = (text or '').strip()
    if text.startswith('```'):
        text = text.split('\n', 1)[1].rsplit('```', 1)[0].strip()
    return json.loads(text)


def response_text(response):
    text = getattr(response, 'output_text', None)
    if text:
        return text
    items = getattr(response, 'outputs', None) or getattr(response, 'steps', None) or []
    parts = []
    for item in items:
        if getattr(item, 'type', '') == 'text' and getattr(item, 'text', None):
            parts.append(item.text)
    return '\n'.join(parts)


def function_calls(response):
    items = getattr(response, 'outputs', None) or getattr(response, 'steps', None) or []
    return [c for c in items if getattr(c, 'type', '') == 'function_call']


def make_client(timeout_ms=120000):
    load_gemini_env()
    if not (os.getenv('GEMINI_API_KEY') or os.getenv('GOOGLE_API_KEY')):
        raise RuntimeError(
            'GEMINI_API_KEY가 없습니다. export 하거나 ~/.config/dobot_er2.env 에 넣으세요.')
    from google import genai
    return genai.Client(http_options={'timeout': timeout_ms})


def infer_pick_place(api, model, png_bytes, prompt, tools=None):
    import base64
    image_part = {
        'type': 'image',
        'data': base64.b64encode(png_bytes).decode(),
        'mime_type': 'image/png',
    }
    kwargs = {
        'model': model,
        'input': [image_part, {'type': 'text', 'text': prompt}],
        'generation_config': {
            'thinking_level': 'medium',
            'max_output_tokens': 4096,
        },
    }
    if tools:
        kwargs['tools'] = tools
    started = time.monotonic()
    response = api.interactions.create(**kwargs)
    return response, time.monotonic() - started
