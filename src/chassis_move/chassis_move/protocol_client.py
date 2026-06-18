"""调度系统到机器人 HTTP 协议的轻量客户端。"""

from __future__ import annotations

import json
from urllib import error, request


class ProtocolHttpClient:
    """用于机器人调度协议的轻量级 JSON HTTP 客户端。"""

    def __init__(
        self,
        base_url: str,
        timeout_sec: float,
        headers: dict[str, str] | None = None,
    ) -> None:
        self._base_url = base_url.rstrip('/')
        self._timeout_sec = timeout_sec
        self._headers = {
            'Content-Type': 'application/json',
        }
        if headers:
            self._headers.update(headers)

    def post(self, path: str, payload: dict[str, object]) -> dict[str, object]:
        """发送 POST JSON 请求，并返回解析后的 JSON 响应。"""
        url = self._base_url + path
        http_request = request.Request(
            url=url,
            data=json.dumps(payload, ensure_ascii=True, separators=(',', ':')).encode(
                'utf-8'
            ),
            headers=self._headers,
            method='POST',
        )

        try:
            with request.urlopen(http_request, timeout=self._timeout_sec) as response:
                body = response.read().decode('utf-8')
        except error.HTTPError as exc:
            body = exc.read().decode('utf-8')
            raise RuntimeError(
                f'HTTP {exc.code} calling {path}: {body or exc.reason}'
            ) from exc
        except error.URLError as exc:
            raise RuntimeError(f'Failed to reach {url}: {exc.reason}') from exc

        if not body:
            return {'code': 0}

        try:
            parsed = json.loads(body)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f'Invalid JSON response from {url}: {body}') from exc

        if isinstance(parsed, dict):
            return parsed
        raise RuntimeError(f'Unexpected JSON response type from {url}: {parsed!r}')
