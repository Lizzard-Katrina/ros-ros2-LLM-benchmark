"""Thin wrapper around the OpenRouter chat-completions API.

Mirrors the client setup already used elsewhere in this repo
(pipeline.py, run_all_5.py) so behavior stays consistent.
"""

import os

from openai import OpenAI

DEFAULT_MODEL = "anthropic/claude-opus-4.6"


def get_client() -> OpenAI:
    api_key = os.environ.get("OPENROUTER_API_KEY", "")
    if not api_key:
        raise RuntimeError("OPENROUTER_API_KEY environment variable is not set.")
    return OpenAI(api_key=api_key, base_url="https://openrouter.ai/api/v1")


def chat(client: OpenAI, system_prompt: str, user_prompt: str, model: str = DEFAULT_MODEL,
         temperature: float = 0.1, timeout: int = 300) -> str:
    response = client.chat.completions.create(
        model=model,
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt},
        ],
        temperature=temperature,
        timeout=timeout,
    )
    return response.choices[0].message.content
