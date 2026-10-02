FROM python:3.12-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

WORKDIR /workspace

RUN apt-get update \
    && apt-get install -y --no-install-recommends build-essential \
    && rm -rf /var/lib/apt/lists/*

COPY pyproject.toml README.md ./
COPY src/ ./src/
COPY tests/ ./tests/
COPY controller/ ./controller/
COPY config/ ./config/
COPY docs/ ./docs/
COPY scripts/ ./scripts/

RUN python -m pip install --no-cache-dir -e ".[test]"

CMD ["wam-run-policy", "--help"]