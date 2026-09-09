# ADAPT-X backend - development container.
#
# Runs the FastAPI backend only. CARLA is not containerised at this stage and
# stays optional; the backend starts and reports CARLA: DISCONNECTED without it.

FROM python:3.13-slim

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

WORKDIR /app

# Dependency metadata first, so a source-only change does not reinstall
# everything on rebuild.
COPY pyproject.toml README.md ./
COPY src ./src
RUN pip install --no-cache-dir .

# Run as a non-root user.
RUN useradd --create-home --uid 1000 adaptx \
    && chown -R adaptx:adaptx /app
USER adaptx

# Container defaults; every value stays overridable through the environment.
ENV ADAPTX_API__HOST=0.0.0.0 \
    ADAPTX_API__PORT=8000 \
    ADAPTX_API__RELOAD=false \
    ADAPTX_LOGGING__JSON_FORMAT=true

EXPOSE 8000

HEALTHCHECK --interval=30s --timeout=5s --start-period=10s --retries=3 \
    CMD python -c "import urllib.request, sys; sys.exit(0 if urllib.request.urlopen('http://localhost:8000/health', timeout=3).status == 200 else 1)"

CMD ["python", "-m", "adaptx"]
