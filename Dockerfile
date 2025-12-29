FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# -------------------------
# Base deps + SystemC 2.3.4 (from Ubuntu repo)
# -------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    git \
    wget \
    unzip \
    pkg-config \
    python3 \
    python3-pip \
    gdb \
    nano \
    binutils \
    libboost-all-dev \
    libsystemc-dev \
    libsystemc-doc \
 && rm -rf /var/lib/apt/lists/*

# -------------------------
# SystemC environment
# -------------------------
ENV SYSTEMC_HOME=/usr
ENV SYSTEMC_INSTALL=/usr
ENV CXX=g++
ENV CXXFLAGS="-O3 -std=c++17"
ENV LD_LIBRARY_PATH=/usr/lib:/usr/lib/x86_64-linux-gnu
ENV CMAKE_PREFIX_PATH=/usr

# -------------------------
# ForSyDe-SystemC
# -------------------------
WORKDIR /opt
ARG FORSYDE_REPO=https://github.com/MaryamSam/ForSyDe-SystemC-DigitalTwin.git
RUN git clone --depth 1 ${FORSYDE_REPO} ForSyDe-SystemC
ENV FORSYDE_HOME=/opt/ForSyDe-SystemC

# -------------------------
# Add APSA-to-MAPE under examples/sadf (DO NOT DELETE existing content)
# -------------------------
ARG GH_TOKEN
ARG APSA_REPO=https://github.com/SBU-CPS-Lab/APSA-to-MAPE.git

RUN set -eux; \
    mkdir -p "${FORSYDE_HOME}/examples/sadf"; \
    if [ ! -d "${FORSYDE_HOME}/examples/sadf/APSA-to-MAPE/.git" ]; then \
      git clone --depth 1 "https://${GH_TOKEN}@github.com/SBU-CPS-Lab/APSA-to-MAPE.git" \
        "${FORSYDE_HOME}/examples/sadf/APSA-to-MAPE"; \
    fi

# -------------------------
# Convenience
# -------------------------
WORKDIR /workspace
CMD ["/bin/bash"]
