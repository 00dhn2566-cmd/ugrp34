# C++ 제어기 리눅스 빌드/검증 컨테이너 (Gazebo 머신 타깃 이식성 보증)
# 빌드:  docker build -f docker/Dockerfile.cpp -t qc-cpp .        (control_seoungjin/에서)
# 실행:  docker run --rm qc-cpp                                    (스모크)
#        docker run --rm -v "$PWD/output:/data" qc-cpp ./build/qc_trace --io-test /data/trajectory.json /tmp
FROM debian:bookworm-slim
RUN apt-get update && apt-get install -y --no-install-recommends \
        g++ cmake make \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY controller_cpp/ /app/
RUN cmake -B build && cmake --build build -j
CMD ["./build/qc_trace", "--smoke"]
