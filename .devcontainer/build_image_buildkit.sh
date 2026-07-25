docker build --platform linux/arm64 \                                    
    --build-arg USERNAME=$USER \                                           
    --build-arg UID=$(id -u) \
    --build-arg GID=$(id -g) \
    -t nuc_forzaeth_racestack_ros2:jazzy \
    -f .devcontainer/Dockerfile .