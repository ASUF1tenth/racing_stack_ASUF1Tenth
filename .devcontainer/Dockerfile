#
# Container Creation for the F110 race-stack at Center for Project Based Learning, ETH
#
ARG BASE_IMAGE=base_x86
FROM race_stack:${BASE_IMAGE}

# initial user setup
ARG USER
ARG UID
ARG GID

RUN useradd -ms /bin/bash -u ${UID} -G sudo ${USER} && groupadd -fg ${GID} ${USER} \
    && echo "${USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USER

ADD .install_utils/bashrc/ /tmp/.install_utils/
RUN echo "export USERNAME=$USER" >> /home/$USER/.bashrc &&\
    cat /tmp/.install_utils/.bashrc_git >> /home/$USER/.bashrc &&\
    cat /tmp/.install_utils/.bashrc_forzaeth >> /home/$USER/.bashrc

USER root
# Clean up
RUN rm -rf /tmp/.install_utils

RUN chown -R $USER:$USER /home/ws
USER $USER

CMD ["/bin/bash"]
