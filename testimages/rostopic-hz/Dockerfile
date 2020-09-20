FROM ros:melodic-ros-core-bionic

RUN apt update && apt install -y --no-install-recommends \
    psmisc

COPY start.sh /
RUN chmod +x /start.sh

CMD [ "/start.sh" ]
