FROM photoneo/binpicking:0.3.0

RUN sudo apt-get update && \
    sudo apt-get install -y firefox qtcreator screen

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
