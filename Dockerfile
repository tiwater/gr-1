FROM ccr.ccs.tencentyun.com/fftai/fftai-webots:1.1
ARG PROJECT_PATH
RUN mkdir -p $PROJECT_PATH
COPY . $PROJECT_PATH

# Use a script to start the services
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod 755 /usr/local/bin/entrypoint.sh
CMD ["/bin/bash", "/usr/local/bin/entrypoint.sh"]