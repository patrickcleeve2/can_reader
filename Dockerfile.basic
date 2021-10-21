FROM python:3.8-slim

COPY can_reader_demo.py /app/can_reader_demo.py

RUN pip3 install pyserial

CMD ["python3", "-u", "/app/can_reader_demo.py"]