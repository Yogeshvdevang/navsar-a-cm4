# CM4 Setup (PiCam + VO)

This setup targets Raspberry Pi OS on CM4 and uses the PiCam (OV9281) via Picamera2/libcamera.

## System Packages (one time)

```bash
sudo apt update
sudo apt install -y libcamera-apps python3-libcamera python3-picamera2 python3-venv
```

## Project Venv (system Python)

Create a venv that can see the system `picamera2` module:

```bash
cd ~/NAVSAR-A
python3 -m venv --system-site-packages venv
source venv/bin/activate
```

## Python Dependencies

```bash
pip install -r requirements.txt
pip install smbus2
```

## Run

```bash
PYTHONPATH=src python -m navisar.main
```

## VS Code Interpreter

Select the venv interpreter:

`Ctrl+Shift+P` → `Python: Select Interpreter` → `./venv/bin/python`
