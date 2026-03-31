# CF_AnnController

Out-of-tree neural network flight controller for the Crazyflie 2.1+ nano-drone, deployed on STM32 using ST-AI featuring ANN inference at 100 Hz, PID-assisted takeoff, and a Python ground station.

## Requirements

### Hardware
- Crazyflie 2.1+
- Flow deck v2
- Crazyradio PA

### Software
- Python 3.11 (tested)
- Python dependencies (managed via Conda, see Setup section)
- Crazyflie firmware toolchain prerequisites
- ST Edge AI Core 4.0

## Repository Structure

```text
.
├── environment.yml
├── Makefile
├── README.md
├── external/
│   └── crazyflie-firmware/ 
├── scripts/
│   ├── client.py            # Custom Python client
│   ├── controller.py        # Sends commands
│   ├── logger.py            
│   └── plotter.py          
└── src/
    ├── ann_controller.c     # Main OOT controller
    ├── ann_controller.h
    ├── Kbuild         
    └── ST_nn/               # Neural network 
```
## Setup

### 1) Install STEdgeAI Core
1. Go to the [STEdgeAI Core download page](https://www.st.com/en/development-tools/stedgeai-core.html#section-get-software-table) and download STEAICore-Linux.
2. Install the software. **You must use the default installation directory** (`/opt/ST/STEdgeAI/`) for the setup script to work correctly.

### 2) Clone and Initialize
Clone the repository and run the setup script. This will automatically initialize the required submodules, check your toolchain, and link the ST neural network runtime library:
```bash
git clone https://github.com/100nze/Crazyflie_ANN_controller.git
cd Crazyflie_ANN_controller
chmod +x setup.sh
./setup.sh
```

### 3) Compile and Upload the Firmware
Configure and compile the firmware using the commands suggested by the setup script:
```bash
make cf2_defconfig
make -j$(nproc)
```

### 4) Create the Python Environment 
Inside the repository root folder, create and activate the Conda environment to install all required dependencies for the ground station:
```bash
conda env create -n <env_name> -f environment.yml
conda activate <env_name>
```
With the virtual environment active put the Crazyflie in bootloader mode (turn it off, then hold the power button until the blue LED blinks). Make sure the Crazyradio PA is plugged into your PC, then execute:
```bash
cfloader flash build/cf2.bin stm32-fw
```
If you have troubles using the Crazyradio refer to [USB permissions troubleshooting](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/).

After flashing the firmware you can run the custom client script or the standard Crazyflie client :
```bash
# Run the official Crazyflie client
cfclient 

# Or run the custom client script
python scripts/client.py 
```

### 5) Use your Custom Model (Optional)
If you want to deploy a different neural network, add the STEdgeAI tools to your system path:
```bash
echo 'export PATH="$PATH:/opt/ST/STEdgeAI/4.0/Utilities/linux"' >> ~/.bashrc
source ~/.bashrc
```
Navigate to the folder containing your trained model (`.onnx`, Keras, or TensorFlow) and run the generation tool:
```bash
stedgeai generate --model <model_name> --output <output_folder> --optimization balanced --target stm32f4
```
Once generated, simply copy the resulting `.c` and `.h` files from your `<output_folder>` into the `src/ST_nn/` directory of this repository, replacing the existing files. (The `.a` runtime library is already handled by `setup.sh`).

## Fly the Drone

### Configure the Connection and Start

Before flying, you must set your drone's specific radio address. Open `scripts/client.py` and modify the `URI` variable to match your Crazyflie's configuration:
```python
URI = 'radio://0/50/2M/E7E7E7E7E8' # Change this to your drone's URI
```

Make sure your Conda environment is active and the Crazyradio PA is plugged in. Start the custom client from the root directory:
```bash
python scripts/client.py
```

This script will connect to the drone, start the telemetry logger, create a folder called `flight_log`, and automatically open a live-updating plot of the flight.

A HUD will appear in the terminal showing key telemetry data, and you can communicate with the drone. for the list of the commands press 'h'.

![](<images/Hud.png>)

### View Past Flights
If you want to review the logging of a previous flight offline, you can run the plotter script:
```bash
python scripts/plotter.py flight_log/<csv_name>
```

![](<images/flight_log.png>)

## Controller performances


- **Inference Time**: The neural network inference takes an average of **~869 µs** per step with a variance of just **9.8 µs²**.
- **CPU Load**: The stabilize task consumes only **~4.2%** of the CPU. The system idle load remains above **72%**.
- **Memory Footprint**: The neural network requires **58.1 KB** of Flash memory. In total, the entire custom firmware occupies only **356 KB**
- **RAM Usage**: The ST-AI runtime requires only **840 Bytes** of RAM for the input/output buffers and intermediate activation tensors. The overall firmware RAM usage remains highly safe at **68%**.

## Credits

- **Bitcraze**: For the [Crazyflie 2.1](https://www.bitcraze.io/) hardware platform, the open-source `crazyflie-firmware`, and the `crazyflie-lib-python` used for the ground station.
- **STMicroelectronics**: For the **ST Edge AI Core** used to generate the optimized C-code for neural network inference on the STM32 microcontroller.
- **Learning to Fly in Seconds**: Their repository was used as a technical reference for integrating and deploying neural network policies within the Crazyflie firmware.
- **Guazzaloca, M.** (2025). *Learning-Based Control for Nano-Drones Flight: From Simulation to Reality* (Master's thesis). Alma Mater Studiorum - University of Bologna, Department of Electrical, Electronic, and Information Engineering "Guglielmo Marconi".
