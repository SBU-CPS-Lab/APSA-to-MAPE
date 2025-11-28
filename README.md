# APSA-to-MAPE
APSA-to-MAPE

This repository contains two main components used to simulate and execute an APSA-based smart home system.
The workflow consists of running a home environment simulation first, and then running the APSA model implemented in ForSyDe/SADF.

📁 Folder Structure
1. sinergym/

This folder contains the smart-home environment simulation.
The environment is executed using Docker and provides the external signals and conditions required by the APSA model.

2. ForSyDe-systemC-SmartHome/

This folder contains the implementation of the APSA smart home using ForSyDe::SADF (SystemC).
The model interacts with the environment and generates APSA-based decisions.

🔧 Running the Environment (Sinergym)

To start the home simulation environment, run the following command:

docker run --rm -v "$PWD":/app -w /app -p 9000:9000 sinergym:latest python3 -u environment.py

This will launch the simulation on port 9000, using your current directory as /app inside the container.

⚙️ Running the APSA Model (ForSyDe/SADF)

After the environment is running, compile the APSA SystemC model:

g++ main.cpp -o output -lsystemc  -I #forsyde source files address


Notes:

The -lsystemc flag links the SystemC library.

The -I path must point to your local installation of
ForSyDe-SystemC-DigitalTwin/src/.
Update this path according to your system.

To run the compiled model:

./output

▶️ Execution Order

Start the Sinergym environment

python3 environment.py   # (inside Docker as shown above)

Compile and run the APSA model
./output

The APSA model expects the environment to be running first, as it reads input values produced by the simulation.

