Branch Predictor Simulator Implementation
I implemented a Level 2 Branch Predictor Simulator as part of this project.
The simulator is designed to predict branch outcomes in a program execution trace,mimicking the behavior of a real-world branch predictor.
The configuration of the predictor is highly flexible and can be customized using parameters defined at the start of the simulation.

Key Features:
Dynamic Configuration: 
The predictor's behavior is determined by parameters provided in an input file, allowing for customizable simulation setups.

Trace-Based Simulation: 
The simulator processes a trace file containing program execution events, including command addresses, jump decisions, and calculated destination addresses.

Prediction and Update:
For each branch event, the simulator provides a prediction based on the command address. 
It then updates its internal state using the actual branch decision (from the EXE phase) as detailed in the trace.

How It Works:
Input File: The simulation reads an input file containing the predictor parameters and the execution trace.

Prediction: For every branch event, the simulator predicts the outcome using the command address.

Update: After the actual branch decision is known, the simulator updates its state to improve future predictions.

This project demonstrates my understanding of branch prediction mechanisms and my ability to implement complex simulation environments. 
The code is modular, well-documented, and designed for easy experimentation with different predictor configurations.
