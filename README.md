# PSO Follow Line Deep Learning Simulator

A Processing simulator of a differential line-following robot with traditional control (PID), machine learning (recurrent neural networks), and PSO-based weight optimization (Particle Swarm Optimization). Includes a simplified physical model of the robot, virtual sensors, and an interactive interface for parameter visualization and tuning.

## Main Features
- Differential-drive robot with dynamic and kinematic model
- Odometry and virtual front/side sensors for line detection
- PID control and Recurrent Neural Network (RNN) control
- In-simulation PSO optimization of neural network weights
- Menu interface for adjusting evolutionary parameters and viewing information
- Optional logging of simulation data into .txt files

---

## Requirements
- Processing 4 (Java Mode)
- Processing Sound library (used for audio feedback)
  - Install at: Sketch > Import Library… > Add Library… > search for “Sound” and install “Sound” (Processing Foundation)

Media files (images) are included in the project’s `data/` folder.

## How to Run
1. Install Processing 4 and the Sound library (see above).
2. Open the project in Processing: File > Open… and select this repository’s directory (the `.pde` tabs will load automatically).
3. Make sure the image files exist in `data/`:
   - `data/Iron Cup 2019.png`
   - `data/Ratão 4.0.png`
4. Run the sketch (Play button or Ctrl/Cmd + R). The main file is `PSOFollowLineDeepLearning04.pde`.

**Optional (command line):** if you have `processing-java` installed, you can run:
```
processing-java --sketch="/path/to/this/project" --run
```

## Controls During Simulation

### Keyboard:
- **S / N**: toggle sound on/off
- **A / D**: toggle trail drawing
- **M / U**: show/hide multiple “ghost” robots (visual comparison)
- **I / O**: open PID interface (informational text)
- **C**: toggle a guiding circle under the mouse

### Mouse:
- **Right-click**: reposition the robot (center)
- **Top bar**: click to open the menu; hover tabs to display content; in some tabs you can drag sliders to adjust parameters

## Menu and Parameters
The top menu has tabs:
- **Genetic Algorithm / PSO**: displays and allows tuning of evolutionary parameters
  - Population size, mutation rate, and fitness function weights (stability, speed, distance)
- **Physics**: displays physical parameters (mass, inertia, gravity)
- **PID Control**: shows current gains of the outer control loop
- **Neural Network**: shows network input/hidden/output sizes

**Notes:**
- PSO optimization runs in a background thread (`trainNet`) during execution and periodically updates the best weights.
- Metrics such as “generation”, “fitness”, etc. are printed to the Processing console and shown in the menu.

## Quick Configuration (in the Code)
Main configurable points are in the `setup()` of `PSOFollowLineDeepLearning04.pde`:

- Initial PID gains:
  ```java
  m[0].setPID(200, 0, 8)
  m[0].malha_ext_esq = new Control(200, 0, 8);
  m[0].malha_ext_dir = new Control(200, 0, 8);
  ```
- Neural network (sizes):
  ```java
  int[] deepLearning = {11, 5, 2}; // input, hidden, output
  int[] temporalLayers = {2, 0, 0}; // temporal layers for RNN
  ```
- Dynamics/time:
  ```java
  m[0].setVelMax(3.45); // (m/s)
  m[0].setVariacaoTempo(1000); // (µs per step)
  ```
- Evolution algorithm parameters (class DNA in `AI.pde`):
  - `tamPopulacao` (population size), `mutacao` (rate), fitness weights (`pesoVel`, `pesoDist`, `pesoEstabilidade`), time and distance limits.

## Project Structure (Main Files)
- `PSOFollowLineDeepLearning04.pde`: main sketch; loads track, initializes robot, network, PSO, and simulation loop
- `AI.pde` (DNA class): population evaluation, fitness calculation, and PSO iteration over network weights
- `Robot.pde` (Movimento class): robot logic, line reading, PID, neural control, odometry, wheel dynamics
- `Dynamic.pde`: robot physics (simplified), velocities, integration
- `Deep_Learning.pde`: implementation of Neural_Network and auxiliary matrix operations
- `Algebra.pde`: matrix/vector utilities
- `Heuristic.pde`: PSO implementation (population, individuals, pose updates)
- `Interface.pde`: top menu, tabs, and sliders
- `Controles.pde`: Control class (controller parameters/state) and logs
- `Botoes.pde`: button/clickable area components used by the menu
- `data/`: static assets (images) used by the sketch
- `controle/`: optional exported logs (velocity, distance, time) when enabled

## Logs and Export
When enabled (see `Control.save_data`), the simulator saves time-series logs to `controle/`:
- `vel_linear.txt`
- `vel_estrategia.txt`
- `dist_percorrida.txt`
- `tempo.txt`

## Tips
- If the window does not open or sound fails, check if the “Sound” library is installed in Processing 4.
- To speed up the simulation, adjust `velTempo` in `PSOFollowLineDeepLearning04.pde`.
- If the robot “disappears” from the track (leaves the image), reposition it with the right mouse button.

## License
Distributed under the MIT license. See the `LICENSE` file for details.

## Credits
Author: Jakson Almeida
