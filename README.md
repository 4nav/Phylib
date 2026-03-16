# Phylib

A real-time 2D physics engine built from scratch in C++17 with OpenGL 3.3 Core Profile.  
Demonstrates rigid body dynamics, impulse-based collision, Hooke's Law springs, single and double pendulums — all rendered live at 60+ FPS.

![Demo](demo.gif)

---

## Features

| Category | Details |
|---|---|
| **Rigid Bodies** | Position, velocity, mass, AABB shape |
| **Integration** | Semi-implicit (Symplectic) Euler — energy-stable |
| **Gravity** | Global constant force; toggle on/off at runtime |
| **Collision Detection** | Broad & narrow phase AABB overlap test |
| **Collision Response** | Impulse-based resolution with restitution coefficient |
| **Friction** | Coulomb friction model applied on every collision |
| **Boundary Walls** | Bodies bounce off all four screen edges |
| **Simple Pendulum** | `α = -(g/L)sin(θ)` equation, damping, spawn at runtime |
| **Double Pendulum** | Full Lagrangian equations, RK4 integration, chaotic motion |
| **Trail Effect** | Fading 400-point trace of the double pendulum tip |
| **Springs** | Hooke's Law + viscous damping; anchored or body-to-body |
| **Spring Chain** | 4-mass vertical mobile with lateral oscillation |
| **Rendering** | VAO/VBO circle, quad, procedural zigzag coil, line strip |
| **Interactive** | Keyboard shortcuts + mouse click + scroll wheel |

---

## Dependencies

| Library | Purpose | Version |
|---|---|---|
| [GLFW](https://www.glfw.org/) | Window creation, OpenGL context, input | 3.3+ (auto via CMake FetchContent) |
| [GLM](https://github.com/g-truc/glm) | Vector/matrix math (header-only) | 0.9.9.8 (auto via CMake FetchContent) |
| **GLEW** | OpenGL 3.3 function loader | System install required |
| **OpenGL** | GPU rendering API | 3.3 Core Profile |
| **CMake** | Build system | 3.16+ |

---

## Build Instructions

### Linux (Ubuntu / Debian)

```bash
# Install system dependencies
sudo apt update
sudo apt install cmake build-essential libglew-dev libgl1-mesa-dev

# Clone and build
git clone https://github.com/yourusername/Phylib.git
cd Phylib
mkdir build && cd build
cmake ..          # Downloads GLFW + GLM automatically
make -j$(nproc)

# Run
./Phylib
```

### macOS

```bash
# Install dependencies via Homebrew
brew install cmake glew

# Clone and build
git clone https://github.com/yourusername/Phylib.git
cd Phylib
mkdir build && cd build
cmake ..
make -j$(sysctl -n hw.logicalcpu)

./Phylib
```

### Windows (Visual Studio / vcpkg)

```powershell
# Install GLEW via vcpkg
vcpkg install glew:x64-windows

# Configure with cmake (point to your vcpkg toolchain)
cmake -B build -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release

.\build\Release\Phylib.exe
```

> **Note:** GLFW and GLM are downloaded automatically by CMake FetchContent — no manual install needed for those.

---

## Controls

| Key / Action | Effect |
|---|---|
| `Space` | Pause / Resume the simulation |
| `R` | Reset all objects to initial conditions |
| `G` | Toggle gravity on / off |
| `P` | Spawn a new simple pendulum (random position) |
| `S` | Spawn a new spring-mass system (random position) |
| `Left Click` | Spawn a rigid body at the mouse cursor |
| `Scroll Up / Down` | Increase / decrease gravity strength |
| `Esc` | Quit |

---

## Physics Concepts

### Semi-Implicit Euler Integration

The simulation advances each body using **Symplectic Euler**:

```
velocity += acceleration × dt    // update velocity first
position += velocity × dt        // use the NEW velocity
```

Unlike forward Euler, this conserves energy in oscillatory systems (springs, pendulums) making it suitable for real-time physics.

---

### AABB Collision Detection

Each body is represented as an **Axis-Aligned Bounding Box** (rectangle). Two boxes overlap when they intersect on *both* the X and Y axes simultaneously:

```
overlapX = (halfW_A + halfW_B) - |centreA.x - centreB.x|
overlapY = (halfH_A + halfH_B) - |centreA.y - centreB.y|
```

The axis with *minimum* overlap determines the contact normal.

---

### Impulse-Based Collision Response

On detection, an instantaneous **impulse** `j` is applied along the contact normal:

```
j = -(1 + e) × v_relative · n / (1/mA + 1/mB)
```

where `e` is the **restitution coefficient** (0 = inelastic, 1 = perfectly elastic).  
A Coulomb friction impulse is then applied tangentially: `|j_t| ≤ μ × j_n`.

**Positional correction** (Baumgarte stabilisation) nudges overlapping bodies apart each frame to prevent sinking under gravity.

---

### Hooke's Law Springs

Each spring applies a restoring force proportional to its extension:

```
F_spring  = k × (currentLength − restLength)
F_damping = −d × (vA − vB) · direction
```

Equal and opposite forces are applied to both endpoints. The damping term `d × relative_velocity` prevents infinite oscillation.

---

### Simple Pendulum

The angular acceleration of a pendulum bob follows:

```
α = −(g / L) × sin(θ)
```

For large angles this is fully non-linear (not the "small-angle" approximation).  
Angular velocity is damped each frame to model air resistance.

---

### Double Pendulum & Chaos

The double pendulum obeys the **Lagrangian equations of motion** — two coupled second-order nonlinear ODEs derived from energy conservation. The key property is **sensitive dependence on initial conditions**: infinitesimally different starting angles produce wildly different trajectories.

The engine integrates these equations with **4th-order Runge-Kutta (RK4)** to minimise numerical drift and preserve the chaotic behaviour accurately.

The trail of the tip bob visualises this chaotic attractor in real time.

---

## Project Structure

```
Phylib/
├── src/
│   ├── main.cpp          ← GLFW window, main loop, input callbacks
│   ├── PhysicsWorld      ← Simulation manager: owns all objects, runs steps
│   ├── RigidBody         ← Body state, semi-implicit Euler integration
│   ├── Collision         ← AABB test, impulse resolution, boundaries
│   ├── Pendulum          ← Simple + double pendulum (RK4)
│   ├── Spring            ← Hooke's Law with damping
│   ├── Renderer          ← OpenGL VAO/VBO drawing, zigzag springs, trails
│   └── Shader            ← GLSL compile/link, uniform setters
├── shaders/
│   ├── vertex.glsl       ← Orthographic transform
│   └── fragment.glsl     ← Uniform color output
├── CMakeLists.txt
└── README.md
```

---

## License

MIT License — free to use, modify, and distribute.
