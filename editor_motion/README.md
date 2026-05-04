# Robot Motion Editor

**Robot Motion Editor** is a web-based tool developed by **Project Instinct group** for visualizing, editing, and smoothing robot motion data directly in the browser. It combines a 3D robot visualizer (Three.js) with a custom 2D curve editor to modify joint angles, base positions, and base rotations frame-by-frame.

## 🌟 Features

*   **3D Visualization**: Load and render URDF robot models in real-time.
*   **Motion Loading**: Import `.npz` motion files containing joint positions and base trajectories.
*   **Channel Editing**:
    *   **Joints**: Edit individual joint angles.
    *   **Base Position**: Edit Global X, Y, Z coordinates.
    *   **Base Rotation**: Edit Roll, Pitch, and Yaw (automatically converted to/from Quaternions).
*   **Curve Editor**:
    *   Interactive 2D canvas to visualizing motion curves.
    *   Click-and-drag to modify keyframes.
    *   Scrub through the timeline to see immediate 3D feedback.
*   **Smoothing**: Apply a moving average filter to smooth out jittery motion curves.
*   **Export**: Save modified motion data back to a `.npz` file in this format.

## 🛠️ Tech Stack

*   **Frontend**: HTML5, JavaScript (ES6 Modules)
*   **3D Engine**: [Three.js](https://threejs.org/)
*   **Loaders**: `urdf-loader`
*   **Backend**: Python (Flask) – *Required for parsing and saving npz files.*

## 📂 Project Structure

```text
/robot-motion-editor
├── app.py              # Python Backend (Flask)
├── templates/
│   └── index.html      # Main Editor Interface (HTML/JS)
├── static/
│   ├── robot.urdf      # Place your robot URDF here
│   └── meshes/         # Place robot mesh files here (stl/obj)
└── README.md           # This file
```

## 🚀 Quick Start

### 1. Prerequisites
You need **Python 3.8+** installed.

### 2. Install Dependencies
Install the required Python libraries for the backend.

```bash
pip install flask numpy
```

### 3. Setup Robot Files
1.  Place your robot's `.urdf` file inside the `static/` folder.
2.  Ensure any mesh files referenced in the URDF are also located inside `static/` (and paths in the URDF match, e.g., `filename="package://meshes/..."` usually maps to `static/meshes/...`).

### 4. Run the Application
Start the Flask server:

```bash
python app.py
```

Open your browser and navigate to:  
`http://127.0.0.1:5000`

## 📖 User Guide

1.  **Load Robot**:
    *   Enter the relative file path (w.r.t `static/`) of your URDF (e.g., `unitree_g1/urdf/g1_29dof_torsobase_popsicle.urdf`) in the sidebar text box.
    *   Click **"1. Load Robot"**.
2.  **Load Motion**:
    *   Click "Choose File" and select a valid `.npz` file.
    *   The file must contain keys like `joint_pos`, `base_pos_w`, and `base_quat_w`.
3.  **Edit Curves**:
    *   Select a channel from the sidebar list (Joints, Base Position, or Rotation).
    *   **Click & Drag** points on the graph at the bottom to change values.
    *   Use the **Slider** at the bottom to scrub through time.
4.  **Smooth**:
    *   If a curve looks noisy, click **Smooth Current Curve** to apply a filter.
5.  **Save**:
    *   Click **Download Motion NPZ** to save your changes.

## 📝 Data Format (NPZ)

The application expects `.npz` files with the following NumPy arrays:

*   `joint_pos`: `(num_frames, num_joints)` - Float array of joint angles.
*   `base_pos_w`: `(num_frames, 3)` - Global position formatted as `[x, y, z]`.
*   `base_quat_w`: `(num_frames, 4)` - Global orientation formatted as `[w, x, y, z]`.
*   `joint_names`: List of strings matching the URDF joint names.
*   `framerate`: The framerate of the motion, if not provided, will be exported as 30.
