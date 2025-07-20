
# swarm-of-drones

##  What the Code Does

The C code simulates a swarm of drones performing a search mission:

###  Key Behaviors:
- **Swarm Logic**: Drones adjust motion using *cohesion*, *separation*, and *alignment* rules.
- **Target Zones**: There are 3 zones in 3D space where drones are supposed to search.
- **Search Action**:
  - If a drone gets close to a target zone, it stops and begins "scanning" for a few timesteps.
  - Once a zone is scanned, it’s marked as “searched.”
- **Mission Completion**: When all target zones are scanned by any drone, the mission ends with a message.

---

##  How to Test on Linux

###  Step 1: Save the Code
Create a file for the code:

```bash
nano drone_swarm.c
```

Paste the code into the editor, then press `CTRL+O`, `ENTER`, and `CTRL+X` to save and exit.

---

### ️ Step 2: Compile the Program
Use `gcc` to compile the C file:

```bash
gcc drone_swarm.c -o drone_swarm -lm
```

- `-lm` links the math library (`sqrt` is used in the code).
- `drone_swarm` is the output executable.

---

###  Step 3: Run the Simulation

```bash
./drone_swarm
```

This will print the drone positions and statuses (e.g. `[Searching]`) for each timestep. It continues until all targets are searched.

---

###  Optional Add‑Ons

You can redirect the output to a log file and analyze the data:

```bash
./drone_swarm > output.log
```

Then use tools like `grep` or `awk` to summarize the results:

```bash
grep "Mission complete" output.log
```
