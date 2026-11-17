# TORCS Behavioral Cloning

An autonomous driver for the [TORCS](http://torcs.sourceforge.net/) racing simulator built with **behavioral cloning**: a human drives the car with the keyboard, every (sensor reading, driver action) pair is recorded, and a **k-Nearest Neighbors** classifier then reproduces the driver's behaviour by predicting the action to perform from the current sensor readings.

The project was developed as the final project of the university course *Intelligenza Artificiale: Metodi ed Applicazioni* (2023-2024), whose contest asked every team to complete a lap on its own in the shortest possible time.

## How it works

```
 TORCS (scr_server) ──UDP──▶ Client ──▶ SimpleDriver.control(sensors)
                                              │
              normalize 22 features           ▼
   (speed, track position, 19 track      DrivingData
    edge sensors, angle to track axis)        │
                                              ▼
                                  NearestNeighbor (KD-tree, k = 3)
                                              │  predicted class (0-5)
                                              ▼
                       action (steer / throttle / brake) + recovery routines
                                              │
                                              ▼
                                  Action ──UDP──▶ TORCS
```

1. **Data acquisition** (`train` mode). A small Swing window ([`CharReader`](src/scr/CharReader.java)) captures the arrow keys while a human drives. At every simulation step [`SimpleDriver`](src/scr/SimpleDriver.java) appends a CSV row with the normalized sensor readings and the class of the key being pressed.
2. **Classification** (`drive` mode). The dataset is loaded into a KD-tree ([`KDTree`](src/scr/KDTree.java), [`NearestNeighbor`](src/scr/NearestNeighbor.java)). At every step the current readings are normalized in the same way and the 3 nearest recorded samples vote for the class.
3. **Control.** Each class is mapped to a fixed set of commands (see below). Gear shifting and clutch are handled automatically.
4. **Recovery.** Two routines override the classifier when needed: one steers the car back when it leaves the track (`|trackPos| > 1.2`), the other detects a stuck car (angle to the track axis above 30° for 40 consecutive steps) and drives it back parallel to the track axis, in reverse if necessary.

### Dataset

[`data/Torcs_data.csv`](data/Torcs_data.csv) contains 21,243 samples with 22 normalized features and the class label, separated by `;`:

| Column(s) | Content | Normalization |
|---|---|---|
| 1 | Speed along the car axis | `[0, 310]` km/h → `[0, 1]` |
| 2 | Track position (`-1` right edge, `+1` left edge) | `[-1, 1]` → `[0, 1]` |
| 3-21 | 19 track edge sensors (angles −90°…+90°) | `[0, 200]` m → `[0, 1]` |
| 22 | Angle between the car and the track axis | `[-π, π]` → `[0, 1]` |
| 23 | Class label | see below |

> The header labels the last feature column `TrackEdgeSensors[19]`, but the track edge sensors are only 19 (`0`-`18`): that column is the angle to the track axis.

| Class | Key | Action | Samples |
|---|---|---|---|
| 0 | ↑ | accelerate | 12,971 |
| 1 | ↓ | brake | 731 |
| 2 | ← | steer left | 4,236 |
| 3 | → | steer right | 1,181 |
| 4 | Shift | reverse | 0 |
| 5 | – (no key) | default (moderate throttle) | 2,124 |

## Repository structure

```
├── src/scr/          Java sources (package `scr`)
├── data/             Recorded dataset (Torcs_data.csv)
├── scripts/          build / run helpers for Linux-macOS and Windows
└── README.md
```

## Requirements

- **JDK 14 or newer** (tested with JDK 17)
- **TORCS** with the SCR (Simulated Car Racing championship) patch, which provides the `scr_server` bot

## Build

```bash
# Linux / macOS / Git Bash
scripts/build.sh
```

```bat
:: Windows
scripts\build.bat
```

or directly, from the repository root:

```bash
javac -encoding UTF-8 -d out src/scr/*.java
```

## Run

All commands must be launched **from the repository root**, so that the default dataset path (`data/Torcs_data.csv`) resolves correctly.

1. Start the client (it waits for TORCS):

   ```bash
   scripts/run.sh drive          # Linux / macOS / Git Bash
   ```
   ```bat
   scripts\run.bat drive         :: Windows
   ```

   which is equivalent to:

   ```bash
   java -Dfile.encoding=UTF-8 -cp out scr.Client scr.SimpleDriver host:localhost port:3001
   ```

2. Start TORCS and select **Race → Quick Race → Configure Race**, pick a track (the dataset was driven on *CG Speedway number 1*), select **scr_server 1** as the only driver, set the number of laps, confirm and click **New Race**.

### Recording new data

```bash
scripts/run.sh train
```

A small window titled *Continuous Character Reader* opens: keep it focused while driving and use **↑** accelerate, **↓** brake, **←** / **→** steer, **Shift** reverse. The recording is written to `data/recorded_session.csv`, which is **overwritten at the start of every session**. To drive with a new dataset:

```bash
JAVA_OPTS="-Dtorcs.data=data/recorded_session.csv" scripts/run.sh drive
```

### Configuration

`SimpleDriver` is configured through JVM system properties:

| Property | Default | Meaning |
|---|---|---|
| `torcs.mode` | `drive` | `drive` (k-NN autonomous driving) or `train` (record keyboard driving) |
| `torcs.data` | `data/Torcs_data.csv` | dataset used by the k-NN in `drive` mode |
| `torcs.output` | `data/recorded_session.csv` | recording file in `train` mode |

The standard SCR client options (`host:`, `port:`, `id:`, `verbose:`, `maxEpisodes:`, `maxSteps:`, `stage:`, `trackName:`) are documented in [`Client.java`](src/scr/Client.java).

## Design notes and known limitations

- The classifier predicts one of six **discrete actions**, each mapped to fixed command values; no continuous steering or throttle is learned.
- The "no key pressed" class is replayed as moderate throttle, and the dataset contains no reverse samples (class 4): reversing is performed only by the stuck-car recovery routine.
- Features are normalized with fixed ranges and are not weighted; `k` is fixed to 3.
- In `SimpleDriver.sterzaDX`, the low-speed branch uses a steering of `+0.55` (towards the left) instead of `-0.55`. It is kept unchanged to preserve the behaviour of the driver that was submitted for the contest.

## Authors and credits

Group project by:

- Salvatore Francesco Tartaglione
- Christian Rosa
- Luigi Veniero

**What is original and what was provided.** The client / simulator interface comes from the SCR championship software (Daniele Loiacono et al.), and all the remaining classes were supplied by the course (`Action`, `CharReader`, `Client`, `Controller`, `DeadSimpleSoloController`, `DrivingData`, `KDTree`, `MessageBasedSensorModel`, `MessageParser`, `NearestNeighbor`, `SensorModel`, `SocketHandler`). They did not need any modification and are included here unchanged. **[`SimpleDriver.java`](src/scr/SimpleDriver.java) is the only class that was modified (implemented) by the authors**, together with the recording of the behavioral cloning dataset in [`data/Torcs_data.csv`](data/Torcs_data.csv).
