# 24780 Final Project

CMU 24-780 C++ final project.

## Opponent Logic test (Anthony)

Code: `src/opponent.h`, `src/opponent.cpp`. Test: `tests/opponent_test.cpp` (has its own `main`, so build it
separately from `src/main.cpp` and `src/mobile_robot_test.cpp`).

The test runs 16 console checks first (they print `[PASS]` / `[FAIL]` and a total), then opens a scenario
window. The checks use a FakeRobot; the window can also run Harrison's real `MobileRobot`.

### Compile and run

Windows (MinGW g++), from the repo root:

```
g++ -std=c++17 -O2 -Isrc -Ilib tests/opponent_test.cpp src/opponent.cpp src/shared.cpp src/mobile_robot.cpp lib/fssimplewindow.cpp -x c++ lib/ysglfontdata.c -o opponent_test.exe -lopengl32 -lglu32 -lgdi32 -luser32 -lwinmm -limm32
./opponent_test.exe          # checks, then the window
./opponent_test.exe check    # checks only
```

(`-x c++` compiles `ysglfontdata.c` as C++; newer gcc rejects it as C.)

Visual Studio: make a project with `tests/opponent_test.cpp`, `src/opponent.cpp`, `src/shared.cpp`,
`src/mobile_robot.cpp`, `lib/fssimplewindow.cpp`, `lib/ysglfontdata.c`, and add `src` and `lib` to the include
directories. Don't add `src/main.cpp` or `src/mobile_robot_test.cpp` (they have their own `main`).

macOS: same source files, but use the Mac FsSimpleWindow files instead of `lib/fssimplewindow.cpp`
(the ones kept locally in `lib/`, see `.gitignore`), plus `-framework Cocoa -framework OpenGL`.

### Keys in the scenario window

| Key | Action |
|---|---|
| `1` | Fast shot at the robot's goal (expect DEFEND: block at the defense line) |
| `2` | Slow puck on the robot's half (expect ATTACK, strike, RECOVER) |
| `3` | Same, with the human mallet blocking the middle (expect a bank shot) |
| `4` | Puck stays on the human's half (expect GUARD, shifted toward the puck) |
| `SPACE` | Replay the current scenario |
| `T` | Switch robot: FakeRobot / MobileRobot |
| `E` / `M` / `H` | Difficulty: Easy / Medium / Hard |
| `ESC` | Quit |

On screen: mode, robot and difficulty at the top; the scenario, the expected result and the actual result in
the middle. Gray line = predicted puck path, red line = defense line, orange ring = robot's current target,
in ATTACK green/pink lines = usable/blocked candidate shots and the thick green line = chosen shot.
