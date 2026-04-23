## Legacy scripts (do not rely on them for day-to-day work)

The package `moonmapper_description` contains two scripts:

- `scripts/archive_gazebo.sh`
- `scripts/restore_gazebo.sh`

Historically these scripts **moved source files in/out of** `src/moonmapper_description/_archive/gazebo/`
and **overwrote** `package.xml` / `CMakeLists.txt` to switch stacks.

### Why this is risky

- It makes the workspace **stateful** (two developers can have different trees without git diffs).
- It can break launch files/config paths unexpectedly.
- It is hard to support/teach to new student teams.

### Current behavior (safe-by-default)

These scripts now exit early unless you explicitly opt into the destructive behavior:

```bash
FORCE_DESTRUCTIVE=1 bash src/moonmapper_description/scripts/archive_gazebo.sh
FORCE_DESTRUCTIVE=1 bash src/moonmapper_description/scripts/restore_gazebo.sh
```

Recommended approach going forward: keep Gazebo and Unity assets **in-tree** and select the desired
stack through launch files / parameters, not by moving files around.

