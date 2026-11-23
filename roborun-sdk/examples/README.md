# Examples

Small, runnable examples of building on Roborun from your own files. [docs/BUILDING.md](../docs/BUILDING.md) explains
each extension point; `roborun new my-robot` writes a starter project with a driver, rules and a task client.

| File | What it shows | Run it |
| --- | --- | --- |
| `minimal_arm.py` | The smallest arm driver: an `ArmAdapter` with `read()` and `command()` | `python minimal_arm.py` (starts it on a station on a free port) |
| `folder_camera.py` | Your own camera: a class whose `read()` returns RGB images | `python folder_camera.py FOLDER_OF_IMAGES` |
| `keep_out_box.py` | A rules file: the gripper stays out of a box on the table | `python keep_out_box.py` |
| `robot_client.py` | A command line on `roborun_sdk.client.Robot`: tasks, live stream, Stop, a camera frame | `roborun python robot_client.py status` (needs `roborun up`) |
| `llm_tools.py` | The robot as an LLM's tools: specs (JSON Schema) and `call_tool()` for look, run a task, arm state, Stop | `roborun python llm_tools.py tools` |

Run them with `roborun python FILE`: Roborun's own Python, where `roborun_sdk` is installed. `roborun docs --examples DIR`
copies them out of an installed Roborun.

