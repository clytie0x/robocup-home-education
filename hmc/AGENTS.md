# AGENTS.md

Rules for coding agents (Claude Code, Codex and others) working in `hmc/`.

## Hard rules

- Python only, no Rust. All repo content is in English.
- Keep the code readable for beginners. The team is still learning. Use small functions and plain names, and comment the why.
- Simulation only. Do not write hardware, deploy, flash, SSH, serial, USB or GPIO code.
- Use the standard library only. matplotlib is the one optional extra, and it only draws the plot.
- Keep the module interfaces ROS-free, so each module can later be wrapped as a ROS 2 Jazzy node on Ubuntu 24.04. Do not add ROS yet.
- Install packages only into `hmc/.venv` with pip.
- Do not delete files or branches. Do not force-push. Do not push to `main`, do not merge, and do not change the repo's visibility.
- Do not touch the other teams' folders (`voice/`, `arm/`).
- Claim only what you actually ran. Paste the real pytest summary line and the real scenario output.

## Source of truth

The rulebook is `docs/reference/2025.1_rulebook.pdf`, section 5.1, pp. 39-41.
`docs/tasks/5.1-help-me-carry.md` restates it. If the two disagree, the rulebook wins. Then fix the spec.

## Before you say you are done

```bash
cd hmc
pytest
python -m hmc.run
```

`pytest` must pass and `python -m hmc.run` must end in state `done`.
If you change the behaviour, update the acceptance-test table in the spec. Then regenerate the evidence folder for the current slice (`docs/evidence/help-me-carry-slice2/`) and its `SHA256SUMS`.

## Good to know

- The task code (`task.py`) must never read the car position, and neither `task.py` nor `navigation.py` may read obstacles or people from the world. Tests check this.
- `docs/evidence/` has a `.gitattributes` with `-text`, so git keeps the files byte-for-byte and the checksums stay valid.
- On some Windows setups Python cannot write to `%TEMP%`, so pytest and pip put temp folders in the current directory. If that happens, run `pytest --basetemp=out/pytest-tmp` and set `TMP` to a writable folder before `pip install`.
